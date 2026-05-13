#include "runtime_config.h"

#include "IO_EEPROM.h"
#include "IO_RTC.h"

#include "config/torque_config.h"

/**************************************************************************
*                          P R I V A T E    T Y P E S
**************************************************************************/

// these all have to be sbyte2 here so the general function works and shi
typedef struct {
    sbyte2 max_torque;
    sbyte2 motor_direction;
    sbyte2 regen_enabled;
} RuntimeConfig_Data_t;

typedef struct {
    ubyte1  id;
    sbyte2* value;
    sbyte2  default_value;
    sbyte2  min_value;
    sbyte2  max_value;
} RuntimeConfig_ParamDesc_t;

typedef enum {
    EEPROM_STATE_READ_TRIGGER = 0,
    EEPROM_STATE_READ_WAIT,
    EEPROM_STATE_READY,
    EEPROM_STATE_WRITE_WAIT,
} RuntimeConfig_EepromState_t;

/**************************************************************************
*                         P R I V A T E    D E F I N E S
**************************************************************************/

/* EEPROM preload docs note 64 bytes reserved for internal use.
   Stay above that to be safe across driver variants. */
#define RUNTIME_CFG_EEPROM_OFFSET     ((ubyte2)64)

#define RUNTIME_CFG_MAGIC             ((ubyte4)0x52434647UL) /* 'RCFG' */
#define RUNTIME_CFG_VERSION           ((ubyte2)1)

#define RUNTIME_CFG_MAX_PARAMS        ((ubyte2)32)

/* Header:
   u32 magic
   u16 version
   u16 record_count
   u32 checksum
*/
#define RUNTIME_CFG_HEADER_LEN        ((ubyte2)12)

/* Record:
    u8 param_id
    i16 value
*/
#define RUNTIME_CFG_RECORD_LEN        ((ubyte2)3)

/* Total EEPROM blob length (fixed). */
#define RUNTIME_CFG_EEPROM_LEN        ((ubyte2)(RUNTIME_CFG_HEADER_LEN + (RUNTIME_CFG_RECORD_LEN * RUNTIME_CFG_MAX_PARAMS)))

/* Coalesce writes a little to avoid repeated EEPROM cycles on bursty CAN sets. */
#define RUNTIME_CFG_MIN_WRITE_PERIOD_US    ((ubyte4)50000) /* 50 ms */

/**************************************************************************
*                       P R I V A T E    D A T A
**************************************************************************/

static RuntimeConfig_Data_t runtime_cfg;

static RuntimeConfig_EepromState_t eeprom_state;
static ubyte1 eeprom_buf[RUNTIME_CFG_EEPROM_LEN];

static bool write_pending;
static ubyte4 last_write_trigger_ts;

/* Request-based config TX trigger */
static bool config_tx_pending;

static RuntimeConfig_ParamDesc_t param_descs[] = {
    {
        .id = RUNTIME_PARAM_MAX_TORQUE,
        .value = &runtime_cfg.max_torque,
        .default_value = MAX_TORQUE_DEFAULT,
        .min_value = 0,
        .max_value = 230,
    },
    {
        .id = RUNTIME_PARAM_MOTOR_DIRECTION,
        .value = &runtime_cfg.motor_direction,
        .default_value = MOTOR_DIRECTION_DEFAULT,
        .min_value = MOTOR_REVERSE,
        .max_value = MOTOR_FORWARDS,
    },
    {
        .id = RUNTIME_PARAM_REGEN_ENABLED,
        .value = &runtime_cfg.regen_enabled,
        .default_value = REGEN_ENABLED_DEFAULT,
        .min_value = 0,
        .max_value = 1,
    },
};

#define PARAM_COUNT ((ubyte2)(sizeof(param_descs) / sizeof(param_descs[0])))

/**************************************************************************
*                  P R I V A T E    F U N C T I O N S
**************************************************************************/

static RuntimeConfig_ParamDesc_t* FindParam(const RuntimeParamId_t param_id)
{
    for (ubyte2 i = 0; i < PARAM_COUNT; i++) {
        if (param_descs[i].id == param_id) {
            return &param_descs[i];
        }
    }
    return NULL;
}

static sbyte2 ClampI16(const sbyte4 v, const sbyte2 min_v, const sbyte2 max_v)
{
    if (v < min_v) {
        return min_v;
    }
    if (v > max_v) {
        return max_v;
    }
    return (sbyte2)v;
}

static void ApplyDefaults(void)
{
    for (ubyte2 i = 0; i < PARAM_COUNT; i++) {
        *param_descs[i].value = ClampI16(param_descs[i].default_value,
                        param_descs[i].min_value,
                        param_descs[i].max_value);
    }
}

static ubyte2 ReadU16LE(const ubyte1* const p)
{
    return (ubyte2)((ubyte2)p[0] | ((ubyte2)p[1] << 8));
}

static ubyte4 ReadU32LE(const ubyte1* const p)
{
    return (ubyte4)((ubyte4)p[0] |
                    ((ubyte4)p[1] << 8) |
                    ((ubyte4)p[2] << 16) |
                    ((ubyte4)p[3] << 24));
}

static void WriteU16LE(ubyte1* const p, const ubyte2 v)
{
    p[0] = (ubyte1)(v & 0xFF);
    p[1] = (ubyte1)(v >> 8);
}

static void WriteU32LE(ubyte1* const p, const ubyte4 v)
{
    p[0] = (ubyte1)(v & 0xFF);
    p[1] = (ubyte1)((v >> 8) & 0xFF);
    p[2] = (ubyte1)((v >> 16) & 0xFF);
    p[3] = (ubyte1)((v >> 24) & 0xFF);
}

static void WriteI16LE(ubyte1* const p, const sbyte2 v)
{
    WriteU16LE(p, (ubyte2)v);
}

static sbyte2 ReadI16LE(const ubyte1* const p)
{
    return (sbyte2)ReadU16LE(p);
}

/* FNV-1a 32-bit over the entire blob; checksum field is treated as 0. */
static ubyte4 ComputeBlobChecksum(const ubyte1* const blob)
{
    ubyte4 hash = 2166136261UL;

    for (ubyte2 i = 0; i < RUNTIME_CFG_EEPROM_LEN; i++) {
        ubyte1 b = blob[i];

        /* Checksum field is at offset 8..11. */
        if ((i >= 8) && (i <= 11)) {
            b = 0;
        }

        hash ^= (ubyte4)b;
        hash *= 16777619UL;
    }

    return hash;
}

static void PackToEepromBlob(ubyte1* const blob)
{
    /* Header */
    WriteU32LE(&blob[0], RUNTIME_CFG_MAGIC);
    WriteU16LE(&blob[4], RUNTIME_CFG_VERSION);
    WriteU16LE(&blob[6], PARAM_COUNT);
    WriteU32LE(&blob[8], 0);

    /* Records (fixed slots). Fill unused slots with 0xFF-ish values for clarity. */
    ubyte2 offset = RUNTIME_CFG_HEADER_LEN;
    for (ubyte2 i = 0; i < RUNTIME_CFG_MAX_PARAMS; i++) {
        if (i < PARAM_COUNT) {
            blob[offset + 0] = param_descs[i].id;
            WriteI16LE(&blob[offset + 1], *param_descs[i].value);
        }
        else {
            blob[offset + 0] = 0xFF;
            WriteU16LE(&blob[offset + 1], 0xFFFF);
        }

        offset = (ubyte2)(offset + RUNTIME_CFG_RECORD_LEN);
    }

    const ubyte4 checksum = ComputeBlobChecksum(blob);
    WriteU32LE(&blob[8], checksum);
}

static bool UnpackFromEepromBlob(const ubyte1* const blob)
{
    const ubyte4 magic = ReadU32LE(&blob[0]);
    const ubyte2 version = ReadU16LE(&blob[4]);
    const ubyte2 record_count = ReadU16LE(&blob[6]);
    const ubyte4 stored_checksum = ReadU32LE(&blob[8]);

    if (magic != RUNTIME_CFG_MAGIC) {
        return FALSE;
    }

    if (version != RUNTIME_CFG_VERSION) {
        return FALSE;
    }

    if (record_count > RUNTIME_CFG_MAX_PARAMS) {
        return FALSE;
    }

    const ubyte4 computed_checksum = ComputeBlobChecksum(blob);
    if (computed_checksum != stored_checksum) {
        return FALSE;
    }

    /* Apply records (ignore unknown param IDs). */
    ubyte2 offset = RUNTIME_CFG_HEADER_LEN;
    for (ubyte2 i = 0; i < record_count; i++) {
        const ubyte1 pid = blob[offset + 0];
        const sbyte2 val = ReadI16LE(&blob[offset + 1]);
        RuntimeConfig_ParamDesc_t* desc = FindParam(pid);

        if (desc != NULL) {
            *desc->value = ClampI16(val, desc->min_value, desc->max_value);
        }

        offset = (ubyte2)(offset + RUNTIME_CFG_RECORD_LEN);
    }

    return TRUE;
}

/**************************************************************************
*                   P U B L I C    F U N C T I O N S
**************************************************************************/

void RuntimeConfig_Init(void)
{
    ApplyDefaults();

    eeprom_state = EEPROM_STATE_READ_TRIGGER;
    write_pending = FALSE;
    last_write_trigger_ts = 0;

    config_tx_pending = FALSE;
}

void RuntimeConfig_Task(void)
{
    const IO_ErrorType st = IO_EEPROM_GetStatus();
    const ubyte4 now = IO_RTC_GetTimeUS(0);

    switch (eeprom_state) {
        case EEPROM_STATE_READ_TRIGGER:
            if (st == IO_E_OK) {
                (void)IO_EEPROM_Read(RUNTIME_CFG_EEPROM_OFFSET, RUNTIME_CFG_EEPROM_LEN, eeprom_buf);
                eeprom_state = EEPROM_STATE_READ_WAIT;
            }
            break;

        case EEPROM_STATE_READ_WAIT:
            if (st == IO_E_OK) {
                if (!UnpackFromEepromBlob(eeprom_buf)) {
                    /* EEPROM invalid: keep defaults and write them back for determinism. */
                    write_pending = TRUE;
                }
                eeprom_state = EEPROM_STATE_READY;
            }
            break;

        case EEPROM_STATE_READY:
            if (write_pending && (st == IO_E_OK)) {
                if ((now - last_write_trigger_ts) >= RUNTIME_CFG_MIN_WRITE_PERIOD_US) {
                    PackToEepromBlob(eeprom_buf);
                    (void)IO_EEPROM_Write(RUNTIME_CFG_EEPROM_OFFSET, RUNTIME_CFG_EEPROM_LEN, eeprom_buf);
                    last_write_trigger_ts = now;
                    write_pending = FALSE;
                    eeprom_state = EEPROM_STATE_WRITE_WAIT;
                }
            }
            break;

        case EEPROM_STATE_WRITE_WAIT:
            if (st == IO_E_OK) {
                eeprom_state = EEPROM_STATE_READY;
            }
            break;

        default:
            eeprom_state = EEPROM_STATE_READ_TRIGGER;
            break;
    }
}

bool RuntimeConfig_IsEepromLoadComplete(void)
{
    return ((eeprom_state == EEPROM_STATE_READY) ||
            (eeprom_state == EEPROM_STATE_WRITE_WAIT));
}

bool RuntimeConfig_Set(const RuntimeParamId_t param_id, const sbyte2 value)
{
    RuntimeConfig_ParamDesc_t* desc = FindParam(param_id);

    if (desc == NULL) {
        return FALSE;
    }

    const sbyte2 clamped = ClampI16(value, desc->min_value, desc->max_value);

    if (*desc->value != clamped) {
        *desc->value = clamped;

        write_pending = TRUE;
        /* Any successful SET should be followed by an immediate config TX. */
        config_tx_pending = TRUE;
    }

    return TRUE;
}

bool RuntimeConfig_GetI32(const RuntimeParamId_t param_id, sbyte2* const out_value)
{
    if (out_value == NULL) {
        return FALSE;
    }

    RuntimeConfig_ParamDesc_t* const desc = FindParam(param_id);
    if (desc == NULL) {
        return FALSE;
    }

    *out_value = (sbyte2)*desc->value;
    return TRUE;
}

ubyte1 RuntimeConfig_GetMaxTorque(void)
{
    return runtime_cfg.max_torque;
}

bool RuntimeConfig_GetMotorDirection(void)
{
    return runtime_cfg.motor_direction;
}

bool RuntimeConfig_GetRegenEnabled(void)
{
    return runtime_cfg.regen_enabled;
}

bool RuntimeConfig_ConfigTxTrigger(void)
{
    if (config_tx_pending) {
        config_tx_pending = FALSE;
        return TRUE;
    }

    return FALSE;
}
