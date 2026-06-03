#ifndef LEARNING_MODE_H
#define LEARNING_MODE_H

#include "IO_Constants.h"

/*
 * Launch Control Learning helper.
 *
 * Collects wheel-slip statistics during three launch runs and derives a relative
 * grip score per curve plus a recommended slip target. There is no longitudinal
 * accelerometer onboard, so this produces a RELATIVE comparison between the
 * curves (not an absolute friction coefficient). All values are integer; slip is
 * stored as a ratio x1000 (1000 == 1.000, i.e. no slip).
 *
 * Memory: running statistics only (peak + running sum/count) — no sample buffers,
 * to stay within TTC60 RAM limits.
 */

#define LEARNING_RUN_COUNT 3   /* run 1 (Curve A), run 2 (Curve B), run 3 (Curve C) */

typedef struct {
    bool   active;             /* TRUE while a run is being sampled */
    ubyte1 current_run;        /* 1, 2, or 3 while sampling, 0 otherwise */

    ubyte2 peak_slip_x1000[LEARNING_RUN_COUNT];
    ubyte2 avg_slip_x1000[LEARNING_RUN_COUNT];
    ubyte4 sample_count[LEARNING_RUN_COUNT];
    ubyte4 slip_sum_x1000[LEARNING_RUN_COUNT];  /* running sum for the average */

    ubyte2 last_slip_x1000;    /* most recent computed slip (for abort checks) */

    ubyte1 best_curve;         /* 0 = Curve A, 1 = Curve B, 2 = Curve C */
    sbyte2 grip_score_x1000[LEARNING_RUN_COUNT];
    sbyte2 recommended_slip_x1000;
} LearningMode_Data_t;

void LearningMode_Init(void);

/* Begin sampling for run_number (1, 2, or 3). Clears that run's stats. */
void LearningMode_StartRun(ubyte1 run_number);

/* Stop sampling the active run (keeps accumulated stats). */
void LearningMode_StopRun(void);

/* Accumulate one slip sample for the active run from the current motor speed.
 * Returns the computed slip ratio x1000 (0 if not enough data). */
ubyte2 LearningMode_Update(sbyte2 motor_rpm);

/* Compute grip scores, best curve, and recommended slip from all runs, then
 * persist the results to runtime config (EEPROM). Call once after the last run. */
void LearningMode_ProcessResults(void);

const LearningMode_Data_t* LearningMode_GetData(void);

#endif // LEARNING_MODE_H
