#ifndef UNITS_H
#define UNITS_H


#define CYCLE_TIME_MS 10
#define CYCLE_TIME_US MsToUs(CYCLE_TIME_MS)

/* macro to convert a milliseconds time to microseconds (for rtc functions) */
#define MsToUs(x) ((x)*(1000ul))

#define MsToCycles(x) ((x)/(CYCLE_TIME_MS))

#endif // UNITS_H
