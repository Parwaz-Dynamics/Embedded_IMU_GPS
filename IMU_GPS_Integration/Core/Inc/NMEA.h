/*
 * NMEA.h
 *
 *  Pure NMEA parser (stateless).
 *  - latitude/longitude decoded as TRUE signed decimal degrees (double)
 *  - fractional seconds preserved + seconds-of-day timestamp
 *  - HDOP extracted from GGA
 *  - RMC speed converted to m/s; course kept in degrees (true)
 *
 *  Velocity (vN/vE/vD) and the consolidated output now live in sensors.c,
 *  since they are application/sensor-layer concerns, not protocol parsing.
 */

#ifndef INC_NMEA_H_
#define INC_NMEA_H_

typedef struct {
	int   hour;
	int   min;
	float sec;        // includes fractional part (e.g. 19.50)
	double secOfDay;  // seconds since 00:00:00 UTC
} TIME;

typedef struct {
	double latitude;  // signed decimal degrees, South is negative
	char   NS;
	double longitude; // signed decimal degrees, West is negative
	char   EW;
} LOCATION;

typedef struct {
	float altitude;   // metres above mean sea level
	char  unit;
} ALTITUDE;

typedef struct {
	int Day;
	int Mon;
	int Yr;
} DATE;

typedef struct {
	LOCATION lcation;
	TIME     tim;
	int      isfixValid;
	ALTITUDE alt;
	int      numofsat;
	float    hdop;     // horizontal dilution of precision
} GGASTRUCT;

typedef struct {
	DATE  date;
	float speed;       // metres per second (converted from knots)
	float course;      // degrees, true track over ground
	int   isValid;
} RMCSTRUCT;

typedef struct {
	GGASTRUCT ggastruct;
	RMCSTRUCT rmcstruct;
} GPSSTRUCT;

/* Decode a single GGA sentence. Returns 0 on success, 1 if no valid fix. */
int decodeGGA(char *GGAbuffer, GGASTRUCT *gga);

/* Decode a single RMC sentence. Returns 0 on success, 1 if data invalid. */
int decodeRMC(char *RMCbuffer, RMCSTRUCT *rmc);

#endif /* INC_NMEA_H_ */
