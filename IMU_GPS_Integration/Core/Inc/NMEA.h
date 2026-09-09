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
	int    hour;
	int    minute;
	float  seconds;       // includes fractional part (e.g. 19.50)
	double secondsOfDay;  // seconds since 00:00:00 UTC
} GpsTime;

typedef struct {
	double latitude;            // signed decimal degrees, South is negative
	char   latitudeHemisphere;
	double longitude;           // signed decimal degrees, West is negative
	char   longitudeHemisphere;
} GpsLocation;

typedef struct {
	float vN;
	float vE;
} GpsVelocity;

typedef struct {
	float altitude;         // metres above mean sea level
	float geoidSeparation;  // metres, geoid separation (field 11)
	char  unit;
} GpsAltitude;

typedef struct {
	int day;
	int month;
	int year;
} GpsDate;

typedef struct {
	GpsLocation location;
	GpsTime     time;
	int         isFixValid;
	GpsAltitude altitude;
	int         satelliteCount;
	float       hdop;
	GpsDate     date;
	float       speed;
	float       course;
	GpsVelocity velocity;
	int         isValid;
	int         ggaUpdated;
	int         rmcUpdated;
	double      ggaTime;
	double      rmcTime;
	int         velocityValid;
} GPSDATA;

void GPS_ResetUpdateFlag(GPSDATA *gps);
int GPS_IsUpdated(const GPSDATA *gps);
void readGPS(void);

/* Decode a single GGA sentence. Returns 0 on success, 1 if no valid fix. */
int decodeGGA(char *GGAbuffer, GPSDATA *gps);

/* Decode a single RMC sentence. Returns 0 on success, 1 if data invalid. */
int decodeRMC(char *RMCbuffer, GPSDATA *gps);

#endif /* INC_NMEA_H_ */
