/*
 * NMEA.c
 *
 *  Pure NMEA parser (field-index based). No application state.
 *
 *   - lat/long converted to TRUE decimal degrees (deg + minutes/60), signed.
 *   - HDOP extracted.
 *   - Speed converted from knots to m/s.
 *   - Time keeps fractional seconds and is exposed as seconds-of-day (UTC).
 */

#include "NMEA.h"
#include "UARTRingBuffer.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#define KNOTS_TO_MPS 0.514444f

extern GPSDATA gpsData;

static enum {
	GPS_HUNT_DOLLAR,
	GPS_READ_SENTENCE
} gpsState = GPS_HUNT_DOLLAR;

static char gpsSentenceBuf[128];
static uint8_t gpsSentenceIdx = 0;

/* ---- helpers ----------------------------------------------------------- */

/* Copy the idx-th comma-separated field (0-based) into 'out'. Stops at ',',
 * '*' (start of checksum) or end of string. Returns chars copied. */
static int nmea_field(const char *s, int idx, char *out, int outsz)
{
	int field = 0;
	const char *p = s;

	while (field < idx && *p && *p != '*') {
		if (*p == ',') field++;
		p++;
	}

	int oi = 0;
	if (field == idx) {
		while (*p && *p != ',' && *p != '*' && oi < outsz - 1) {
			out[oi++] = *p++;
		}
	}
	out[oi] = '\0';
	return oi;
}

/* Convert an NMEA ddmm.mmmm / dddmm.mmmm string to decimal degrees. */
static double nmea_to_degrees(const char *field)
{
	if (field[0] == '\0') return 0.0;
	double ddmm    = atof(field);          // e.g. 4807.038
	int    deg     = (int)(ddmm / 100.0);  // 48
	double minutes = ddmm - deg * 100.0;   // 7.038
	return deg + minutes / 60.0;           // 48.1173
}

static void updateCourseVelocityComponents(GPSDATA *gps)
{
	if (gps == NULL) {
		return;
	}

	float courseRadians = gps->course * (3.14159265f / 180.0f);
	gps->velocity.vN = gps->speed * cosf(courseRadians);
	gps->velocity.vE  = gps->speed * sinf(courseRadians);
}

/* ---- GGA --------------------------------------------------------------- */

void GPS_ResetUpdateFlag(GPSDATA *gps)
{
	if (gps != NULL) {
		gps->updated = 0;
	}
}

int GPS_IsUpdated(const GPSDATA *gps)
{
	return (gps != NULL) ? gps->updated : 0;
}

void readGPS(void)
{
	while (IsDataAvailable()) {
		char c = (char) UART_Read();

		if (gpsState == GPS_HUNT_DOLLAR) {
			if (c == '$') {
				gpsSentenceBuf[0] = c;
				gpsSentenceIdx = 1;
				gpsState = GPS_READ_SENTENCE;
			}
		} else {
			if (c == '$') {
				gpsSentenceBuf[0] = c;
				gpsSentenceIdx = 1;
				continue;
			}

			if (gpsSentenceIdx < sizeof(gpsSentenceBuf) - 1) {
				gpsSentenceBuf[gpsSentenceIdx++] = c;
			}

			if (c == '\n') {
				gpsSentenceBuf[gpsSentenceIdx] = '\0';
				gpsSentenceIdx = 0;
				gpsState = GPS_HUNT_DOLLAR;

				char *star = strchr(gpsSentenceBuf, '*');
				if (star)
					*star = '\0';

				if (strstr(gpsSentenceBuf, "GGA") != NULL) {
					decodeGGA(gpsSentenceBuf, &gpsData);
				} else if (strstr(gpsSentenceBuf, "RMC") != NULL) {
					decodeRMC(gpsSentenceBuf, &gpsData);
				}
			}
		}
	}
}

int decodeGGA(char *GGAbuffer, GPSDATA *gps)
{
	char f[16];

	if (gps == NULL) {
		return 1;
	}

	/* field 6: fix quality (0 = no fix) */
	nmea_field(GGAbuffer, 6, f, sizeof(f));
	if (f[0] == '\0' || atoi(f) == 0) {
		gps->isFixValid = 0;
		return 1;
	}
	gps->isFixValid = 1;

	/* field 1: time hhmmss.sss (UTC) */
	nmea_field(GGAbuffer, 1, f, sizeof(f));
	double t  = atof(f);
	int    hh = (int)(t / 10000);
	int    mm = ((int)(t / 100)) % 100;
	double ss = fmod(t, 100.0);
	gps->time.hour        = hh;
	gps->time.minute      = mm;
	gps->time.seconds     = (float)ss;
	gps->time.secondsOfDay = hh * 3600.0 + mm * 60.0 + ss;

	/* fields 2,3: latitude + hemisphere */
	nmea_field(GGAbuffer, 2, f, sizeof(f));
	double lat = nmea_to_degrees(f);
	nmea_field(GGAbuffer, 3, f, sizeof(f));
	gps->location.latitudeHemisphere = f[0];
	if (f[0] == 'S') lat = -lat;
	gps->location.latitude = lat;

	/* fields 4,5: longitude + hemisphere */
	nmea_field(GGAbuffer, 4, f, sizeof(f));
	double lon = nmea_to_degrees(f);
	nmea_field(GGAbuffer, 5, f, sizeof(f));
	gps->location.longitudeHemisphere = f[0];
	if (f[0] == 'W') lon = -lon;
	gps->location.longitude = lon;

	/* field 7: number of satellites */
	nmea_field(GGAbuffer, 7, f, sizeof(f));
	gps->satelliteCount = atoi(f);

	/* field 8: HDOP */
	nmea_field(GGAbuffer, 8, f, sizeof(f));
	gps->hdop = (float)atof(f);

	/* fields 9,10: altitude (MSL) + unit */
	nmea_field(GGAbuffer, 9, f, sizeof(f));
	gps->altitude.altitude = (float)atof(f);
	nmea_field(GGAbuffer, 10, f, sizeof(f));
	gps->altitude.unit = f[0];
	gps->updated = 1;

	return 0;
}

/* ---- RMC --------------------------------------------------------------- */

int decodeRMC(char *RMCbuffer, GPSDATA *gps)
{
	char f[16];

	if (gps == NULL) {
		return 1;
	}

	/* field 2: status A = valid, V = invalid */
	nmea_field(RMCbuffer, 2, f, sizeof(f));
	if (f[0] != 'A') {
		gps->isValid = 0;
		return 1;
	}
	gps->isValid = 1;

	/* field 7: speed over ground in knots -> m/s */
	nmea_field(RMCbuffer, 7, f, sizeof(f));
	gps->speed = (f[0]) ? (float)(atof(f) * KNOTS_TO_MPS) : 0.0f;

	/* field 8: course over ground (deg, true). Blank when stationary. */
	nmea_field(RMCbuffer, 8, f, sizeof(f));
	gps->course = (f[0]) ? (float)atof(f) : 0.0f;
	updateCourseVelocityComponents(gps);

	/* field 9: date ddmmyy */
	nmea_field(RMCbuffer, 9, f, sizeof(f));
	int d = atoi(f);
	gps->date.day = d / 10000;
	gps->date.month = (d / 100) % 100;
	gps->date.year = d % 100;
	gps->updated = 1;

	return 0;
}
