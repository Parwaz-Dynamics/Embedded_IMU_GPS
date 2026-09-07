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
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#define KNOTS_TO_MPS 0.514444f

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

/* ---- GGA --------------------------------------------------------------- */

int decodeGGA(char *GGAbuffer, GGASTRUCT *gga)
{
	char f[16];

	/* field 6: fix quality (0 = no fix) */
	nmea_field(GGAbuffer, 6, f, sizeof(f));
	if (f[0] == '\0' || atoi(f) == 0) {
		gga->isfixValid = 0;
		return 1;
	}
	gga->isfixValid = 1;

	/* field 1: time hhmmss.sss (UTC) */
	nmea_field(GGAbuffer, 1, f, sizeof(f));
	double t  = atof(f);
	int    hh = (int)(t / 10000);
	int    mm = ((int)(t / 100)) % 100;
	double ss = fmod(t, 100.0);
	gga->tim.hour     = hh;
	gga->tim.min      = mm;
	gga->tim.sec      = (float)ss;
	gga->tim.secOfDay = hh * 3600.0 + mm * 60.0 + ss;

	/* fields 2,3: latitude + hemisphere */
	nmea_field(GGAbuffer, 2, f, sizeof(f));
	double lat = nmea_to_degrees(f);
	nmea_field(GGAbuffer, 3, f, sizeof(f));
	gga->lcation.NS = f[0];
	if (f[0] == 'S') lat = -lat;
	gga->lcation.latitude = lat;

	/* fields 4,5: longitude + hemisphere */
	nmea_field(GGAbuffer, 4, f, sizeof(f));
	double lon = nmea_to_degrees(f);
	nmea_field(GGAbuffer, 5, f, sizeof(f));
	gga->lcation.EW = f[0];
	if (f[0] == 'W') lon = -lon;
	gga->lcation.longitude = lon;

	/* field 7: number of satellites */
	nmea_field(GGAbuffer, 7, f, sizeof(f));
	gga->numofsat = atoi(f);

	/* field 8: HDOP */
	nmea_field(GGAbuffer, 8, f, sizeof(f));
	gga->hdop = (float)atof(f);

	/* fields 9,10: altitude (MSL) + unit */
	nmea_field(GGAbuffer, 9, f, sizeof(f));
	gga->alt.altitude = (float)atof(f);
	nmea_field(GGAbuffer, 10, f, sizeof(f));
	gga->alt.unit = f[0];

	return 0;
}

/* ---- RMC --------------------------------------------------------------- */

int decodeRMC(char *RMCbuffer, RMCSTRUCT *rmc)
{
	char f[16];

	/* field 2: status A = valid, V = invalid */
	nmea_field(RMCbuffer, 2, f, sizeof(f));
	if (f[0] != 'A') {
		rmc->isValid = 0;
		return 1;
	}
	rmc->isValid = 1;

	/* field 7: speed over ground in knots -> m/s */
	nmea_field(RMCbuffer, 7, f, sizeof(f));
	rmc->speed = (f[0]) ? (float)(atof(f) * KNOTS_TO_MPS) : 0.0f;

	/* field 8: course over ground (deg, true). Blank when stationary. */
	nmea_field(RMCbuffer, 8, f, sizeof(f));
	rmc->course = (f[0]) ? (float)atof(f) : 0.0f;

	/* field 9: date ddmmyy */
	nmea_field(RMCbuffer, 9, f, sizeof(f));
	int d = atoi(f);
	rmc->date.Day = d / 10000;
	rmc->date.Mon = (d / 100) % 100;
	rmc->date.Yr  = d % 100;

	return 0;
}
