#include <string.h>
#include "stm32f4xx.h"

#define LAT_POSITION 2
#define LON_POSITION 4
#define SPD_POSITION 6
#define MAG_POSITION 9

typedef struct sGeoPointString {
	char lat[14];
	char lon[14];
	char lat_dir[2];
	char lon_dir[2];
} geoPointString;

typedef struct sGeoPointVector {
	char magVar[4];
	char speedKnt[8];
} geoPointVector;

typedef struct geoPointFloat {
	double lat;
	double lon;
} geoPointFloat;

void getGpsPointString (const uint8_t *nmeaString, geoPointString* geoPoint, geoPointVector* geoVector);
