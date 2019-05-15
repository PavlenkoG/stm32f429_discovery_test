
#include "geo.h"
#include <errno.h>


void getGpsPointString (const uint8_t* nmeaString, geoPointString* geoPoint, geoPointVector* geoVector){

	uint8_t* pos;
	uint8_t* posNext;
	uint32_t lat_len = 0;
	uint32_t lon_len = 0;
	uint32_t lat_pos = 0;
	uint32_t lon_pos = 0;
	uint32_t speed_pos = 0;
	uint32_t speed_len = 0;
	uint32_t magVar_pos = 0;
	uint32_t magVar_len = 0;

	if (strstr(nmeaString, "RMC") != 0){
		if (strstr(nmeaString, ",A,")!=0) {
			int i = 0;
			int currentPos = 0;
			while (nmeaString[i]) {
				if (nmeaString[i] == ','){
					if (currentPos == LAT_POSITION) {lat_pos = i + 1;}
					if (currentPos == (LAT_POSITION + 1)){lat_len = i - lat_pos;}
					if (currentPos == LON_POSITION){ lon_pos = i + 1;}
					if (currentPos == (LON_POSITION + 1)){lon_len = i - lon_pos;}
					if (currentPos == SPD_POSITION) {speed_pos = i + 1;}
					if (currentPos == (SPD_POSITION + 1)) {speed_len = i - speed_pos;}
					if (currentPos == MAG_POSITION) {magVar_pos = i + 1;}
					if (currentPos == (MAG_POSITION + 1)) {magVar_len = i - magVar_pos;}
					currentPos ++;
				}
				i++;
			}
			strncpy(geoPoint->lat,&nmeaString[lat_pos], lat_len);
			geoPoint->lat[lat_len] = 0;
			geoPoint->lat_dir[0] = nmeaString[(lat_pos + lat_len + 1)];
			strncpy(geoPoint->lon,&nmeaString[lon_pos], lon_len);
			geoPoint->lon_dir[0] = nmeaString[(lon_pos + lon_len + 1)];
			geoPoint->lon[lon_len] = 0;
			strncpy(geoVector->speedKnt,&nmeaString[speed_pos], speed_len);
			geoVector->speedKnt[speed_len] = 0;
			strncpy(geoVector->magVar,&nmeaString[magVar_pos], magVar_len);
			geoVector->magVar[magVar_len] = 0;

		} else {
		}
	}
}
