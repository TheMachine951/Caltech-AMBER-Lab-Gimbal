#ifndef __ALTF_INCLUDED__
#define __ALTF_INCLUDED__

#include <cmath>
#include "Arduino.h"

#include "Adafruit_VL53L0X.h"  // Radar
#include "Adafruit_BMP280.h"   // Pressure sensor

#include "../SharedTools/SharedTools.h"

#include "ALT_Config.h"

namespace Cybercortex {

	class AltitudeFilter {
	public:
		AltitudeFilter();
		int32_t setup();
		int32_t loop();
	
	private:
		Adafruit_VL53L0X lox;
		Adafruit_BMP280 bme;

		float nominal_pressure = 99400;
		bool nominal_updated = false;

		uint8_t *msg;
		num32_t altitude;

		float radar_altitude, sonar_altitude, pressure_altitude;

		uint32_t frameT_;
	};

} // end of namespace

#endif