#include "AltitudeFilter.h"

namespace Cybercortex {

	void fill_message(uint8_t *msg, num32_t const & msg_content) {
		// pack float into 6-byte message
		msg[0] = 0x48;
		msg[1] = msg_content.c[0];
	    msg[2] = msg_content.c[1];
	    msg[3] = msg_content.c[2];
	    msg[4] = msg_content.c[3];	
	    msg[5] = msg_content.c[0] + msg_content.c[1] + msg_content.c[2] + msg_content.c[3];
	}

	AltitudeFilter::AltitudeFilter() : frameT_(0) {
		msg = new uint8_t[6];
		altitude.f = 0;
	}

	int32_t AltitudeFilter::setup() {

		// Init debug and info ports
		DEBUG_PORT.begin(DEBUG_BAUD);
		INFO_PORT.begin(INFO_BAUD);

		// Set up radar
		if (!lox.begin()) {
		    INFO_LN("Failed to boot VL53L0X");
		    while(1);
		};

		// Set up pressure sensor
		if (!bme.begin()) {  
			INFO_LN("Failed to boot BMP280");
			while (1);
		}

		frameT_ = millis();

		INFO_LN("Setup completed");

		return 0;   
	}

	int32_t AltitudeFilter::loop() {

		if (millis() - frameT_ > MSG_RATE) {
			// Send data on Serial1

			fill_message(msg, altitude);
		    Serial1.write(msg, 6);
			frameT_ = millis();

		    // Serial.write(msg, 6);
		}

		// Read radar
		VL53L0X_RangingMeasurementData_t radar_data;
		lox.rangingTest(&radar_data, false);
		radar_altitude = radar_data.RangeMilliMeter / 1000.0;
	    DBG("RADAR ");
	    DBG_LN(radar_altitude);
	    DBG("RADAR STATUS ");
	    DBG_LN(radar_data.RangeStatus);

	    // Read sonar
	    sonar_altitude = analogRead(SONAR_ANALOG_PIN) * 5.0 / 1000.0;
	    DBG("SONAR ");
	    DBG_LN(sonar_altitude);

	    // Read pressure
	    pressure_altitude = log(nominal_pressure/bme.readPressure()) 
	    					* CONST_R * CONST_TSEA / (CONST_M * CONST_G);
	    DBG("PRESSURE ");
	    DBG_LN(pressure_altitude);

	    if (radar_data.RangeStatus != 4 && radar_altitude < SONAR_MIN) {
		    // Case 0: radar
	    	DBG_LN("Using radar");

	    	altitude.f = radar_altitude;
	    } else if (radar_data.RangeStatus != 4 && radar_altitude < RADAR_MAX) {
	    	// Case 1: mix radar/sonar
			DBG_LN("Using radar/sonar");

	    	altitude.f = radar_altitude * (radar_altitude - SONAR_MIN) / (RADAR_MAX - SONAR_MIN) + 
	    			  		  sonar_altitude * (RADAR_MAX - radar_altitude) / (RADAR_MAX - SONAR_MIN);
	    } else if (sonar_altitude < PRESS_MIN) {
	    	// Case 2: sonar
			DBG_LN("Using sonar");

	    	altitude.f = sonar_altitude;
	    } else if (sonar_altitude < SONAR_MAX) {
	    	// Case 3: mix sonar/pressure
			DBG_LN("Using sonar/pressure");

	    	altitude.f = sonar_altitude * (sonar_altitude - PRESS_MIN) / (SONAR_MAX - PRESS_MIN) + 
	    			  		  pressure_altitude * (SONAR_MAX - sonar_altitude) / (SONAR_MAX - PRESS_MIN);
	    } else {
	    	// Case 4: rely on pressure
			DBG_LN("Using pressure");

	    	altitude.f = pressure_altitude;
	    }

	    DBG("ALTITUDE ");
	    DBG_LN(altitude.f);

		if (!nominal_updated && radar_data.RangeStatus == 0) {
	    	// set nominal pressure from radar reading
	    	nominal_pressure = bme.readPressure() / 
	    					   exp(-CONST_M * CONST_G * altitude.f / (CONST_R * CONST_TSEA));

			DBG_LN("Calibrated pressure");
			DBG("Nominal pressure: ");
			DBG_LN(nominal_pressure);

			nominal_updated = true;
		}

		return 0;
	}
} // end of namespace