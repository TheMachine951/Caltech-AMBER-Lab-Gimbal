#ifndef __FTS_H_INCLUDED__
#define __FTS_H_INCLUDED__

#include "Arduino.h"
#include "SPI.h"
#include "TimeLib.h"
#include "../../SO_Config.h"
#include "../SharedTools/SharedTools.h"
#include "../SharedTools/XbeeWifi/XbeeWifi.h"
#include "../SharedTools/Logger/Logger.h"
#include "../SharedTools/CANFD/CANFD.h"
#include "../IMUs/IMUs.h"
#include "../BatteryMonitor/BatteryMonitor.h"
#include "../Optitrack/Optitrack.h"
#include "../AdafruitGPS/AdafruitGPS.h"

namespace Cybercortex
{
	class SO
	{
	public:
		enum class REQUEST : uint8_t
		{
			UPDATE_DATE = 'D',
			TARE_IMUS = 'T',
			GPS_ENABLE = 'G',
			REZERO_POS = 'Z'
		};

	public:
		SO(void);
		void init(void);
		void update(void);

	protected:
		bool useGps_;
		bool useOptitrack_;
		const bool waitForGpsFix_;
		const uint16_t broadcastPort_;
		const uint8_t broadcastIdx_;
		const std::vector<String> logColumnNames_;
		const std::vector<uint8_t> logColumnTypes_;

		CANFD canfd_;
		YOST_TTS_LX IMUmain_;
		LPMS_CURS2 IMUbackup_;
		Adafruit_GPS gps_;
		SO_STATUS status_;
		BatteryMonitor batteryMonitor_;
		XbeeWifi xbeeWifi_;
		Optitrack optitrack_;

		uint32_t runT_;
		uint32_t iterT_;
		uint32_t iterCount_;
		uint32_t broadcastTnm1_;
		uint32_t displayTnm1_;
		bool newMainIMUData_;
		bool newBackupIMUData_;
		bool newGPSData_;
		bool newOptitrackData_;
		bool newInput_;
		uint8_t overrun_;
		float state_[STATE_LENGTH];
		float input_[INPUT_LENGTH];
		uint32_t hybridState_;

		void setStatus(SO_STATUS status);
		int32_t updateState(void);
		void publishState(void);
		int32_t parseTX2(const std::vector<uint8_t> &tx2Packet);
		int32_t observerFusionUpdate(const float stateCurrent[STATE_LENGTH],
																 const float gyro[3],
			                           const float acc[3],
			                           const float mag[3],
			                           float stateNew[STATE_LENGTH]);
		int32_t observerFusionUpdate(const float stateCurrent[STATE_LENGTH],
																 const float gyro[3],
			                           const float acc[3],
			                           const float mag[3],
			                           const float latitude,
			                           const float longitude,
			                           const float geoidheight,
			                           float stateNew[STATE_LENGTH]);
		bool checkStateConsistency(void);
		int32_t checkS0messages(void);
		bool isValidRequest(const uint8_t &req);
	};
}

#endif