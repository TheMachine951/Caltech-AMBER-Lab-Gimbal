#ifndef __IMU_ABSTRACT_H_INCLUDED__
#define __IMU_ABSTRACT_H_INCLUDED__

#include "Arduino.h"
#include "../SharedTools/SharedTools.h"
#include "../SharedTools/Logger/Logger.h"

namespace Cybercortex
{
	class IMUAbstract
	{
	public:
		enum class MODE : uint32_t
		{
			RAW      = 1,
			FUSIONED = 2
		};

		enum class STATUS : uint32_t
		{
			INIT     = 1,
			RUNNING  = 2,
			TIMEDOUT = 3,
			FAILURE  = 4
		};

		struct RAW_DATA
		{
			float gyro[3];
			float acc[3];
			float mag[3];
		};

		struct FUSIONED_DATA
		{
			float quat[4];
			float rates[3];
		};

		IMUAbstract(const MODE &mode,
			          const uint32_t &receiveTimeout);	
		virtual ~IMUAbstract();

		virtual int32_t init() = 0;
		virtual int32_t update() = 0;

		void resetTimeout(void);
		
		RAW_DATA rawData_;
		FUSIONED_DATA fusionedData_;
		STATUS status_;
		
	protected:
		const uint32_t receiveTimeout_;
		uint32_t tnm1Received_;
		uint32_t dataCounter_;
		MODE mode_;

	};
}

#endif