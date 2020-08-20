#ifndef __OPTITRACK_H_INCLUDED__
#define __OPTITRACK_H_INCLUDED__

#include "Arduino.h"
#include <numeric>
#include "../SharedTools/SharedTools.h"
#include "../SharedTools/Logger/Logger.h"
#include "../SharedTools/XbeeWifi/XbeeWifi.h"

namespace Cybercortex
{

	class Optitrack
	{
	public:
		struct CONFIG
		{
			uint8_t msgIdx = 'O';
			uint8_t msgSize = 25;
		};

		enum class STATUS : uint8_t
		{
			INIT      = 0,
			RUNNING   = 1,
			FAILURE   = 2,
			TIMEDOUT  = 3
		};

	public:
		Optitrack(const uint32_t &timeout);
		Optitrack(const uint32_t &timeout,
					    const CONFIG &config);

		void init(XbeeWifi *xbeewifi);
		int32_t update(void);
		void rezero(void);

		STATUS status_;
		float pos_[3];
		float vel_[3];
	protected:
		const CONFIG config_;
		const uint32_t timeout_;

		bool hasWifi_;
		XbeeWifi *xbeeWifi_;
		float deltaPos_[3];
		float rawPos_[3];
		uint32_t tnm1Received_;
	};

}
#endif