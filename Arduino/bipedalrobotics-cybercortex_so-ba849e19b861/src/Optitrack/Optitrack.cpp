#include "Optitrack.h"

namespace Cybercortex
{

	Optitrack::Optitrack(const uint32_t &timeout):
	status_(Optitrack::STATUS::INIT),
	pos_{0.0F},
	vel_{0.0F},
	config_(),
	timeout_(timeout),
	hasWifi_(false),
	xbeeWifi_(nullptr),
	deltaPos_{0.0F},
	rawPos_{0.0F},
	tnm1Received_(0)
	{}

	Optitrack::Optitrack(const uint32_t &timeout,
					             const CONFIG &config):
	status_(Optitrack::STATUS::INIT),
	pos_{0.0F},
	vel_{0.0F},
	config_(config),
	timeout_(timeout),
	hasWifi_(false),
	xbeeWifi_(nullptr),
	deltaPos_{0.0F},
	rawPos_{0.0F},
	tnm1Received_(0)
	{}



	void Optitrack::init(XbeeWifi *xbeewifi)
	{
		logger.logln("Optitrack - Initializing...");
		xbeeWifi_ = xbeewifi;
		hasWifi_ = true;
    logger.logln("Optitrack - Initialized!");
	}

	int32_t Optitrack::update(void)
	{
		if(!hasWifi_)
		{
			return 2;
		}

		std::vector<uint8_t> packet;
		num32_t sourceAddress;

		int32_t rtCode = xbeeWifi_->getMyPacket(config_.msgIdx,sourceAddress.ui,packet);
		if(rtCode==1)
		{
			if(packet.size()!=config_.msgSize)
			{
				logger.log("Optitrack - update - wrong message  size: ");
				logger.loglnadd(static_cast<uint32_t>(packet.size()));
				return -1;
			}

			if(!checkSumOK(packet))
			{
				logger.log("Optitrack - update - checksum failed");
				return -2;
			}

			tnm1Received_ = millis();
			status_ = STATUS::RUNNING;
			
			//Parse positions
			num32_t n32;
			for(uint32_t i = 0; i<3; i++)
			{
				for(uint32_t j = 0; j<4; j++)
				{
					n32.c[j] = packet[4*i+j];
				}
				rawPos_[i] = n32.f;
				pos_[i] = rawPos_[i] - deltaPos_[i];
			}

			//Parse velocities
			for(uint32_t i = 0; i<3; i++)
			{
				for(uint32_t j = 0; j<4; j++)
				{
					n32.c[j] = packet[4*i+j+12];
				}
				vel_[i] = n32.f;
			}

			return 1;
		}
		else
		{
			if((millis()-tnm1Received_)>timeout_ && status_ == STATUS::RUNNING)
			{
				logger.logln("Optitrack - update - timeout");
				status_ = STATUS::TIMEDOUT;
				return -3;
			}
			return 2;
		}

	}

	void Optitrack::rezero(void)
	{
		memcpy(deltaPos_,rawPos_,12);
	}

	
}