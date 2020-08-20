#include "YOST_TTS_LX.h"

namespace Cybercortex
{

	YOST_TTS_LX::YOST_TTS_LX(const IMUAbstract::MODE &mode,
		                       SPIClass &spi,
		                       const int32_t &pinSS,
			                     const int32_t &pinATT,
		                       const uint32_t &receiveTimeout):
	IMUAbstract(mode,receiveTimeout),
	spi_(spi),
	pinSS_(pinSS),
	pinATT_(pinATT),
	config_(),
	pinValNm1_(1),
	SPIsetting_(config_.spiBaud,config_.spiMSB,config_.spiMode)
	{}

	YOST_TTS_LX::YOST_TTS_LX(const IMUAbstract::MODE &mode,
		                       SPIClass &spi,
           			           const int32_t &pinSS,
			          					 const int32_t &pinATT,
			                     const uint32_t &receiveTimeout,
			                     const CONFIG &config):
	IMUAbstract(mode,receiveTimeout),
	spi_(spi),
	pinSS_(pinSS),
	pinATT_(pinATT),
	config_(config),
	SPIsetting_(config_.spiBaud,config_.spiMSB,config_.spiMode)
	{}

	YOST_TTS_LX::~YOST_TTS_LX()
	{}

	int32_t YOST_TTS_LX::init()
	{
		Serial.println("YOST_TTS_LX - Initializing...");

		pinMode(pinATT_,INPUT_PULLUP);
		pinMode(pinSS_,OUTPUT);
		digitalWriteFast(pinSS_,HIGH);

		Serial.println("YOST_TTS_LX - init - reset to default setting");
		resetSetting();
		delay(50);

		Serial.println("YOST_TTS_LX - init - interrupt type set");
		setInterruptType();
		delay(10);

		// setStreamTiming();
		// Serial.println("YOST_TTS_LX - init - stream timing set");
		// delay(10);

		Serial.println("YOST_TTS_LX - init - axis convention");
		setAxisConvention();
		delay(10);

		Serial.println("YOST_TTS_LX - init - commit settings");
		commitSetting();
		delay(10);

		Serial.println("YOST_TTS_LX - init - tare");
		tare();
		delay(100);
		
		resetTimeout();
		status_ = STATUS::RUNNING;

		Serial.println("YOST_TTS_LX - Initialized!");
		return 1;
	}

	int32_t YOST_TTS_LX::update()
	{
		if(newMsgCheck())
		{
			if(mode_==MODE::RAW)
			{
				if(getRawData()!=1)
				{
					Serial.println("YOST_TTS_LX - update - getRawData failed");	
					return -1;	
				}
			}
			else if(mode_==MODE::FUSIONED)
			{
				if(getFusionedData()!=1)
				{
					Serial.println("YOST_TTS_LX - update - getFusionedData failed");	
					return -1;
				}
			}
			else
			{
				Serial.println("YOST_TTS_LX - update - unknown mode");
				status_ = STATUS::FAILURE;
				return -1;
			}
			tnm1Received_ = micros();
			dataCounter_++;
			status_ = STATUS::RUNNING;
			return 1;
		}
		else
		{
			if((micros()-tnm1Received_)>receiveTimeout_)
			{
				Serial.println("YOST_TTS_LX - update - timeout");
				status_ = STATUS::TIMEDOUT;
				return -2;
			}
			return 2;
		}
	}

	bool YOST_TTS_LX::newMsgCheck(void)
	{
		int32_t pinVal = digitalReadFast(pinATT_);
		if(pinVal==config_.interruptTypePolarity && pinVal!=pinValNm1_)
		{
			pinValNm1_ = pinVal;
			return true;
		}
		else
		{
			pinValNm1_ = pinVal;
			return false;
		}
	}

	int32_t YOST_TTS_LX::getRawData(void)
	{
		const COMMAND cmd = COMMAND::GET_ALL_CORRECTED;
		const uint32_t len = 36;

		std::vector<uint8_t> data;
		sendMessage(cmd);
		num32_t n32;
		RAW_DATA rawDataTemp;

		readMessage(len,data);

		//Parse gyro
		for(uint32_t i = 0; i<3; i++)
		{
			for(uint32_t j = 0; j<4; j++)
			{
				n32.c[j] = data[(3-j)+i*4];
			}
			rawDataTemp.gyro[i] = n32.f;
		}
		//Parse acc
		for(uint32_t i = 0; i<3; i++)
		{
			for(uint32_t j = 0; j<4; j++)
			{
				n32.c[j] = data[(3-j)+i*4+12];
			}
			rawDataTemp.acc[i] = n32.f;
		}

		//Parse mag
		for(uint32_t i = 0; i<3; i++)
		{
			for(uint32_t j = 0; j<4; j++)
			{
				n32.c[j] = data[(3-j)+i*4+24];
			}
			rawDataTemp.mag[i] = n32.f;
		}

		// //reorder axis
		// float tempData;
		// tempData = rawDataTemp.gyro[2];
		// rawDataTemp.gyro[2] = rawDataTemp.gyro[1];
		// rawDataTemp.gyro[1] = tempData;
		// tempData = rawDataTemp.acc[2];
		// rawDataTemp.acc[2] = rawDataTemp.acc[1];
		// rawDataTemp.acc[1] = tempData;
		// tempData = rawDataTemp.mag[2];
		// rawDataTemp.mag[2] = rawDataTemp.mag[1];
		// rawDataTemp.mag[1] = tempData;

		if(checkConsistancy(rawDataTemp))
		{
			rawData_ = rawDataTemp;
			return 1;
		}
		else
		{
			Serial.println("YOST_TTS_LX - getRawData - checkConsistancy failed");
			return -1;
		}
	}

	int32_t YOST_TTS_LX::getFusionedData(void)
	{
		COMMAND cmd;
		uint32_t len;
		std::vector<uint8_t> data;
		num32_t n32;
		FUSIONED_DATA fusionedDataTemp;

		//Get quaternion
		cmd = COMMAND::GET_QUAT_TARED;
		len = 16;
		sendMessage(cmd);
		readMessage(len,data);

		//Parse quat
		for(uint32_t i = 0; i<4; i++)
		{
			for(uint32_t j = 0; j<4; j++)
			{
				n32.c[j] = data[(3-j)+i*4];
			}
			fusionedDataTemp.quat[i] = n32.f;
		}

		//Reorder quat
		float tempData;
		tempData = fusionedDataTemp.quat[3];
		fusionedDataTemp.quat[3] = fusionedDataTemp.quat[2];
		fusionedDataTemp.quat[2] = fusionedDataTemp.quat[1];
		fusionedDataTemp.quat[1] = fusionedDataTemp.quat[0];
		fusionedDataTemp.quat[0] = tempData;

		data.clear();

		//Get rates
		cmd = COMMAND::GET_GYRO_CORRECTED;
		len = 12;
		sendMessage(cmd);
		readMessage(len,data);

		//Parse rates
		for(uint32_t i = 0; i<3; i++)
		{
			for(uint32_t j = 0; j<4; j++)
			{
				n32.c[j] = data[(3-j)+i*4];
			}
			fusionedDataTemp.rates[i] = n32.f;
		}


		if(checkConsistancy(fusionedDataTemp))
		{
			fusionedData_ = fusionedDataTemp;
			return 1;
		}
		else
		{
			Serial.println("YOST_TTS_LX - getFusionedData - checkConsistancy failed");
			return -1;
		}
	}

	void YOST_TTS_LX::readMessage(const uint32_t &len,
		                              std::vector<uint8_t> &data)
	{
		uint8_t msg[len+1] = {0xFF};

		spi_.beginTransaction(SPIsetting_);
		digitalWriteFast(pinSS_, LOW);
		spi_.transfer(config_.SPIread);
		spi_.transfer(msg,len+1);
		digitalWriteFast(pinSS_, HIGH);
		spi_.endTransaction();

		data.insert(data.end(),msg+1,msg+len+1);
	}

	int32_t YOST_TTS_LX::sendMessage(const COMMAND &cmd,
		                               const std::vector<uint8_t> &data)
	{

		uint32_t dataLen = data.size();
		uint32_t msgLen = dataLen + 2;

		uint8_t msg[msgLen] = {config_.SPIwrite};
		msg[1] = static_cast<uint8_t>(cmd);

		memcpy(msg+2,data.data(),dataLen);

		spi_.beginTransaction(SPIsetting_);
		digitalWriteFast(pinSS_, LOW);
		spi_.transfer(msg, msgLen);
		digitalWriteFast(pinSS_, HIGH);
		spi_.endTransaction();

		return 1;
	}

	int32_t YOST_TTS_LX::sendMessage(const COMMAND &cmd)
	{
		uint32_t msgLen = 2;
		uint8_t msg[msgLen] = {config_.SPIwrite,static_cast<uint8_t>(cmd)};

		spi_.beginTransaction(SPIsetting_);
		digitalWriteFast(pinSS_, LOW);
		spi_.transfer(msg, msgLen);
		digitalWriteFast(pinSS_, HIGH);
		spi_.endTransaction();
		delayMicroseconds(20);
		return 1;
	}

	void YOST_TTS_LX::setInterruptType(void)
	{
		const COMMAND cmd = COMMAND::SET_INTERRUPT_TYPE;
		const std::vector<uint8_t> data{config_.interruptTypeMode,
			                              config_.interruptTypePin,
			                              config_.interruptTypePolarity};
		sendMessage(cmd,data);                 
	}

	void YOST_TTS_LX::setStreamTiming(void)
	{
		const COMMAND cmd = COMMAND::SET_STREAM_TIMING;
		const std::vector<uint8_t> data{config_.timingInterval,
			                              config_.timingDuration,
			                              config_.timingDelay};
		sendMessage(cmd,data);                 
	}

	void YOST_TTS_LX::setAxisConvention(void)
	{
		const COMMAND cmd = COMMAND::SET_AXIS_DIRECTION;
		const std::vector<uint8_t> data{1};
		sendMessage(cmd,data);                 
	}

	void YOST_TTS_LX::tare(void)
	{
		const COMMAND cmd = COMMAND::TARE;
		sendMessage(cmd);              
	}

	void YOST_TTS_LX::resetSensor(void)
	{
		const COMMAND cmd = COMMAND::RESET_SENSOR;
		sendMessage(cmd);              
	}

	void YOST_TTS_LX::resetSetting(void)
	{
		const COMMAND cmd = COMMAND::RESET_SETTINGS;
		sendMessage(cmd);              
	}

	void YOST_TTS_LX::commitSetting(void)
	{
		const COMMAND cmd = COMMAND::COMMIT_SETTINGS;
		sendMessage(cmd);              
	}

	bool YOST_TTS_LX::isValidCommand(const uint8_t &cmd)
	{
		if(cmd==0 ||
			 cmd==1 ||
			 cmd==2 ||
			 cmd==3 ||
			 cmd==4 ||
			 cmd==5 ||
			 cmd==6 ||
			 cmd==7 ||
			 cmd==9 ||
			 cmd==10 ||
			 cmd==11 ||
			 cmd==20 ||
			 cmd==21 ||
			 cmd==25 ||
			 cmd==26 ||
			 cmd==31 ||
			 cmd==32 ||
			 cmd==33 ||
			 cmd==34 ||
			 cmd==41 ||
			 cmd==42 ||
			 cmd==43 ||
			 cmd==44 ||
			 cmd==60 ||
			 cmd==61 ||
			 cmd==66 ||
			 cmd==67 ||
			 cmd==68 ||
			 cmd==75 ||
			 cmd==84 ||
			 cmd==85 ||
			 cmd==86)
			return true;
		else
			return false;
	}

	bool YOST_TTS_LX::checkConsistancy(const RAW_DATA data)
	{
		return true;
	}

	bool YOST_TTS_LX::checkConsistancy(const FUSIONED_DATA data)
	{
		return true;
	}
}