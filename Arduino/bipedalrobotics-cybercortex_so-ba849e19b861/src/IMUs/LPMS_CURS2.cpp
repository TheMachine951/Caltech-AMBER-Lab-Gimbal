#include "LPMS_CURS2.h"

namespace Cybercortex
{

	LPMS_CURS2::LPMS_CURS2(const IMUAbstract::MODE &mode,
		                     HardwareSerial &port,
		                     const uint32_t &baud,
		                     const uint32_t &readTimeout,
		                     const uint32_t &receiveTimeout,
		                     const uint32_t &ackTimeout,
		                     const uint32_t &resyncTimeout):
	IMUAbstract(mode,receiveTimeout),
	port_(port),
	baud_(baud),
	readTimeout_(readTimeout),
	ackTimeout_(ackTimeout),
	resyncTimeout_(resyncTimeout),
	config_(),
	messageLength_((mode==MODE::RAW)?config_.rawMessageLength:config_.fusionedMessageLength)
	{}

	LPMS_CURS2::LPMS_CURS2(const IMUAbstract::MODE &mode,
		                     HardwareSerial &port,
		                     const uint32_t &baud,
		                     const uint32_t &readTimeout,
		                     const uint32_t &receiveTimeout,
		                     const uint32_t &ackTimeout,
		                     const uint32_t &resyncTimeout,
		                     const CONFIG &config):
	IMUAbstract(mode,receiveTimeout),
	port_(port),
	baud_(baud),
	readTimeout_(readTimeout),
	ackTimeout_(ackTimeout),
	resyncTimeout_(resyncTimeout),
	config_(),
	messageLength_((mode==MODE::RAW)?config_.rawMessageLength:config_.fusionedMessageLength)
	{}

	LPMS_CURS2::~LPMS_CURS2()
	{}

	int32_t LPMS_CURS2::init()
	{
		logger.logln("LPMS_CURS2 - Initializing...");
		port_.begin(baud_);
		port_.setTimeout(readTimeout_);

		if(stopStream()!=1)
		{
			logger.logln("LPMS_CURS2 - init - stopStream failed");
			status_ = STATUS::FAILURE;
			return -1;
		}
		logger.logln("LPMS_CURS2 - init - Stream Stopped");

		if(setLpbusDataMode()!=1)
		{
			logger.logln("LPMS_CURS2 - init - setLpbusDataMode failed");
			status_ = STATUS::FAILURE;
			return -1;
		}
		logger.logln("LPMS_CURS2 - init - BUS data mode set");

		if(setStreamFreq()!=1)
		{
			logger.logln("LPMS_CURS2 - init - setStreamFreq failed");
			status_ = STATUS::FAILURE;
			return -1;
		}
		logger.logln("LPMS_CURS2 - init - Stream frequency set");

		if(setTransmitData()!=1)
		{
			logger.logln("LPMS_CURS2 - init - setTransmitData failed");
			status_ = STATUS::FAILURE;
			return -1;
		}
		logger.logln("LPMS_CURS2 - init - Transmit data set");

		resetTimeout();
		status_ = STATUS::RUNNING;
		logger.logln("LPMS_CURS2 - Initialized!");

		return 1;
	}

	int32_t LPMS_CURS2::update()
	{
		COMMAND cmd;
		std::vector<uint8_t> data;
		int32_t rtCode;
		uint32_t msgCounter = 0;
		uint32_t t0 = micros();

		while(newStreamCheck())
		{
			rtCode = readMessage(cmd,data);
			if(rtCode!=1)
			{
				if(rtCode==-2)
				{
					if(reSync()==1)
					{
						return 2;
					}
					else
					{
						logger.logln("LPMS_CURS2 - update - resync failed");
						status_ = STATUS::FAILURE;
						return -2;
					}
				}
				else
				{
					logger.logln("LPMS_CURS2 - update - readMessage failed");
					return -1;
				}
			}

			if(cmd!=COMMAND::GET_SENSOR_DATA)
			{
				logger.log("LPMS_CURS2 - update - unkown command: ");
				logger.loglnadd(static_cast<uint16_t>(cmd));
				return -3;
			}

			rtCode = parseStream(data);
			if(rtCode!=1)
			{
				logger.logln("LPMS_CURS2 - update - parse failed");
				status_ = STATUS::FAILURE;
				return -5;
			}

			resetTimeout();
			msgCounter++;
			dataCounter_++;
			status_ = STATUS::RUNNING;
		}

		if(msgCounter == 0 &&
			 (micros()-tnm1Received_)>receiveTimeout_)
		{
			logger.logln("LPMS_CURS2 - update - timed out");
			logger.logadd("msgCounter: ");
			logger.loglnadd(msgCounter);
			logger.logadd("dt: ");
			logger.loglnadd((micros()-tnm1Received_));
			logger.logadd("dataCounter_: ");
			logger.loglnadd(dataCounter_);
			logger.logadd("data available: ");
			logger.loglnadd(static_cast<int32_t>(port_.available()));
			status_ = STATUS::TIMEDOUT;
			return -2;
		}
		else if(msgCounter==0)
		{
			return 2;
		}
		else if(msgCounter==1)
		{
			return 1;
		}
		else if(msgCounter>1)
		{
			logger.log("LPMS_CURS2 - update - overrun of: ");
			logger.logadd(msgCounter);
			logger.loglnadd("msgs");

			return 1;
		}
	}

	int32_t LPMS_CURS2::startStream()
	{	
		if(gotoStreamMode()!=1)
		{
			logger.logln("LPMS_CURS2 couldn't start stream");
			status_ = STATUS::FAILURE;
			return -1;
		}
		return 1;
	}
	
	int32_t LPMS_CURS2::stopStream()
	{

		if(gotoCommandMode()!=1)
		{
			logger.logln("LPMS_CURS2 couldn't got to command mode");
			return -1;
		}

		delay(10);
		uint32_t t0 = millis();
		while(newMsgCheck())
		{
			if(millis()-t0>ackTimeout_)
			{
				logger.logln("LPMS_CURS2 couldn't stop stream timedout");
				return -2;
			}
			port_.read();
		}
		return 1;
	}

	bool LPMS_CURS2::newMsgCheck()
	{
		if(port_.available()>0)
			return true;
		else
			return false;
	}

	bool LPMS_CURS2::newStreamCheck()
	{
		if(port_.available()>=messageLength_)
			return true;
		else
			return false;
	}

	int32_t LPMS_CURS2::readMessage(LPMS_CURS2::COMMAND &cmd,
		                              std::vector<uint8_t> &data)
	{
		int32_t SOP = port_.read();
		num16_t n16;
		int32_t lenRead;
		uint16_t packetLen;
		uint16_t checkSum = 0;
		uint16_t cmdUi;

		if(SOP == -1)
		{
			logger.logln("LPMS_CURS2 Nothing to read");
			return -1;
		}

		if(SOP==static_cast<int32_t>(0x3A))
		{
			// Reading header
			uint8_t header[6];
			lenRead = port_.readBytes(header,6);
			if(lenRead!=6)
			{
				logger.logln("LPMS_CURS2 read timeout");
				return -3;
			}

			// Checking openMATID
			n16.c[0] = header[0];
			n16.c[1] = header[1];
			if(n16.ui!=config_.openMATID)
			{
				logger.logln("LPMS_CURS2 wrong openMATID received");
				return -4;
			}

			// Checking Command
			n16.c[0] = header[2];
			n16.c[1] = header[3];
			cmdUi = n16.ui;
			if(!isValidCommand(cmdUi))
			{
				logger.logln("LPMS_CURS2 unkown command received");
				return -5;
			}
			cmd = static_cast<COMMAND>(cmdUi);

			// Checking packet length
			n16.c[0] = header[4];
			n16.c[1] = header[5];
			packetLen = n16.ui;
			uint8_t packet[packetLen+4];
			lenRead = port_.readBytes(packet,packetLen+4);
			if(lenRead!=packetLen+4)
			{
				logger.logln("LPMS_CURS2 read timeout");
				return -3;
			}
			if(!(packet[packetLen+2]==0x0D && packet[packetLen+3]==0x0A))
			{
				// logger.logln("LPMS_CURS2 wrong packet size");
				// logger.logln("LPMS_CURS2 now out of sync");
				return -2;
			}

			// Checking checksum
			n16.c[0] = packet[packetLen];
			n16.c[1] = packet[packetLen+1];

			checkSum = config_.openMATID + cmdUi + packetLen;
			for(uint16_t i = 0; i<packetLen; i++)
			{
				checkSum+=packet[i];
			}

			if(checkSum!=n16.ui)
			{
				logger.logln("LPMS_CURS2 checksum failed");
				return -6;
			}

			//Reading packet
			if(packetLen>0)
			{
				std::vector<uint8_t> dataTemp(packet, packet + packetLen);
				data = dataTemp;
			}		
			return 1;
		}
		else
		{
			logger.logln("LPMS_CURS2 out of sync");
			return -2;
		}
	}

	int32_t LPMS_CURS2::sendMessage(const LPMS_CURS2::COMMAND &cmd,
						                      const uint16_t &dataLen,
												          const uint8_t data[])
	{
		uint32_t msgLen = dataLen+11;
		uint8_t msg[msgLen] = {0x00};
		uint16_t cmdUi = static_cast<uint16_t>(cmd);
		num16_t n16;

		//Start of packet
		msg[0] = 0x3A;


		n16.ui = config_.openMATID;
		msg[1] = n16.c[0];
		msg[2] = n16.c[1];

		n16.ui = cmdUi;
		msg[3] = n16.c[0];
		msg[4] = n16.c[1];

		n16.ui = dataLen;
		msg[5] = n16.c[0];
		msg[6] = n16.c[1];

		memcpy(msg+7,data,dataLen);

		uint16_t checkSum = config_.openMATID + cmdUi + dataLen;

		for(uint32_t i = 0; i<dataLen; i++)
		{
			checkSum+=data[i];
		}
		n16.ui = checkSum;

		msg[msgLen-4] = n16.c[0];
		msg[msgLen-3] = n16.c[1];
		msg[msgLen-2] = 0x0D;
		msg[msgLen-1] = 0x0A;

		port_.write(msg,msgLen);
		return 1;
	}

	int32_t LPMS_CURS2::gotoCommandMode()
	{

		COMMAND cmd = COMMAND::GOTO_COMMAND_MODE;
		uint16_t dataLen = 0;
		if(sendMessage(cmd,dataLen,nullptr)!=1)
		{
			logger.logln("LPMS_CURS2 couldn't got to command mode");
			return -1;
		}
		return 1;
	}

	int32_t LPMS_CURS2::gotoStreamMode()
	{
		COMMAND cmd = COMMAND::GOTO_STREAM_MODE;
		uint16_t dataLen = 1;
		if(sendMessage(cmd,dataLen,nullptr)!=1)
		{
			logger.logln("LPMS_CURS2 couldn't got to stream mode");
			return -1;
		}
		if(checkACK()!=1)
		{
			logger.logln("LPMS_CURS2 couldn't got to stream mode");
			return -1;
		}
		return 1;
	}

	int32_t LPMS_CURS2::setStreamFreq()
	{

		COMMAND cmd = COMMAND::SET_STREAM_FREQ;
		uint16_t dataLen = 4;
		num32_t n32;
		n32.i = config_.streamFreq;

		if(sendMessage(cmd,dataLen,n32.c)!=1)
		{
			logger.logln("LPMS_CURS2 couldn't set stream frequency");
			return -1;
		}

		if(checkACK()!=1)
		{
			logger.logln("LPMS_CURS2 couldn't set stream frequency");
			return -1;
		}
		return 1;
	}

	int32_t LPMS_CURS2::setTransmitData()
	{

		COMMAND cmd = COMMAND::SET_TRANSMIT_DATA;
		uint16_t dataLen = 4;
		num32_t n32;
		if(mode_==MODE::RAW)
			n32.i = config_.rawStreamData;
		else if(mode_==MODE::FUSIONED)
			n32.i = config_.fusionedStreamData;
		else
		{
			logger.logln("LPMS_CURS2 - setTransmitData - unkown mode");
			return -1;
		}

		if(sendMessage(cmd,dataLen,n32.c)!=1)
		{
			logger.logln("LPMS_CURS2 - setTransmitData - couldn't set transmit data");
			return -1;
		}

		if(checkACK()!=1)
		{
			logger.logln("LPMS_CURS2 - setTransmitData - couldn't set transmit data");
			return -1;
		}
		return 1;
	}

	int32_t LPMS_CURS2::setUartFormat()
	{
		COMMAND cmd = COMMAND::SET_UART_FORMAT;
		uint16_t dataLen = 4;
		num32_t n32;
		n32.i = config_.uartFormat;
		if(sendMessage(cmd,dataLen,n32.c)!=1)
		{
			logger.logln("LPMS_CURS2 - setUartFormat - couldn't set transmit data");
			return -1;
		}

		if(checkACK()!=1)
		{
			logger.logln("LPMS_CURS2 - setUartFormat - couldn't set transmit data");
			return -1;
		}
		return 1;
	}

	int32_t LPMS_CURS2::setLpbusDataMode()
	{
		COMMAND cmd = COMMAND::SET_LPBUS_DATA_MODE;
		uint16_t dataLen = 4;
		num32_t n32;
		n32.i = config_.busDataMode;
		if(sendMessage(cmd,dataLen,n32.c)!=1)
		{
			logger.logln("LPMS_CURS2 - setLpbusDataMode - couldn't set transmit data");
			return -1;
		}

		if(checkACK()!=1)
		{
			logger.logln("LPMS_CURS2 - setLpbusDataMode - couldn't set transmit data");
			return -1;
		}
		return 1;
	}
	int32_t LPMS_CURS2::reSync()
	{
		uint32_t t0 = millis();
		uint8_t buff[2] = {0x00};

		while(port_.available()>0)
		{
			if(millis()-t0>resyncTimeout_)
			{
				logger.logln("LPMS_CURS2 - reSync - timedout");
				return -1;	
			}
			port_.read();
		}

		t0 = millis();
		while(true)
		{
			if(millis()-t0>resyncTimeout_)
			{
				logger.logln("LPMS_CURS2 - reSync - timedout");
				return -2;
			}
			if(newMsgCheck())
			{
				buff[0] = buff[1];
				buff[1] = static_cast<uint8_t>(port_.read());
				if(buff[0]==0x0D && buff[1]==0x0A)
				{
					return 1;
				}
			}
		}
	}

	int32_t LPMS_CURS2::checkACK()
	{
		uint32_t t0 = millis();
		std::vector<uint8_t> data;
		int32_t rtCode;
		COMMAND cmd;

		while(millis()-t0<ackTimeout_)
		{
			if(newMsgCheck())
			{
				rtCode = readMessage(cmd,data);
				if(rtCode!=1)
				{
					logger.logln("LPMS_CURS2 checkACK readMessage failed");
					return rtCode;
				}
				else if(cmd==COMMAND::REPLY_ACK)
					return 1;
				else if(cmd==COMMAND::REPLY_NACK)
					return 0;
				else
				{
					logger.logln("LPMS_CURS2 checkACK unknow reply");
					return -1;
				}
			}
		}
		logger.logln("LPMS_CURS2 checkACK timedout");
		return -2;
	}

	int32_t LPMS_CURS2::parseStream(const std::vector<uint8_t> &data)
	{
		num32_t n32;
		if(mode_==MODE::FUSIONED)
		{
			//Check length
			if(data.size()!=32)
			{
				logger.log("LPMS_CURS2 - parseStream - wrong length");
				return -1; 
			}

			//Parse rates
			for(uint32_t i = 0; i<3; i++)
			{
				for(uint32_t j = 0; j<4; j++)
				{
					n32.c[j] = data[j+i*4+4];
				}
				fusionedData_.rates[i] = n32.f;
			}

			//Parse quat
			for(uint32_t i = 0; i<4; i++)
			{
				for(uint32_t j = 0; j<4; j++)
				{
					n32.c[j] = data[j+i*4+16];
				}
				fusionedData_.quat[i] = n32.f;
			}
		}
		else if(mode_==MODE::RAW)
		{
			//Check length
			if(data.size()!=40)
			{
				logger.log("LPMS_CURS2 - parseStream - wrong length");
				return -1; 
			}

			//Parse gyro
			for(uint32_t i = 0; i<3; i++)
			{
				for(uint32_t j = 0; j<4; j++)
				{
					n32.c[j] = data[j+i*4+4];
				}
				rawData_.gyro[i] = n32.f;
			}

			//Parse acc
			for(uint32_t i = 0; i<3; i++)
			{
				for(uint32_t j = 0; j<4; j++)
				{
					n32.c[j] = data[j+i*4+16];
				}
				rawData_.acc[i] = n32.f;
			}

			//Parse mag
			for(uint32_t i = 0; i<3; i++)
			{
				for(uint32_t j = 0; j<4; j++)
				{
					n32.c[j] = data[j+i*4+28];
				}
				rawData_.mag[i] = n32.f;
			}
		}
		else
		{
			logger.logln("LPMS_CURS2 - parseStream - unkown mode");
			return -2;
		}

		return 1;
	}

	bool LPMS_CURS2::isValidCommand(const uint16_t &cmd)
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
}