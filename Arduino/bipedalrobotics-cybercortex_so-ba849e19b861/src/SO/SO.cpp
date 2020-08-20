#include "SO.h"

namespace Cybercortex
{
	//Constructor
	SO::SO(void) : 
	useGps_(false),
	useOptitrack_(true),
	waitForGpsFix_(ADAFRUIT_GPS_WAIT_FOR_FIX),
	broadcastPort_(SO_BROADCAST_PORT),
	broadcastIdx_(SO_BROADCAST_IDX),
	logColumnNames_{String("runT"),
		              String("iterT"),
		              String("x"),
		              String("y"),
		              String("z"),
		              String("vx"),
		              String("vy"),
		              String("vz"),
		              String("qw"),
		              String("qx"),
		              String("qy"),
		              String("qz"),
		              String("p"),
		              String("q"),
		              String("r"),
		              String("w1"),
		              String("w2"),
		              String("w3"),
		              String("w4"),
		              String("u1"),
		              String("u2"),
		              String("u3"),
		              String("u4"),
		              String("batV"),
		              String("batI"),
									String("statusSO"),
									String("overrun"),
									String("hybridState"),
									String("GPSfix"),
									String("GPSlattitude"),
									String("GPSlongitude")},
	logColumnTypes_{'u',
	                'u',
	                'f','f','f','f','f','f','f','f','f','f','f','f','f','f','f','f','f',
	                'f','f','f','f',
	                'f',
	                'f',
	                'u',
	                'u',
	                'u',
	                'u',
	                'f',
	                'f'},
	canfd_(CANFD_PIN_SS,
		     CANFD_PIN_INT,
		     CANFD_PIN_INT0,
		     CANFD_PIN_INT1,
		     CANFD_READ_TIMEOUT),
	IMUmain_(YOST_TTS_LX_MODE,
		       SPI1,
		       YOST_TTS_LX_PIN_SS,
		       YOST_TTS_LX_PIN_ATT,
		       YOST_TTS_LX_RECEIVE_TIMEOUT),
	IMUbackup_(IMUAbstract::MODE::FUSIONED,
						 LPMS_CURS2_PORT,
						 LPMS_CURS2_BAUD,
						 LPMS_CURS2_READ_TIMEOUT,
						 LPMS_CURS2_RECEIVE_TIMEOUT,
						 LPMS_CURS2_ACK_TIMEOUT,
						 LPMS_CURS2_RESYNC_TIMEOUT),
	gps_(&ADAFRUIT_GPS_PORT,
		   ADAFRUIT_GPS_PIN_EN,
		   ADAFRUIT_GPS_PIN_PPS),
	status_(SO_STATUS::INIT),
	batteryMonitor_(BATTERY_CURRENT_PIN,
		              BATTERY_VOLTAGE_PIN,
		              BATTERY_CURRENT_TAU,
		              BATTERY_VOLTAGE_TAU),
	xbeeWifi_(SPI1,
		        XBEE_WIFI_PIN_SS,
						XBEE_WIFI_PIN_ATT,
						XBEE_WIFI_PIN_DOUT,
						XBEE_WIFI_PIN_RESET),
	optitrack_(OPTITRACK_TIMEOUT),
	runT_(0),
	iterT_(0),
	iterCount_(0),
	broadcastTnm1_(0),
	displayTnm1_(0),
	newMainIMUData_(false),
	newBackupIMUData_(false),
	newGPSData_(false),
	newOptitrackData_(false),
	newInput_(false),
	overrun_(0),
	state_(),
	input_(),
	hybridState_(0)
	{}

	void SO::init(void)
	{
		int32_t rtCode;

		// Init Serial port
		Serial.begin(9600);

	  // Init sensor SPI
	  SPI1.setMOSI(SO_PIN_MOSI);
	  SPI1.setMISO(SO_PIN_MISO);
	  SPI1.setSCK(SO_PIN_SCK);
    SPI1.begin();
    delay(10);
    
    // Init log
    if(logger.init("State Observer",SO_BROADCAST_IDX,logColumnNames_,logColumnTypes_)!=1)
		{
			logger.logln("State Observer - Logger couldn't initialized. Check SD card");
			setStatus(SO_STATUS::FAILURE);
			return;
		}

		logger.logln("Initializing State Observer...");

		//Initialize xbee wifi	
		rtCode = 0;
		uint32_t xbeeWifiInitCount = 0;
		while(rtCode!=1)
		{
			rtCode = xbeeWifi_.init();
			if(xbeeWifiInitCount>XBEE_WIFI_MAX_INIT_TRIALS)
			{
				setStatus(SO_STATUS::FAILURE);
				return;
			}
			else if(rtCode!=1)
			{
				logger.logln("State Observer - init - xbeeWifi failed to initialize, trying again");
			}
			xbeeWifiInitCount++;
		}

		logger.setWifi(&xbeeWifi_);
		logger.logln("State Observer - init - xbeeWifi set for logger");

		//Initialize gps
		if(EEPROM.read(USE_GPS_ADDRESS)==1)
		{
			useGps_ = true;
		}
		else
		{
			EEPROM.write(USE_GPS_ADDRESS,0);
			useGps_ = false;	
		}

		if(useGps_)
		{
			useOptitrack_ = false;
			rtCode = gps_.init();
			if(rtCode<0)
			{
				setStatus(SO_STATUS::FAILURE);
				return;
			}
			
			if(waitForGpsFix_)
			{
				logger.logln("State Observer - init - waiting for GPS fix...");
				rtCode = gps_.waitForFix(ADAFRUIT_GPS_WAIT_FOR_FIX_TIMEOUT);
				if(rtCode<0)
				{
					setStatus(SO_STATUS::FAILURE);
					return;
				}
				logger.logln("State Observer - init - GPS fix established!");
				setTime(gps_.hour,gps_.minute,gps_.seconds,gps_.day,gps_.month,gps_.year);
				Teensy3Clock.set(now());
				logger.log("State Observer - init - RTC synchronized to ");
				logger.logadd(gps_.hour, DEC); logger.logadd(':');
				logger.logadd(gps_.minute, DEC); logger.logadd(':');
				logger.logadd(gps_.seconds, DEC);
				logger.logadd(" - ");
				logger.logadd(gps_.day, DEC); logger.logadd('/');
				logger.logadd(gps_.month, DEC); logger.logadd("/20");
				logger.loglnadd(gps_.year, DEC);
			}
		}
		else
		{
			gps_.initShort();
			gps_.disable();
		}

		//Rename log to match possibly updated time
		runT_ = static_cast<uint32_t>(Teensy3Clock.get());
		logger.setLogNumber(runT_);
		logger.log("SO - Logger date: ");
		logger.loglnadd(runT_);

		//Initialize optitrack
		if(useOptitrack_)
		{
			optitrack_.init(&xbeeWifi_);
		}

    //Initialize battery monitor
		batteryMonitor_.init();

    //Initialize canfd
		rtCode = canfd_.init();
		if(rtCode!=1)
		{
			setStatus(SO_STATUS::FAILURE);
			return;
		}

		 //Initialize main IMU
		rtCode = IMUmain_.init();
		if(rtCode!=1)
		{
			setStatus(SO_STATUS::FAILURE);
			return;
		}

    //Initialize backup IMU
		rtCode = IMUbackup_.init();
		if(rtCode!=1)
		{
			setStatus(SO_STATUS::FAILURE);
			return;
		}

		logger.logln("State Observer initialized!");
		setStatus(SO_STATUS::NOMINAL);
		delay(20);

		broadcastTnm1_ = micros();
		displayTnm1_ = micros();
		iterT_ = micros();

		IMUmain_.resetTimeout();
		IMUbackup_.resetTimeout();
		return;	
	}


	//Update internal state
	void SO::update(void)
	{
		int32_t rtCode;
		switch(status_)
		{
			case SO_STATUS::NOMINAL:
			{
				IMUmain_.resetTimeout();
				//Updating the sensors
				while(true)
				{
					//Updating canfd
					rtCode = canfd_.update();
					if(rtCode==1)
					{
						std::vector<uint8_t> tx2Packet;
						rtCode = canfd_.getMyPacket(CTRL_BROADCAST_IDX,tx2Packet);
						if(rtCode==1)
						{
							rtCode = parseTX2(tx2Packet);
							if(rtCode==1)
							{
								newInput_ = true;
							}
						}
					}

					//Updating gps
					if(useGps_)
					{
						rtCode = gps_.update();
						if(rtCode==1)
						{
							newGPSData_=true;
						}
					}

					//Updating optitrack
					if(useOptitrack_)
					{
						rtCode = optitrack_.update();
						if(rtCode==1)
						{
							newOptitrackData_=true;
						}
					}

					//Updating main IMU
					rtCode = IMUmain_.update();
					if(IMUmain_.status_ != IMUAbstract::STATUS::RUNNING)
					{
						logger.logln("State Observer - update - main IMU stopped running");
						setStatus(SO_STATUS::BACKUP);
						return;
					}
					else if(rtCode==1)
					{
						uint32_t t0Temp = micros();
						if((t0Temp-iterT_)>SO_DT_TRAGET_LOW)
						{
							if((t0Temp-iterT_)>SO_DT_TRAGET_HIGH)
								overrun_ = 1;
							else
								overrun_ = 0;

							newMainIMUData_=true;
							iterT_ = t0Temp;
							iterCount_++;
							break;
						}
					}
				}

				//Update and publish state
				rtCode = updateState();
				if(rtCode==-2)
				{	
					logger.logln("State Observer - update - updateState failed");
					setStatus(SO_STATUS::BACKUP);
					return;
				}
				else if(rtCode<0)
				{
					setStatus(SO_STATUS::FAILURE);
					return;
				}
				publishState();

				// setStatus(SO_STATUS::BACKUP);
				break;
			}//end nominal case

			case SO_STATUS::BACKUP:
			{
				IMUbackup_.resetTimeout();
				//Updating the sensors
				while(true)
				{
					//Updating gps
					if(useGps_)
					{
						rtCode = gps_.update();
						if(rtCode==1)
						{
							newGPSData_=true;
						}
					}

					//Updating backup IMU
					rtCode = IMUbackup_.update();
					if(IMUbackup_.status_ != IMUAbstract::STATUS::RUNNING)
					{
						logger.logln("State Observer - update - backup IMU stopped running");
						setStatus(SO_STATUS::FAILURE);
						return;
					}
					else if(rtCode==1)
					{
						uint32_t t0Temp = micros();
						if((t0Temp-iterT_)>SO_DT_TRAGET_HIGH_BAK)
							overrun_ = 1;
						else
							overrun_ = 0;

						newBackupIMUData_=true;
						iterT_ = t0Temp;
						break;
					}
				}
				
				//Publish state
				rtCode = updateState();
				if(rtCode==-2)
				{	
					logger.logln("State Observer - update - updateState failed");
					setStatus(SO_STATUS::BACKUP);
					return;
				}
				else if(rtCode<0)
				{
					setStatus(SO_STATUS::FAILURE);
					return;
				}

				publishState();

				break;
			}

			case SO_STATUS::FAILURE:
			{
				if(micros()-displayTnm1_>SO_DISPLAY_FREQUENCY_DT)
				{
					displayTnm1_ = micros();
					logger.logln("State Observer - update - in failure...");
				}

				//Updating gps
				if(useGps_)
				{
					rtCode = gps_.update();
					if(rtCode==1)
					{
						newGPSData_=true;
					}
				}

				break;
			}//end failure case

			default:
			{
				logger.logln("State Observer - update - unknown SO state...");
				setStatus(SO_STATUS::FAILURE);
				break;
			} //end default case

		}// end switch
			
	  //Updating batteryMonitor
		batteryMonitor_.update();

		//Updating xbee wifi
		xbeeWifi_.update();

		//Check SO sessages
		checkS0messages();

		//Updating logger
		logger.logBin(runT_,0);
		logger.logBin(iterT_,1);
		logger.logBin(state_[0],2);
		logger.logBin(state_[1],3);
		logger.logBin(state_[2],4);
		logger.logBin(state_[3],5);
		logger.logBin(state_[4],6);
		logger.logBin(state_[5],7);
		logger.logBin(state_[6],8);
		logger.logBin(state_[7],9);
		logger.logBin(state_[8],10);
		logger.logBin(state_[9],11);
		logger.logBin(state_[10],12);
		logger.logBin(state_[11],13);
		logger.logBin(state_[12],14);
		logger.logBin(state_[13],15);
		logger.logBin(state_[14],16);
		logger.logBin(state_[15],17);
		logger.logBin(state_[16],18);
		logger.logBin(input_[0],19);
		logger.logBin(input_[1],20);
		logger.logBin(input_[2],21);
		logger.logBin(input_[3],22);			
		logger.logBin(batteryMonitor_.getVoltage(),23);	
		logger.logBin(batteryMonitor_.getCurrent(),24);	
		logger.logBin(static_cast<uint32_t>(status_),25);	
		logger.logBin(static_cast<uint32_t>(overrun_),26);	
		logger.logBin(hybridState_,27);	
		logger.logBin(static_cast<uint32_t>(gps_.fix),28);	
		logger.logBin(gps_.latitude,29);	
		logger.logBin(gps_.longitude,30);
		if(iterCount_ % SO_LOG_ITER_RATE == 0)
		{
				logger.comitBin();
		}
		logger.update();

		return;
	}

	void SO::setStatus(SO_STATUS status)
	{
		switch(status)
		{
			case SO_STATUS::NOMINAL:
			{
				logger.logln("State Observer - setStatus - Switching to NOMINAL");
				IMUmain_.resetTimeout();
				break;
			}
			case SO_STATUS::BACKUP:
			{
				logger.logln("State Observer - setStatus - Switching to BACKUP");

				if(IMUbackup_.startStream()!=1)
				{
					logger.logln("State Observer - setStatus - backup IMU startStream failed");
					status = SO_STATUS::FAILURE;
				}
				IMUbackup_.resetTimeout();
				break;
			}
			case SO_STATUS::FAILURE:
			{
				logger.logln("State Observer - setStatus - Switching to FAILURE");
				break;
			}
			default :
			{
				logger.logln("State Observer - setStatus - Invalid status");
				return;
			}
		}
		status_ = status;
	}

	int32_t SO::updateState(void)
	{
		int32_t rtCode;
		float stateNew[STATE_LENGTH];
		memcpy(stateNew,stateNew,STATE_LENGTH*4);

		switch(status_)
		{
			case SO_STATUS::NOMINAL:
			{
				if(YOST_TTS_LX_MODE == IMUAbstract::MODE::FUSIONED)
				{				
					stateNew[6] = IMUmain_.fusionedData_.quat[0];
					stateNew[7] = IMUmain_.fusionedData_.quat[1];
					stateNew[8] = IMUmain_.fusionedData_.quat[2];
					stateNew[9] = IMUmain_.fusionedData_.quat[3];

					stateNew[10] = IMUmain_.fusionedData_.rates[0];
					stateNew[11] = IMUmain_.fusionedData_.rates[1];
					stateNew[12] = IMUmain_.fusionedData_.rates[2];

					if(useGps_)
					{
						if(newGPSData_)
						{

						}
					}
					else
					{
						if(newOptitrackData_)
						{
							stateNew[0] = optitrack_.pos_[0];
							stateNew[1] = optitrack_.pos_[1];
							stateNew[2] = optitrack_.pos_[2];
							stateNew[3] = optitrack_.vel_[0];
							stateNew[4] = optitrack_.vel_[1];
							stateNew[5] = optitrack_.vel_[2];
						}
					}
				}
				else
				{
					if(useGps_)
					{
						rtCode = observerFusionUpdate(state_,
							IMUmain_.rawData_.gyro,
							IMUmain_.rawData_.acc,
							IMUmain_.rawData_.mag,
							gps_.latitudeDegrees,
							gps_.longitudeDegrees,
							gps_.geoidheight,
							stateNew);
					}
					else
					{
						rtCode = observerFusionUpdate(state_,
							IMUmain_.rawData_.gyro,
							IMUmain_.rawData_.acc,
							IMUmain_.rawData_.mag,
							stateNew);
					}

					if(rtCode!=1)
					{
						logger.logln("State Observer - updateState - fusionStates failed");
						return -1;
					}
				}

				if(!checkStateConsistency())
				{
					logger.logln("State Observer - updateState - checkStateConsistency failed");
					return -2;
				}

				memcpy(state_,stateNew,STATE_LENGTH*4);

				break;
			}
			case SO_STATUS::BACKUP:
			{
				state_[6] = IMUbackup_.fusionedData_.quat[0];
				state_[7] = IMUbackup_.fusionedData_.quat[1];
				state_[8] = IMUbackup_.fusionedData_.quat[2];
				state_[9] = IMUbackup_.fusionedData_.quat[3];

				state_[10] = IMUbackup_.fusionedData_.rates[0];
				state_[11] = IMUbackup_.fusionedData_.rates[1];
				state_[12] = IMUbackup_.fusionedData_.rates[2];

				if(useGps_)
				{
					if(newGPSData_)
					{

					}
				}
				else
				{
					if(newOptitrackData_)
					{
						stateNew[0] = optitrack_.pos_[0];
						stateNew[1] = optitrack_.pos_[1];
						stateNew[2] = optitrack_.pos_[2];
						stateNew[3] = optitrack_.vel_[0];
						stateNew[4] = optitrack_.vel_[1];
						stateNew[5] = optitrack_.vel_[2];
					}
				}
				break;
			}
			case SO_STATUS::FAILURE:
			{
				if(useGps_)
				{
					//todo
				}
				return 2;
			}
			default:
			{
				logger.logln("State Observer - updateState - Invalid status");
				return -1;
			}
		}
		newMainIMUData_ = false;
		newInput_ = false;
		newBackupIMUData_ = false;
		newOptitrackData_ = false;
		newGPSData_ = false;
		return 1;
	}

	void SO::publishState(void)
	{
		std::vector<uint8_t> packet;
		num32_t n32;

		//Status
		packet.push_back(static_cast<uint8_t>(status_));

		//Run time
		n32.ui = runT_;
		for(uint32_t j = 0; j<4; j++)
		{
			packet.push_back(n32.c[j]);
		}

		//Iteration time
		n32.ui = iterT_;
		for(uint32_t j = 0; j<4; j++)
		{
			packet.push_back(n32.c[j]);
		}

		//State
		for(uint32_t i = 0; i<STATE_LENGTH; i++)
		{
			n32.f = state_[i];
			for(uint32_t j = 0; j<4; j++)
			{
				packet.push_back(n32.c[j]);
			}
		}

		if(canfd_.sendPacket(broadcastIdx_,packet)!=1)
		{
			logger.logln("State Observer - publishState - canfd_.sendPacket failed");
		}
	}

	int32_t SO::parseTX2(const std::vector<uint8_t> &tx2Packet)
	{
		return -1;
	}

	int32_t SO::observerFusionUpdate(const float stateCurrent[STATE_LENGTH],
																   const float gyro[3],
			                             const float acc[3],
			                             const float mag[3],
			                             float stateNew[STATE_LENGTH])
	{
		stateNew[0] = gyro[0];
		stateNew[1] = gyro[1];
		stateNew[2] = gyro[2];
		stateNew[3] = acc[0];
		stateNew[4] = acc[1];
		stateNew[5] = acc[2];
		stateNew[6] = mag[0];
		stateNew[7] = mag[1];
		stateNew[8] = mag[2];
		return 1;
	}	

	int32_t SO::observerFusionUpdate(const float stateCurrent[STATE_LENGTH],
																   const float gyro[3],
			                             const float acc[3],
			                             const float mag[3],
			                             const float latitude,
			                             const float longitude,
			                             const float geoidheight,
			                             float stateNew[STATE_LENGTH])
	{
		stateNew[0] = gyro[0];
		stateNew[1] = gyro[1];
		stateNew[2] = gyro[2];
		stateNew[3] = acc[0];
		stateNew[4] = acc[1];
		stateNew[5] = acc[2];
		stateNew[6] = mag[0];
		stateNew[7] = mag[1];
		stateNew[8] = mag[2];
		return 1;
	}	

	bool SO::checkStateConsistency(void)
	{
		return true;
	}

	int32_t SO::checkS0messages(void)
	{
		std::vector<uint8_t> packet;
		num32_t sourceAddress;

		int32_t rtCode = xbeeWifi_.getMyPacket(SO_BROADCAST_IDX,sourceAddress.ui,packet);
		if(rtCode==1)
		{
			REQUEST req;
			if(isValidRequest(packet[0]))
			{
				req = static_cast<REQUEST>(packet[0]);
			}
			else
			{
				logger.log("State Observer - checkS0messages - invalid request: ");
				logger.loglnadd(packet[0],HEX);
				return 3;
			}

			switch(req)
			{
				case REQUEST::UPDATE_DATE:
				{

					if(!checkSumOK(packet.data()+1,5))
					{
						logger.log("State Observer - checkS0messages - checksum failed: ");
						logger.loglnadd(packet[5],DEC);
						return 4;
					}
					num32_t n32;
					n32.c[0] = packet[1];
					n32.c[1] = packet[2];
					n32.c[2] = packet[3];
					n32.c[3] = packet[4];
					setTime(static_cast<time_t>(n32.ui));
					Teensy3Clock.set(now());
					logger.log("State Observer - checkS0messages - RTC synchronized to ");
					logger.logadd(static_cast<int32_t>(hour())); logger.logadd(':');
					logger.logadd(static_cast<int32_t>(minute())); logger.logadd(':');
					logger.logadd(static_cast<int32_t>(second()));
					logger.logadd(" - ");
					logger.logadd(static_cast<int32_t>(day()), DEC); logger.logadd('/');
					logger.logadd(static_cast<int32_t>(month()), DEC); logger.logadd("/20");
					logger.loglnadd(static_cast<int32_t>(year()), DEC);
					break;
				}
				case REQUEST::TARE_IMUS:
				{
					logger.logln("State Observer - checkS0messages - IMU Tare requested");
					IMUmain_.tare();
					break;
				}
				case REQUEST::GPS_ENABLE:
				{
					if(packet[1] == 0)
					{
						logger.logln("State Observer - checkS0messages - GPS disable requested");
						logger.loglnadd("State Observer - checkS0messages - restart needed");
						EEPROM.write(USE_GPS_ADDRESS,0);
					}
					else if(packet[1] == 1)
					{
						logger.logln("State Observer - checkS0messages - GPS enable requested");
						logger.loglnadd("State Observer - checkS0messages - restart needed");
						EEPROM.write(USE_GPS_ADDRESS,1);
					}
					else
					{
						logger.logln("State Observer - checkS0messages - unknow GPS state requested");
					}
					break;
				}
				case REQUEST::REZERO_POS:
				{
					logger.logln("State Observer - checkS0messages - Position zeroing request");
					optitrack_.rezero();
					break;
				}
				default :
				{
					logger.log("State Observer - checkS0messages - unkown request: ");
					logger.loglnadd(packet[0],HEX);
					return 5;
				}
			}
			return 1;
		}
		return 2;
	}

	bool SO::isValidRequest(const uint8_t &req)
	{
		if(req == 'D' || req == 'T' || req == 'G' || req == 'Z')
			return true;
		else
			return false;
	}
}