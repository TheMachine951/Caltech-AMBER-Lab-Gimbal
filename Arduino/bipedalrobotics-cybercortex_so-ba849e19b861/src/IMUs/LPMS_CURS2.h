#ifndef __LPMS_CURS2_H_INCLUDED__
#define __LPMS_CURS2_H_INCLUDED__

#include "Arduino.h"
#include "../SharedTools/SharedTools.h"
#include "IMUAbstract.h"

namespace Cybercortex
{

	class LPMS_CURS2  : public IMUAbstract
	{
		enum class COMMAND : uint16_t
		{
			REPLY_ACK             = 0,
			REPLY_NACK            = 1,
			UPDATE_FIRMWARE       = 2,
			UPDATE_IAP            = 3,
			GET_CONFIG            = 4,
			GET_STATUS            = 5,
			GOTO_COMMAND_MODE     = 6,
			GOTO_STREAM_MODE      = 7,
			GET_SENSOR_DATA       = 9,
			SET_TRANSMIT_DATA     = 10,
			SET_STREAM_FREQ       = 11,
			SET_IMU_ID            = 20,
			GET_IMU_ID            = 21,
			SET_GYR_RANGE         = 25,
			GET_GYR_RANGE         = 26,
			SET_ACC_RANGE         = 31,
			GET_ACC_RANGE         = 32,
			SET_MAG_RANGE         = 33,
			GET_MAG_RANGE         = 34,
			SET_FILTER_MODE       = 41,
			GET_FILTER_MODE       = 42,
			SET_FILTER_PRESET     = 43,
			GET_FILTER_PRESET     = 44,
			SET_RAW_DATA_LP       = 60,
			GET_RAW_DATA_LP       = 61,
			RESET_TIMESTAMP       = 66,
			SET_LIN_ACC_COMP_MODE = 67,
			GET_LIN_ACC_COMP_MODE = 68,
			SET_LPBUS_DATA_MODE   = 75,
			SET_UART_BAUDRATE     = 84,
			GET_UART_BAUDRATE     = 85,
			SET_UART_FORMAT       = 86,			
		};

		struct CONFIG
		{
			uint16_t openMATID             = 0x01;
			int32_t streamFreq             = 400;
			int32_t rawStreamData          = 7168;
			int32_t fusionedStreamData     = 327680;
			int32_t uartFormat             = 0;
			int32_t busDataMode            = 0;
			uint32_t rawMessageLength      = 51;
			uint32_t fusionedMessageLength = 43;
		};

	public:
		LPMS_CURS2(const IMUAbstract::MODE &mode,
		           HardwareSerial &port,
		           const uint32_t &baud,
		           const uint32_t &readTimeout,
		           const uint32_t &receiveTimeout,
		           const uint32_t &ackTimeout,
		           const uint32_t &resyncTimeout);
		LPMS_CURS2(const IMUAbstract::MODE &mode,
			         HardwareSerial &port,
			         const uint32_t &baud,
			         const uint32_t &readTimeout,
			         const uint32_t &receiveTimeout,
			         const uint32_t &ackTimeout,
			         const uint32_t &resyncTimeout,
			         const CONFIG &config);
		virtual ~LPMS_CURS2();

		virtual int32_t init();
		virtual int32_t update();
		virtual int32_t startStream();
		virtual int32_t stopStream();
		
	protected:
		HardwareSerial port_;
		const uint32_t baud_;
		const uint32_t readTimeout_;
		const uint32_t ackTimeout_;
		const uint32_t resyncTimeout_;
		const CONFIG config_;
		const uint32_t messageLength_;

		bool newMsgCheck();
		bool newStreamCheck();
		int32_t readMessage(COMMAND &cmd,
		                    std::vector<uint8_t> &data);
		int32_t sendMessage(const COMMAND &cmd,
						            const uint16_t &dataLen,
												const uint8_t data[]);
		int32_t gotoCommandMode();
		int32_t gotoStreamMode();
		int32_t setStreamFreq();
		int32_t setTransmitData();
		int32_t setUartFormat();
		int32_t setLpbusDataMode();
		int32_t reSync();
		int32_t checkACK();
		int32_t parseStream(const std::vector<uint8_t> &data);
		bool isValidCommand(const uint16_t &cmd);

	};

}
#endif