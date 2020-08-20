#ifndef __SO_CONFIG_H_INCLUDED__
#define __SO_CONFIG_H_INCLUDED__

///////////////////SO////////////////////
#define SO_BROADCAST_PORT 1560
#define SO_BROADCAST_FREQUENCY_DT 55000 //in microseconds 
#define SO_DISPLAY_FREQUENCY_DT 1000000 //in microseconds
#define SO_DT_TRAGET_LOW 1500 //in microseconds
#define SO_DT_TRAGET_HIGH 2500 //in microseconds
#define SO_DT_TRAGET_HIGH_BAK 3000 //in microseconds
#define SO_LOG_ITER_RATE 20 //in number of iteration
/////////////////////////////////////////

//////////////////CANFD//////////////////
#define CANFD_PIN_SS 24
#define CANFD_PIN_INT 25
#define CANFD_PIN_INT0 26
#define CANFD_PIN_INT1 27
#define CANFD_READ_TIMEOUT 20
/////////////////////////////////////////

//////////////LPMS_CURS2/////////////////
#define LPMS_CURS2_PORT Serial1
#define LPMS_CURS2_BAUD 921600
#define LPMS_CURS2_READ_TIMEOUT 5
#define LPMS_CURS2_RECEIVE_TIMEOUT 50000
#define LPMS_CURS2_ACK_TIMEOUT 10
#define LPMS_CURS2_RESYNC_TIMEOUT 10
/////////////////////////////////////////

///////////////SENSOR SPI////////////////
#define SO_PIN_SCK 20
#define SO_PIN_MISO 5
#define SO_PIN_MOSI 21
/////////////////////////////////////////

//////////////YOST_TTS_LX////////////////
#define YOST_TTS_LX_PIN_SS 2
#define YOST_TTS_LX_PIN_ATT 6
#define YOST_TTS_LX_RECEIVE_TIMEOUT 10000
#define YOST_TTS_LX_MODE IMUAbstract::MODE::FUSIONED // IMUAbstract::MODE::FUSIONED or IMUAbstract::MODE::RAW
/////////////////////////////////////////

/////////////BATTERY_SENSOR//////////////
#define BATTERY_CURRENT_PIN A22
#define BATTERY_CURRENT_TAU 300 //in milliseconds
#define BATTERY_VOLTAGE_PIN A21
#define BATTERY_VOLTAGE_TAU 300 //in milliseconds
/////////////////////////////////////////

//////////////ADAFRUIT_GPS/////////////////
#define USE_GPS_ADDRESS 4
#define ADAFRUIT_GPS_WAIT_FOR_FIX true
#define ADAFRUIT_GPS_PORT Serial5
#define ADAFRUIT_GPS_PIN_EN 23
#define ADAFRUIT_GPS_PIN_PPS 22
#define ADAFRUIT_GPS_WAIT_FOR_FIX_TIMEOUT 60 //in seconds
/////////////////////////////////////////

///////////////XBEE_WIFI/////////////////
#define XBEE_WIFI_PIN_SS 28
#define XBEE_WIFI_PIN_ATT 29
#define XBEE_WIFI_PIN_DOUT 30
#define XBEE_WIFI_PIN_RESET 36
#define XBEE_WIFI_MAX_INIT_TRIALS 5
/////////////////////////////////////////

///////////////OPTITRACK/////////////////
#define OPTITRACK_TIMEOUT 100 //in milliseconds
/////////////////////////////////////////

#endif