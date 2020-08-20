#include <TeensyVector.h>
#include "SPI.h"
#include "src/IMUs/IMUs.h"

#define YOST_TTS_LX_PIN_SS 9
#define YOST_TTS_LX_PIN_ATT 6
#define YOST_TTS_LX_RECEIVE_TIMEOUT 10000000
#define YOST_TTS_LX_MODE IMUAbstract::MODE::FUSIONED
#define SERVO1_PIN 3
#define SERVO2_PIN 4
#define SERVO_F 100.0F
#define BARO_PIN_SS 10

#define offset1 3
#define offset2 10

YOST_TTS_LX IMUmain(YOST_TTS_LX_MODE,
                    SPI,
                    YOST_TTS_LX_PIN_SS,
                    YOST_TTS_LX_PIN_ATT,
                    YOST_TTS_LX_RECEIVE_TIMEOUT);

void setup(void) {

  Serial.begin(9600);
  SPI.begin();

  delay(1000);

  //  Servo Setup
  pinMode(BARO_PIN_SS,OUTPUT);
  digitalWrite(BARO_PIN_SS,HIGH);
  
  pinMode(SERVO1_PIN,OUTPUT);
  pinMode(SERVO2_PIN,OUTPUT);
  analogWriteResolution(16); // 0 - 65,535
  analogWriteFrequency(SERVO1_PIN,SERVO_F);
  analogWriteFrequency(SERVO2_PIN,SERVO_F);
 

  uint32_t rtCode = IMUmain.init();
  if (rtCode != 1)
  {
    while (true)
    {
      Serial.println("Failed to init IMU");
      delay(1000);
    }
  }

  //  delay(5000);
  //  IMUmain.tare();
}

void loop(void)
{
  uint32_t rtCode;
  if (IMUmain.status_ == IMUAbstract::STATUS::RUNNING)
  {
    while (true)
    {
      rtCode = IMUmain.update();
      if (IMUmain.status_ != IMUAbstract::STATUS::RUNNING)
      {
        Serial.println("IMU stopped running");
        return;
      }
      else if (rtCode == 1)
      {
        break;
      }
    }

    float q[4];

    q[0] = IMUmain.fusionedData_.quat[0];
    q[1] = IMUmain.fusionedData_.quat[1];
    q[2] = IMUmain.fusionedData_.quat[2];
    q[3] = IMUmain.fusionedData_.quat[3];

    //////////////////////////////////////////////////////////////////////////
    float angle0 = atan2((2 * (q[0] * q[3] + q[1] * q[2]) ) , (1 - 2 * (sq(q[2]) + sq(q[3]))));
    float angle1 = asin(2 * (q[0] * q[2] - q[3] * q[1]));
    float angle2 = atan2((2 * (q[0] * q[1] + q[2] * q[3]) ) , (1 - 2 * (sq(q[1]) + sq(q[2]))));

    const float pi = 3.14159;
    angle0 = angle0 * (180 / pi); //About Z-axis
    angle1 = angle1 * (180 / pi); //Then About Y'-axis     (Servo1)
    angle2 = angle2 * (180 / pi); //Finally About X''-axis (Servo2)
    
    // Servo Test
    //analogWrite(SERVO1_PIN,PWM2dt(cos((float)(millis())*0.005)*500.0F + 1500.0F));
    //analogWrite(SERVO2_PIN,PWM2dt(sin((float)(millis())*0.005)*500.0F + 1500.0F));

    Serial.print("Quaternion:  ");
    Serial.print(q[0]); Serial.print(", ");
    Serial.print(q[1]); Serial.print(", ");
    Serial.print(q[2]); Serial.print(", ");
    Serial.print(q[3]);

    Serial.print("  |  Euler:  ");
    Serial.print(angle0); Serial.print(", ");
    Serial.print(angle1); Serial.print(", ");
    Serial.println(angle2);

    analogWrite(SERVO1_PIN,PWM2dt(angle2PWM(angle2+offset1)));
    analogWrite(SERVO2_PIN,PWM2dt(angle2PWM(angle1+offset2)));
    
    
    

    /////////////////////////////////////////////////////////////////////////
  }
  else
  {
    Serial.println("IMU error");
  }

  delay(1);
}

int PWM2dt (float t) {
  float t2;
  const float tMax = 1850;
  const float tMin = 1150;
  
  if(t>tMax)
  {
    t2 = tMax;
  }
  else if(t<tMin)
  {
    t2 = tMin;
  }
  else
  {
    t2 = t;
  }
  return static_cast<int>(0.065535 * (t2*SERVO_F));
}

float angle2PWM (float angle)
{
  return (angle+90) * 10.254 + 549;
}


