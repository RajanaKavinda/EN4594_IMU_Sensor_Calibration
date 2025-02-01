#define BAUD_RATE     115200
#define VALUE_COUNT   11


#include <Zumo32U4.h>
#include <Wire.h>

Zumo32U4Encoders m_Encoders;
Zumo32U4IMU m_Imu;

char z_ImuReport[120];

///*********************************************************************************
/// Starts I2C and initializes the inertial sensors.
///*********************************************************************************
void InitInertialSensors()
{
  Wire.begin();

  if(!m_Imu.init())
  {
    // Failed to detect the compass.
    ledRed(1);
    while(1)
    {
      //Serial.println(F("Failed to initialize IMU sensors."));
      delay(100);
    }
  }

  m_Imu.enableDefault();  
}

///*********************************************************************************
///
///*********************************************************************************
void setup() {
  // put your setup code here, to run once:
  // Start serial communication at 9600 baud rate
  Serial.begin(BAUD_RATE);
  InitInertialSensors();  
}

///*********************************************************************************
///
///*********************************************************************************
void loop() {
  // put your main code here, to run repeatedly:
  // Send a string over serial
  // Serial.println("Hello, ROS2!");
  m_Imu.read();

  int values[VALUE_COUNT] = {m_Imu.a.x, m_Imu.a.y, m_Imu.a.z,
                             m_Imu.m.x, m_Imu.m.y, m_Imu.m.z,
                             m_Imu.g.x, m_Imu.g.y, m_Imu.g.z, // IMU values
                             m_Encoders.getCountsLeft(), m_Encoders.getCountsRight()}; // encoder values
  
  // Send each value one by one
  for (int i=0; i<VALUE_COUNT; i++) 
  {
    Serial.print(values[i]); // Send the value
    if (i < VALUE_COUNT-1) 
    {
      Serial.print(','); // Send a delimiter (comma) to separate values
    }
  }
  Serial.println(); // Send a newline character at the end of the transmission

  // Wait
  delay(10);
}
