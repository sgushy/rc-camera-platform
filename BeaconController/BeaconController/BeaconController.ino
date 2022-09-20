#include <ETH.h>
#include <WiFi.h>

//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "SensorFusion.h"
#include "ESPAsyncWebServer.h"

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// HTTP broadcaster variables -------------------
const char* ssid = "FS-Beacon";
const char* password = "888888888";
AsyncWebServer server(80);
volatile bool broadcasting = false;

// I/O variables ----------------------------
#define SWTC_1 39  // On-Off switch
#define LED_13 13  // RED LED
#define LED_12 12  // YELLOW LED

// GPS variables ----------------------------
static const int RXPin = 17, TXPin = 16;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin); // GPS serial connection

// State machine variables
int state = 0;
TaskHandle_t Calibration;

// MPU status -------------------------------
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus = -1;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Position estimator variables ----------------------
SF fusion;
float x, y, z = 0; // Estimated position in world space (E frame)
float vx, vy, vz = 0; // Estimated position in world space (E frame)
float t_prev = 0; // Last measured time
float t_now = 0; // Current measured time
float dt = 0; // Amount of time since last update
float p_ae_x, p_ae_y, p_ae_z = 0; // The accelerations in the last update;
float ae_x, ae_y, ae_z = 0; // The accelerations currently (earth frame)
// Orientation sensor variables ----------------------------
Quaternion q;           // [w, x, y, z]         Quaternion representation of transformation matrix
VectorInt16 aa;         // [ax, ay, az]            Acceleration sensor (B frame)
VectorInt16 aaReal;     // [ax, ay, az]            Acceleration sensor excluding gravitational measurement (B frame)
VectorInt16 aaWorld;    // [ax, ay, az]            Acceleration in fixed inertial frame (E frame)
VectorFloat gravity;    // [grx, gry, grz]            Gravity vector
float euler[3];         // [psi, theta, phi]    Quaternion to Euler angles
float ypr[3];           // [yaw, pitch, roll]   Yaw/pitch/roll and gravity vector

// IMU cclibration variables ------------------------
volatile bool calibrated = false; // Calibration completed?
volatile bool calibrating = false; // Calibration in process?

int calibrationCounter = 0; // Tracks number of times calibration measurements have been taken

int a_tolerance = 8;   //Acceptable margin of accelerometer error
int g_tolerance = 1;   //Acceptable margin of gyroscope error

float offset_ax, offset_ay, offset_az = 0; // DC offset of acceleration sensors; calibration will take care of this...
float offset_gx, offset_gy, offset_gz = 0; // DC offset of gyroscope sensors; same as above

// IMU variables ----------------------------
//#define SS_PIN PB12
//SPIClass mySPI (2);
//Adafruit_MPU6050 mpu;
MPU6050 mpu;

//Timer interrupt variables ----------------------------
volatile bool interruptCounter0 = false;    // check timer interrupt 0
volatile bool interruptCounter1 = false;     // check timer interrupt 1

volatile bool switch_on = false;

int totalInterrupts0 = 0;   // counts the number of triggering of the alarm
int totalInterrupts1 = 0;   // counts the number of triggering of the alarm

hw_timer_t * timer0 = NULL;
hw_timer_t * timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

//Initialization for interrupts ------------------------------------
void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  interruptCounter0 = true; // the function to be called when timer interrupt is triggered
  totalInterrupts0++;
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void IRAM_ATTR onTime1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  interruptCounter1 = true; // the function to be called when timer interrupt is triggered
  totalInterrupts1++;
  portEXIT_CRITICAL_ISR(&timerMux1);
}

#define EULER_DATA
//#define RAW_DATA
//#define PROCESSING
//#define SERIAL_PLOTER

void setup() {
  // serial to display data
  Serial.begin(115200);

  pinMode(SWTC_1, INPUT);  // configures the specified pin to behave either as an input or an output
  pinMode(LED_12, OUTPUT);
  pinMode(LED_13, OUTPUT);

  ss.begin(GPSBaud); // Start GPS monitoring

  timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered
  timerAlarmWrite(timer0, 1000000, true); // 1000000 * 1 us = 0.5 s, autoreload true

  timer1 = timerBegin(1, 80, true);  // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true); // edge (not level) triggered
  timerAlarmWrite(timer1, 20000, true); // 20000 * 1 us = 20 ms, autoreload true

  // at least enable the timer alarms
  timerAlarmEnable(timer0); // enable
  timerAlarmEnable(timer1); // enable
}

void loop() {
  // put your main code here, to run repeatedly:
  switch (state)
  {
    case 0: // Switch is off
      digitalWrite(LED_12, LOW);
      digitalWrite(LED_13, LOW);
      Wire.begin();
      Wire.setClock(400000);

      if (SWITCH_ON())
      {
        state = 1; // If the switch is on, move to state 1
      }
      else if (BROADCASTING_HTTP())
      {
        DISCONNECT_BROADCASTER(); // Turn off broadcasting when the state is zero...
      }
      break;
    case 1: // Switch is on, and beacon is calibrating
      BLINK_LED_1S(LED_13);
      digitalWrite(LED_12, LOW);
      while (ss.available() > 0) {
        gps.encode(ss.read()); // Check GPS if available
      }
      if (!SWITCH_ON())
      {
        state = 0; // If the switch is off, move to state 0
        ESP.restart(); // Restart the micro controller to clear any variables in RAM
      }
      else
      {
        if (!CALIBRATED())
        {
          if (!CALIBRATING())
            INIT_MPU();
          POSITIONAL_CALIBRATE();
        }
        if (!BROADCASTING_HTTP()) // Start broadcasting if it is not already done so
        {
          INIT_BROADCASTER();
        }
        if (CALIBRATED() && BROADCASTING_HTTP())
        {
          state = 2;
        }
      }
      break;
    case 2: // Beacon is fully functional, but GPS is off
      BLINK_LED_1S(LED_12);
      digitalWrite(LED_13, HIGH);
      while (ss.available() > 0) {
        gps.encode(ss.read()); // Try to read from GPS
      }
      if (GPS_AVAILABLE())
      {
        state = 3; // Switch to state 3 if GPS is valid
      }
      POSITIONAL_UPDATE_DMP();
      if (!SWITCH_ON())
      {
        state = 0; // If the switch is off, move to state 0
        ESP.restart();
      }
      break;
    case 3: // GPS is on (Extra portion; Implement only if there is time)
      digitalWrite(LED_12, HIGH);
      digitalWrite(LED_13, HIGH);
      if (!GPS_AVAILABLE())
      {
        state = 2; break; // Don't use GPS if it is not functional
      } if (!SWITCH_ON())
      {
        state = 0; // If the switch is off, move to state 0
        ESP.restart();
      }
      POSITIONAL_UPDATE_GPS();
      break;
    default:
      BLINK_LED_1S(LED_13); // All lights flash when there is an error
      BLINK_LED_1S(LED_12);
      if (!SWITCH_ON())
      {
        state = 0; // If the switch is off, move to state 0
        ESP.restart();
      }
      break;
  }
}

// SERVICES -----------------------------------

/**
   Calibration of position data
   Calibration routine is designed to take measurements and average them to get a DC offset value.
   It is expected that calibration is done when the beacon is on the mount of the device.
*/
void POSITIONAL_CALIBRATE()
{
  if (!CALIBRATING())
  {
    Serial.println("Starting calibration");
    calibrating = true;
    calibrationCounter = 0;
  }
  if (totalInterrupts1 % 10 == 0)
  {
    calibrationCounter++;
    if (calibrationCounter > 100 && calibrationCounter <= 200)
    {
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      offset_ax += ax;
      offset_ay += ay;
      offset_az += az;
      offset_gx += gz;
      offset_gy += gy;
      offset_gz += gz;
    }
  }
  if (calibrationCounter == 200)
  {
    calibrationCounter++;
    offset_ax /= 500;
    offset_ay /= 500;
    offset_az /= 500;
    offset_gx /= 500;
    offset_gy /= 500;
    offset_gz /= 500;

    mpu.setXAccelOffset(offset_ax);
    mpu.setYAccelOffset(offset_ay);
    mpu.setZAccelOffset(offset_az);
    mpu.setXGyroOffset(offset_gx);
    mpu.setYGyroOffset(offset_gy);
    mpu.setZGyroOffset(offset_gz);

    calibrated = true;

    Serial.println("Calibrated");
    Serial.println(offset_ax);
    Serial.println(offset_ay);
    Serial.println(offset_az);

    Serial.println(offset_gx);
    Serial.println(offset_gy);
    Serial.println(offset_gz);

    devStatus = mpu.dmpInitialize();
    if (devStatus == 0)
    {
      Serial.println("DMP Initialized");
      packetSize = mpu.dmpGetFIFOPacketSize();
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      dmpReady = true;
    }
    else
    {
      state = -1;
      Serial.println("DMP failed to initialize, error code is " + char(devStatus));
    }
  }
}
/*
   Use MPU DMP data to determine the state of this device
   Store data as global variable (This is a very inaccurate backup system)
*/
void POSITIONAL_UPDATE_DMP()
{
  if (dmpReady)
  {
    t_prev = t_now;
    t_now = millis() / 1000.0f;
    dt = t_now - t_prev; // Time since last update
    
    
        float p_ae_x = 0.5f * (aaReal.x - 8192.0f) / 8192.0f * 9.81f; // The previous values
        float p_ae_y = 0.5f * (aaReal.y - 8192.0f) / 8192.0f * 9.81f;
        float p_ae_z = 0.5f * (aaReal.z - 8192.0f) / 8192.0f * 9.81f;

    mpu.resetFIFO();   // Refresh values

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    //wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // Get local orientation (B frame) from MPU DMP
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);

    float psi = euler[0] * 180 / M_PI;
    float theta = euler[1] * 180 / M_PI;
    float phi = euler[2] * 180 / M_PI;

    // Get yaw, pitch, roll from MPU
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float pitch = ypr[1] * 180 / M_PI;

    // Get accelerometer values from MPU
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    
        float ae_x = 0.5f * (aaReal.x - 8192.0f) / 8192.0f * 9.81f;
        float ae_y = 0.5f * (aaReal.y - 8192.0f) / 8192.0f * 9.81f;
        float ae_z = 0.5f * (aaReal.z - 8192.0f) / 8192.0f * 9.81f;
    
        // Now, begin to calculate velocity from these values
        // Midpoint rule estimation (may suffer from inaccuracy)
        vx += 0.5f * (ae_x + p_ae_x) * dt;
        vy += 0.5f * (ae_y + p_ae_y) * dt;
        vz += 0.5f * (ae_z + p_ae_z) * dt;
    
        // Double integral for position
        x += vx * dt + 0.5f * (0.5f * (ae_x + p_ae_x) * dt * dt);
        y += vy * dt + 0.5f * (0.5f * (ae_y + p_ae_y) * dt * dt);
        z += vz * dt + 0.5f * (0.5f * (ae_z + p_ae_z) * dt * dt);

    Serial.println(GetStatus());
    Serial.println(GetOrientation());
    Serial.println(GetPosition());
  }
}

void PRINT_MPU_OUTPUT()
{
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

}

/*
   Use MPU data to determine the state of this device
   Store data as global variable
*/
void POSITIONAL_UPDATE_GPS()
{
  POSITIONAL_UPDATE_DMP();
  //TODO: Fuse GPS and inertial data
}

void CALIBRATE_GPS()
{

}

/*
   Start the MPU unit
*/
void INIT_MPU()
{
  if (!CALIBRATING())
  {
    mpu.initialize();

    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    // set the fll scale range of the gyro- and accelerometer respectively
    mpu.setFullScaleGyroRange(0); //0: 250deg/s | 1: 500deg/s | 2: 1000deg/s | 3: 2000deg/s
    mpu.setFullScaleAccelRange(0); //0: 2g | 1: 4g | 2: 8g | 3: 16g
  }
}

/*
   Start broadcast of coordinates
*/
void INIT_BROADCASTER()
{
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("IP address: ");
  Serial.println(IP);

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", GetStatus().c_str());
  });
  server.on("/orientation", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", GetOrientation().c_str());
  });
  server.on("/position", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", GetPosition().c_str());
  });
  server.begin();

  broadcasting = true;
}

/*
   Shut down broadcasting during a state of 0...
*/
void DISCONNECT_BROADCASTER()
{
  if (BROADCASTING_HTTP())
  {
    server.end();
    WiFi.disconnect();
    broadcasting = false;
  }
}

/*
   Blink the LED every second
*/
void BLINK_LED_1S(int LED_PIN_NUM)
{
  if (totalInterrupts0 % 2 == 0)
  {
    digitalWrite(LED_PIN_NUM, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN_NUM, LOW);
  }
}

// EVENT CHECKERS -----------------------------------
// Is the switch on???
bool SWITCH_ON()
{
  int swtc = digitalRead(SWTC_1);

  if (swtc == 0)
    return false;
  else
    return true;
}

// The process of finishing calibration
bool CALIBRATED()
{
  return calibrated;
}

// The process of calibrating
bool CALIBRATING()
{
  return calibrating;
}

bool BROADCASTING_HTTP()
{
  return broadcasting;
}

bool GPS_AVAILABLE()
{
  if (gps.satellites.isValid())
  {
    return (gps.satellites.value() > 2);
  }
  return false;
}

// Utility functions
String GetStatus()
{
  return "s:" + String(state) + "t:" + dt;
}

String GetOrientation()
{
  return "p:" + String(ypr[1]) + ";r:" + String(ypr[2]) + ";y:" + String(ypr[0]);
}

String GetPosition()
{
  return "x:" + String(x) + ";y:" + String(y) + ";z:" + String(z) + ";vx:" + String(vx) + ";vy:" + String(vy) + ";vz:" + String(vz) + ";ax:" + String(aaReal.x) + ";ay:" + String(aaReal.y) + ";az:" + String(aaReal.z);
}
