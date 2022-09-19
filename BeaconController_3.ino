#include <ETH.h>
#include <WiFi.h>


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

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
#define LED_12 12  // GREEN LED

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

int roll, pitch, yaw = 0;

// IMU cclibration variables ------------------------
volatile bool calibrated = false; // Calibration completed?
volatile bool calibrating = false; // Calibration in process?

int calibrationCounter = 0; // Tracks number of times calibration measurements have been taken

int a_tolerance = 8;   //Acceptable margin of accelerometer error
int g_tolerance = 1;   //Acceptable margin of gyroscope error

float pitch_offset, roll_offset, yaw_offset = 0; // Angular offsets

// IMU variables ----------------------------
//#define SS_PIN PB12
//SPIClass mySPI (2);
Adafruit_MPU6050 mpu;
//MPU6050 mpu;

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

// setting PWM properties ----------------------------
const int freq = 5000;
const int ledChannel_1 = 1;
const int ledChannel_2 = 2;
const int resolution = 8;
int MAX_PWM_VOLTAGE = 255;

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

  //ss.begin(GPSBaud); // Start GPS monitoring
  Wire.begin();
  Wire.setClock(400000);
  
  timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered
  timerAlarmWrite(timer0, 1000000, true); // 2000000 * 1 us = 1 s, autoreload true

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

      if (SWITCH_ON())
      {
        state = 1; // If the switch is on, move to state 1
      }
      break;
    case 1: // Switch is on, and beacon is calibrating
      BLINK_LED_1S(LED_13);
      digitalWrite(LED_12, LOW);
      //      while (ss.available() > 0) {
      //        gps.encode(ss.read()); // Check GPS if available
      //      }
      if (!SWITCH_ON())
      {
        state = 0; // If the switch is off, move to state 0
        ESP.restart(); // Restart the micro controller to clear any variables in RAM
      }
      else
      {
        if (!CALIBRATING() && !CALIBRATED())
        {
          INIT_MPU();
        }

        POSITIONAL_CALIBRATE();

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
      //      while (ss.available() > 0) {
      //        gps.encode(ss.read()); // Try to read from GPS
      //      }
      //      if (GPS_AVAILABLE())
      //      {
      //        state = 3; // Switch to state 3 if GPS is valid
      //      }

      POSITIONAL_UPDATE_M();

      if (!SWITCH_ON())
      {
        state = 0; // If the switch is off, move to state 0
        ESP.restart(); // Restart the micro controller to clear any variables in RAM
      }
      break;
    //    case 3: // GPS is on (Extra portion; Implement only if there is time)
    //      digitalWrite(LED_12, HIGH);
    //      digitalWrite(LED_13, HIGH);
    //      if (!GPS_AVAILABLE())
    //      {
    //        state = 2; break; // Don't use GPS if it is not functional
    //      } if (!SWITCH_ON())
    //      {
    //        state = 0; // If the switch is off, move to state 0
    //      }
    //      POSITIONAL_UPDATE_GPS();
    //      break;
    default:
      BLINK_LED_1S(LED_13); // All lights flash when there is an error
      BLINK_LED_1S(LED_12);
      if (!SWITCH_ON())
      {
        state = 0; // If the switch is off, move to state 0
        ESP.restart(); // Restart the micro controller to clear any variables in RAM
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
  if (!calibrating)
  {
    Serial.println("Begin calibration");
    totalInterrupts0 = 0;
    calibrating = true;
    calibrationCounter = 0;
  }
  else if (totalInterrupts0 >= 6 && totalInterrupts0 < 12) // wait 10 seconds before calibrating starts, so that MPU readings will stabilize
  {
    calibrationCounter++;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    dt = fusion.deltatUpdate();
    fusion.MadgwickUpdate(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x, a.acceleration.y, a.acceleration.z, dt);  // Sensor fusion

    roll_offset += fusion.getRoll();
    pitch_offset += fusion.getPitch();
    yaw_offset += fusion.getYaw();
  }
  else if (totalInterrupts0 == 12)
  {
    roll_offset /= calibrationCounter;
    pitch_offset /= calibrationCounter;
    yaw_offset /= calibrationCounter;

    //
    //    if (roll_offset < 0)
    //      //roll_offset = 360 + roll_offset;
    //    if (pitch_offset < 0)
    //      //pitch_offset = 360 + pitch_offset;
    //    if (yaw_offset < 0)
    //      //yaw_offset = 360 + yaw_offset;

    calibrated = true;
    calibrating = false;
  }
}
/*
   Use MPU data to determine the state of this device
   Store data as global variable
   This does not use DMP, but external madgwick filter to do so
*/
void POSITIONAL_UPDATE_M()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  dt = fusion.deltatUpdate();
  fusion.MadgwickUpdate(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x, a.acceleration.y, a.acceleration.z, dt);  // Sensor fusion

  roll = fusion.getRoll();
  pitch = fusion.getPitch();
  yaw = fusion.getYaw();
  //
  //  if (roll < 0)
  //    roll = 360 + roll;
  //  if (pitch < 0)
  //    pitch = 360 + pitch;
  //  if (yaw < 0)
  //    yaw = 360 + yaw;
  //
  // if (roll-roll_offset < 0)
  //    roll = 360 + (roll-roll_offset);
  //    else
  //
  //  if (pitch-pitch_offset < 0)
  //    pitch = 360 + (pitch-pitch_offset);
  //  if (yaw-yaw_offset < 0)
  //    yaw = 360 + (yaw-yaw_offset);

  roll -= roll_offset;
  pitch -= pitch_offset;
  yaw -= yaw_offset;

  if (roll < 0)
    roll = 360 + (roll % 360);
  if (pitch < 0)
    pitch = 360 + (pitch % 360);
  if (yaw < 0)
    yaw = 360 + (yaw % 360);

  Serial.println(GetOrientation());
}

/*
   Use MPU data to determine the state of this device
   Store data as global variable
*/
void POSITIONAL_UPDATE_GPS()
{
  // Not implemented
}

void CALIBRATE_GPS()
{
  // Not implemented
}

/*
   Start the MPU unit
*/
void INIT_MPU()
{
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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

  //  server.on("/status", HTTP_GET, [](AsyncWebServerRequest * request) {
  //    request->send_P(200, "text/plain", GetStatus().c_str());
  //  });
  server.on("/orientation", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", GetOrientation().c_str());
  });
  //  server.on("/position", HTTP_GET, [](AsyncWebServerRequest * request) {
  //    request->send_P(200, "text/plain", GetPosition().c_str());
  //  });
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
  //return "s:" + String(state) + "t:" + dt;
  return "";
}

String GetOrientation()
{
  //return "p:" + String(ypr[1]) + ";r:" + String(ypr[2]) + ";y:" + String(ypr[0]);
  return String(state) + ',' + String(yaw) + ',' + String(pitch) + ',' + String(roll);
}

String GetPosition()
{
  //return "x:" + String(x) + ";y:" + String(y) + ";z:" + String(z) + ";vx:" + String(vx) + ";vy:" + String(vy) + ";vz:" + String(vz) + ";ax:" + String(aaReal.x) + ";ay:" + String(aaReal.y) + ";az:" + String(aaReal.z);
  return "";
}
