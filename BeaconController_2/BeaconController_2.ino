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


// Orientation sensor variables ----------------------------
Quaternion q;           // [w, x, y, z]         Quaternion representation of transformation matrix
VectorInt16 aa;         // [ax, ay, az]            Acceleration sensor (B frame)
VectorInt16 aaReal;     // [ax, ay, az]            Acceleration sensor excluding gravitational measurement (B frame)
VectorInt16 aaWorld;    // [ax, ay, az]            Acceleration in fixed inertial frame (E frame)
VectorFloat gravity;    // [grx, gry, grz]            Gravity vector
float euler[3];         // [psi, theta, phi]    Quaternion to Euler angles
float ypr[3];           // [yaw, pitch, roll]   Yaw/pitch/roll and gravity vector

int16_t ax, ay, az;
int16_t gx, gy, gz;

// IMU cclibration variables ------------------------
volatile bool calibrated = false; // Calibration completed?
volatile bool calibrating = false; // Calibration in process?

int calibrationCounter = 0; // Tracks number of times calibration measurements have been taken

int a_tolerance = 8;   //Acceptable margin of accelerometer error
int g_tolerance = 1;   //Acceptable margin of gyroscope error

float offset_ax, offset_ay, offset_az = 0; // DC offset of acceleration sensors; calibration will take care of this...
float offset_gx, offset_gy, offset_gz = 0; // DC offset of gyroscope sensors; same as above

float mean_ax, mean_ay, mean_az = 0; // Intermediate variables used in calibration
float mean_gx, mean_gy, mean_gz = 0; // Intermediate variables used in calibration

int i_offset_ax, i_offset_ay, i_offset_az, i_offset_gx, i_offset_gy, i_offset_gz = 0; // Initial offsets of variables

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

  ss.begin(GPSBaud); // Start GPS monitoring

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
        if (!CALIBRATING() && !CALIBRATED())
        {
          calibrating = true;
          xTaskCreatePinnedToCore(Calibration_Coroutine, "Calibration", 10000, NULL, 1, &Calibration, 0); // Start calibration on core 0 of processor
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

      //POSITIONAL_UPDATE_DMP(); // Will not use DMP for now, it seems to suffer from excessive noise
      POSITIONAL_UPDATE_M();

      if (!SWITCH_ON())
      {
        state = 0; // If the switch is off, move to state 0
        ESP.restart(); // Restart the micro controller to clear any variables in RAM
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
      }
      POSITIONAL_UPDATE_GPS();
      break;
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

// Run IMU calibration on second core, to prevent interruptions
void Calibration_Coroutine( void * pvParameters ) {
  Serial.print("Calibration of IMU running on core ");
  Serial.println(xPortGetCoreID());

  Serial.print("Zeroing out position and velocity estimation");
  x, y, z = 0; // Reset position estimator during calibration
  vx, vy, vz = 0; // Reset velocity estimator

  int pseudostate = 0; // Calibration state machine (a sub-state machine)
  calibrationCounter = 0; // Number of calibration loops conducted

  while (!CALIBRATED())
  {
    if (!SWITCH_ON())
    {
      delay(50);
      break; // Stop calibration if switch turned off
    }

    switch (pseudostate)
    {
      case 0:
        INIT_MPU();
        //ITERATE_OFFSETS();
        pseudostate = 1;
        break;
      case 1:
        Serial.println("\nCalculating offsets...");
        POSITIONAL_CALIBRATE();
        if (calibrated) {
          Serial.println("\nCalibration successful!");
        }
        else {
          Serial.println("\nCalibration failed!");
        }
        pseudostate++;
        break;
      case 2:
        ITERATE_OFFSETS();
        Serial.println("\nFINISHED!");
        Serial.print("\nSensor readings with offsets:\t");
        Serial.print(mean_ax);
        Serial.print("\t");
        Serial.print(mean_ay);
        Serial.print("\t");
        Serial.print(mean_az);
        Serial.print("\t");
        Serial.print(mean_gx);
        Serial.print("\t");
        Serial.print(mean_gy);
        Serial.print("\t");
        Serial.println(mean_gz);
        Serial.print("Your offsets:\t");
        Serial.print(offset_ax + i_offset_ax);
        Serial.print("\t");
        Serial.print(offset_ay + i_offset_ay);
        Serial.print("\t");
        Serial.print(offset_az + i_offset_az);
        Serial.print("\t");
        Serial.print(offset_gx + i_offset_gx);
        Serial.print("\t");
        Serial.print(offset_gy + i_offset_gy);
        Serial.print("\t");
        Serial.println(offset_gz + i_offset_gz);
        Serial.println("\nIn the format of: aX aY aZ gX gY gZ");

        break;
    }
  }



  delay(5);

  // turn on the DMP, now that it's ready
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true); // Turn on MPU DMP

  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();

  dmpReady = true;
  Serial.println(F("DMP enabled..."));
  vTaskDelete( NULL );
}

// SERVICES -----------------------------------

/**
   Calibration of position data
   Calibration routine is designed to take measurements and average them to get a DC offset value.
   It is expected that calibration is done when the beacon is on the mount of the device.
*/
void POSITIONAL_CALIBRATE()
{
  //  offset_ax = -mean_ax / 8;
  //  offset_ay = -mean_ay / 8;
  //  offset_az = (16384 - mean_az) / 8;
  //
  //  offset_gx = -mean_gx / 4;
  //  offset_gy = -mean_gy / 4;
  //  offset_gz = -mean_gz / 4;
  //  while (1) {
  //    int ready = 0;
  //    mpu.setXAccelOffset(offset_ax + i_offset_ax);
  //    mpu.setYAccelOffset(offset_ay + i_offset_ay);
  //    mpu.setZAccelOffset(offset_az + i_offset_az);
  //
  //    mpu.setXGyroOffset(offset_gx + i_offset_gx);
  //    mpu.setYGyroOffset(offset_gy + i_offset_gy);
  //    mpu.setZGyroOffset(offset_gz + i_offset_gz);
  //
  //    ITERATE_OFFSETS();
  //    Serial.println("...");
  //
  //    if (abs(mean_ax) <= a_tolerance) ready++;
  //    else offset_ax = offset_ax - mean_ax / a_tolerance;
  //
  //    if (abs(mean_ay) <= a_tolerance) ready++;
  //    else offset_ay = offset_ay - mean_ay / a_tolerance;
  //
  //    if (abs(16384 - mean_az) <= a_tolerance) ready++;
  //    else offset_az = offset_az + (16384 - mean_az) / a_tolerance;
  //
  //    if (abs(mean_gx) <= g_tolerance) ready++;
  //    else offset_gx = offset_gx - mean_gx / (g_tolerance + 1);
  //
  //    if (abs(mean_gy) <= g_tolerance) ready++;
  //    else offset_gy = offset_gy - mean_gy / (g_tolerance + 1);
  //
  //    if (abs(mean_gz) <= g_tolerance) ready++;
  //    else offset_gz = offset_gz - mean_gz / (g_tolerance + 1);
  //
  //    if (ready == 6)
  //    {
  //      calibrated = true;
  //      calibrating = false;
  //      break;
  //    }
  //  }
  //  Serial.print("Resulting offset calibration value a/g:\t");
  //  Serial.print(offset_ax + i_offset_ax); Serial.print("\t");
  //  Serial.print(offset_ay + i_offset_ay); Serial.print("\t");
  //  Serial.print(offset_az + i_offset_az); Serial.print("\t");
  //  Serial.print(offset_gx + i_offset_gx); Serial.print("\t");
  //  Serial.print(offset_gy + i_offset_gy); Serial.print("\t");
  //  Serial.println(offset_gz + i_offset_gz);
  //  calibrationCounter = calibrationCounter + 1;
  //  Serial.print("Loop Cnt: "); Serial.println(calibrationCounter);
  //  if (calibrationCounter == 100)
  //  {
  //    return; // exit the calibration routine if no stable results can be obtained after 100 calibration loops
  //  }
  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();

  calibrated = true;
  calibrating = false;
}


void ITERATE_OFFSETS()
{
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  int16_t ax, ay, az, gx, gy, gz;
  while (i < (1000 + 100 + 1)) {
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (1000 + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (1000 + 100)) {
      mean_ax = buff_ax / 1000;
      mean_ay = buff_ay / 1000;
      mean_az = buff_az / 1000;
      mean_gx = buff_gx / 1000;
      mean_gy = buff_gy / 1000;
      mean_gz = buff_gz / 1000;
    }
    i++;
    delay(2);
  }

  Serial.print("Results of measurements a/g:\t");
  Serial.print(mean_ax); Serial.print("\t");
  Serial.print(mean_ay); Serial.print("\t");
  Serial.print(mean_az); Serial.print("\t");
  Serial.print(mean_gx); Serial.print("\t");
  Serial.print(mean_gy); Serial.print("\t");
  Serial.println(mean_gz);
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
    //
    //
    //        float p_ae_x = 0.5f * (aaWorld.x) / 8192.0f * 9.81f; // The previous values
    //        float p_ae_y = 0.5f * (aaWorld.y) / 8192.0f * 9.81f;
    //        float p_ae_z = 0.5f * (aaWorld.z) / 8192.0f * 9.81f;

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
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    //        float ae_x = 0.5f * (aaWorld.x) / 8192.0f * 9.81f;
    //        float ae_y = 0.5f * (aaWorld.y) / 8192.0f * 9.81f;
    //        float ae_z = 0.5f * (aaWorld.z) / 8192.0f * 9.81f;
    //
    //        // Now, begin to calculate velocity from these values
    //        // Midpoint rule estimation (may suffer from inaccuracy)
    //        vx += 0.5f * (ae_x + p_ae_x) * dt;
    //        vy += 0.5f * (ae_y + p_ae_y) * dt;
    //        vz += 0.5f * (ae_z + p_ae_z) * dt;
    //
    //        // Double integral for position
    //        x += vx * dt + 0.5f * (0.5f * (ae_x + p_ae_x) * dt * dt);
    //        y += vy * dt + 0.5f * (0.5f * (ae_y + p_ae_y) * dt * dt);
    //        z += vz * dt + 0.5f * (0.5f * (ae_z + p_ae_z) * dt * dt);

    //Serial.println(GetStatus());
    Serial.println(GetOrientation());
    // Serial.println(GetPosition());
  }
}

/*
   Use MPU data to determine the state of this device
   Store data as global variable
   This does not use DMP, but external madgwick filter to do so
*/
void POSITIONAL_UPDATE_M()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read MPU measurements

float abx,aby,abz,gbx,gby,gbz = 0;
  abx = float(ax)*0.5f / 8192.0f * 9.81f;
  aby = float(ay)*0.5f / 8192.0f * 9.81f;
  abz = float(az)*0.5f / 8192.0f * 9.81f;
  gbx = float(gx)/131.0f;
  gby = float(gy)/131.0f;
  gbz = float(gz)/131.0f;
  dt = fusion.deltatUpdate();
  fusion.MadgwickUpdate(gbx, gby, gbz, abx, aby, abz, dt);  // Sensor fusion

  ypr[0] = fusion.getRoll();
  ypr[1] = fusion.getPitch();
  ypr[2] = fusion.getYaw();

  Serial.println(GetOrientation());
}

/*
   Use MPU data to determine the state of this device
   Store data as global variable
*/
void POSITIONAL_UPDATE_GPS()
{

}

void CALIBRATE_GPS()
{

}

/*
   Start the MPU unit
*/
void INIT_MPU()
{
  mpu.initialize();
  delay(5);

  devStatus = mpu.dmpInitialize();
  if (devStatus != 0)
  {
    state = -1; // Exit into error state if MPU couldn't initialize
    return;
  }
  else
  {
    Serial.println("DMP Initialized");
  }

  mpu.setXAccelOffset(i_offset_ax);
  mpu.setYAccelOffset(i_offset_ax);
  mpu.setZAccelOffset(i_offset_ax);
  mpu.setXGyroOffset(i_offset_ax);
  mpu.setYGyroOffset(i_offset_ax);
  mpu.setZGyroOffset(i_offset_ax);

  // set the fll scale range of the gyro- and accelerometer respectively
  mpu.setFullScaleGyroRange(0); //0: 250deg/s | 1: 500deg/s | 2: 1000deg/s | 3: 2000deg/s
  mpu.setFullScaleAccelRange(0); //0: 2g | 1: 4g | 2: 8g | 3: 16g
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
  return String(state) + ',' + String(ypr[0]) + ',' + String(ypr[1]) + ',' + String(ypr[2]);
}

String GetPosition()
{
  //return "x:" + String(x) + ";y:" + String(y) + ";z:" + String(z) + ";vx:" + String(vx) + ";vy:" + String(vy) + ";vz:" + String(vz) + ";ax:" + String(aaReal.x) + ";ay:" + String(aaReal.y) + ";az:" + String(aaReal.z);
  return "";
}
