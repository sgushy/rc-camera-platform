#include <MPU9250.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>

#include "SensorFusion.h"

// HTTP antenna variables -------------------
const char* ssid = "FS-Beacon";
const char* password = "888888888";
const char* statusChannel = "http://192.168.4.1/status";
const char* positionChannel = "http://192.168.4.1/position";
const char* orientationChannel = "http://192.168.4.1/orientation";
bool httpConnecting = false; // Is connection attempt being made?
bool httpConnected = false; // Is connection done?

TaskHandle_t HTTPSENDRECEIVE;

// I/O variables ----------------------------
#define SWTC_1 39  // On-Off switch
#define EMD_1 26  // Traverse motor driver - direction 1
#define EMD_2 25  // Traverse motor driver - direction 2
#define TMD_1 33  // Elevator motor driver - direction 2
#define TMD_2 15  // Elevator motor driver - direction 2
#define LED_13 13  // YELLOW LED
#define LED_12 12  // RED LED

SF fusion;
// Measurement and control variables ---------------------------------
float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float pitch, roll, yaw; // The pitch, roll, and yaw of the camera platform
float pitch_rate, yaw_rate; // The rates of pitch and yaw (roll should not change for the device)
float deltat;
float pitch_des, yaw_des; // 2 axis camera platform can have pitch and yaw be controlled
float pitch_rate_des, yaw_rate_des; // 2 axis camera platform can have pitch and yaw be controlled

int state = 0;

// Set up IMU sensor -------------------------------------
MPU9250 IMU(Wire, 0x68);
int status;

// IMU cclibration variables ---------------------------------
volatile bool calibrated = false; // Calibration completed?
volatile bool calibrating = false; // Calibration in process?
int calibrationStage = 0; // 0 = find z-offset, 1 = level z, 2 = find x,y offset, 3 = calibrate gyros, 4 = calibrate mag sensor
int calibrationCounter = 0; // Tracks number of times calibration measurements have been taken

// Platform calibration variables
short directionTowardsLevel = 0; // Indicates the direction of elevation offset from a level surface; -1 for below horizon and 1 for above
float tempZMeas = 0.0f;
float maxGravZ = 0.0f; // The maximum value of gravity detected along z-axis; this is the position where the platform is fully level
const float g = 9.81f; // The gravitational acceleration on Earth (we will subtract this from maxGravZ to find the accelerometer z-offset, which we then use to auto-level the platform)

// Remote state variables ---------------------------
float b_X, b_Y, b_Z = 0; // x,y,z positions (currently unused due to lack of good estimator)
int b_Pitch, b_Roll, b_Yaw = 0; // Angular orientation (degrees)
int b_State = 0; // Current state of the remote

int commaIndex = 0; // Intermediate variable for parsing the HTTP message
String val = ""; // Intermediate variable for parsing the HTTP message

const long interval = 200; // Will check for updates every 1/5 second
unsigned long previousMillis = 0;

// PWM Characteristics ------------------------------
const int freq = 5000;
const int ledChannel_1 = 1;
const int ledChannel_2 = 2;

const int ledChannel_3 = 3;
const int ledChannel_4 = 4;

const int resolution = 8;
const int MAX_PWM_VOLTAGE = 255;
const int NOM_PWM_VOLTAGE = 150;

// Motor PWM signals ----------------------------
int TP = 0; // Traversal motor PWM signal
int EP = 0; // Elevator motor PWM signal

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(SWTC_1, INPUT);  // configures the specified pin to behave either as an input or an output
  pinMode(LED_12, OUTPUT);
  pinMode(LED_13, OUTPUT);
  pinMode(TMD_1, OUTPUT);
  pinMode(TMD_2, OUTPUT);
  pinMode(EMD_1, OUTPUT);
  pinMode(EMD_2, OUTPUT);

  // configure LED PWM functionalitites
  ledcSetup(ledChannel_1, freq, resolution);
  ledcSetup(ledChannel_2, freq, resolution);

  // configure LED PWM functionalitites
  ledcSetup(ledChannel_3, freq, resolution);
  ledcSetup(ledChannel_4, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(TMD_2, ledChannel_2);
  ledcAttachPin(TMD_1, ledChannel_1);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(EMD_2, ledChannel_4);
  ledcAttachPin(EMD_1, ledChannel_3);

  timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered
  timerAlarmWrite(timer0, 1000000, true); // 1000000 * 1 us = 0.5 s, autoreload true

  timer1 = timerBegin(1, 80, true);  // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true); // edge (not level) triggered
  timerAlarmWrite(timer1, 20000, true); // 20000 * 1 us = 20 ms, autoreload true

  // at least enable the timer alarms
  timerAlarmEnable(timer0); // enable
  timerAlarmEnable(timer1); // enable

  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    state = -1; // If the IMU could not initialize, then go into the error state
    return;
  }

  // setting the accelerometer full scale range to +/-8G
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
}

void loop()
{
  switch (state)
  {
    case 0: // Switch is off (Do nothing at all)
      digitalWrite(LED_12, LOW);
      digitalWrite(LED_13, LOW);
      STOP_ELEVATOR();  // Stop all motors
      STOP_TRAVERSE();
      if (SWITCH_ON())
      {
        state = 1;
      }
      break;
    case 1: // Calibrate our platform
      digitalWrite(LED_12, HIGH);
      digitalWrite(LED_13, LOW);

      //STOP_ELEVATOR();
      //STOP_TRAVERSE();

      POSITIONAL_CALIBRATE(); // Calibrate the IMU and auto-level the platform

      if (CALIBRATED() && SWITCH_ON())
      {
        state = 2; // If calibration is complete
      }

      if (!SWITCH_ON())
      {
        state = 0;
        digitalWrite(LED_12, LOW);
        digitalWrite(LED_13, LOW);
        STOP_ELEVATOR(); // Stop all motors
        STOP_TRAVERSE();
        ESP.restart(); // Restart the micro controller to clear any variables in RAM
      }
      break;
    case 2: // Try to connect to HTTP channel
      if (HTTPSENDRECEIVE == NULL)
        xTaskCreatePinnedToCore(HTTPCoroutine, "HTTPSENDRECEIVE", 10000, NULL, 0, &HTTPSENDRECEIVE, 0); // Start HTTP on core 0 of processor, to make sure motor is not blocked if there is lag in connection

      STOP_ELEVATOR(); // Stop all motors while connecting
      STOP_TRAVERSE();

      BLINK_LED_1S(LED_13);
      digitalWrite(LED_12, HIGH);
      if (!SWITCH_ON())
      {
        state = 0;
        digitalWrite(LED_12, LOW);
        digitalWrite(LED_13, LOW);
        STOP_ELEVATOR(); // Stop all motors
        STOP_TRAVERSE();
        ESP.restart(); // Restart the micro controller to clear any variables in RAM
      }
      else if (CONNECTED_HTTP())
      {
        state = 3; // Connected? Go to state 3
      }
      break;

    case 3: // Receiving data (actuating as necessary)
      digitalWrite(LED_12, HIGH);
      digitalWrite(LED_13, HIGH);

      POSITIONAL_UPDATE(); // Update own position from IMU readings

      DETERMINE_MOVEMENT_REQ(); // Calculate required motor PWM

      ACTUATE_TRAVERSE(); // Actuate traverse
      ACTUATE_ELEVATOR(); // Actuate elevator

      if (!CONNECTED_HTTP())
      {
        state = 2;
      }
      if (!SWITCH_ON())
      {
        state = 0;
        digitalWrite(LED_12, LOW);
        digitalWrite(LED_13, LOW);
        STOP_ELEVATOR(); // Stop all motors
        STOP_TRAVERSE();
        ESP.restart(); // Restart the micro controller to clear any variables in RAM
      }
      break;
    default:
      BLINK_LED_1S(LED_13); // All lights flash when there is an error
      BLINK_LED_1S(LED_12);
      if (!SWITCH_ON())
      {
        state = 0; // If the switch is off, move to state 0
        digitalWrite(LED_12, LOW);
        digitalWrite(LED_13, LOW);
        STOP_ELEVATOR(); // Stop all motors
        STOP_TRAVERSE();
        ESP.restart(); // Restart the micro controller to clear any variables in RAM
      }
      break;
  }
}

/*
   Run HTTP protocol on second core, so that internet lag does not interrupt the function of the actuator
*/
void HTTPCoroutine( void* pvParameters)
{
  for (;;) {
    switch (state)
    {
      case 2:
        HTTP_TRY_CONNECTION();
        break;
      case 3:
        HTTP_RECEIVE_MSG(); // Receive data from the remote
        break;
    }
    delay(25);
  }
}

// SERVICES ------------------------------------

//Event Service Responses
void STOP_ELEVATOR() {
  ledcWrite(ledChannel_4, LOW);
  ledcWrite(ledChannel_3, LOW);
}
void STOP_TRAVERSE() {
  ledcWrite(ledChannel_2, LOW);
  ledcWrite(ledChannel_1, LOW);
}

void ACTUATE_ELEVATOR() {
  if (EP > 0)
  {
    ledcWrite(ledChannel_3, LOW);
    ledcWrite(ledChannel_4, abs(EP));
  }
  else if (EP < 0)
  {
    ledcWrite(ledChannel_3, abs(EP));
    ledcWrite(ledChannel_4, LOW);
  }
  else
  {
    STOP_ELEVATOR();
  }
}

void ACTUATE_TRAVERSE() {
  if (TP > 0)
  {
    ledcWrite(ledChannel_1, LOW);
    ledcWrite(ledChannel_2, abs(TP));
  }
  else if (TP < 0)
  {
    ledcWrite(ledChannel_1, abs(TP));
    ledcWrite(ledChannel_2, LOW);
  }
  else
  {
    STOP_TRAVERSE();
  }
}

/**
   Use MPU data to determine the state of this device
  Store data as global variable
*/
void POSITIONAL_UPDATE()
{
  IMU.readSensor(); // Update IMU

  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();
  temp = IMU.getTemperature_C();

  deltat = fusion.deltatUpdate(); // Update time
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //Estimate the orientation of the platform using the filtering algorithm

  roll = fusion.getRoll();
  pitch = fusion.getPitch();
  yaw = fusion.getYaw();

  // display the data
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.print("\t");
  Serial.print(mx);
  Serial.print("\t");
  Serial.print(my);
  Serial.print("\t");
  Serial.print(mz);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(), 6);
  Serial.println(roll);
  Serial.println(pitch);
  Serial.println(yaw);
  Serial.println("-");
}

// For calibration of the sensor
void POSITIONAL_CALIBRATE()
{
  switch (calibrationStage)
  {
    case 0: // Begin calibration
      calibrating = true;
      Serial.println("Calibration started...");
      calibrationCounter = 0; // This will be used as a "timer" for calibration

      IMU.readSensor();

      ax = IMU.getAccelX_mss();
      ay = IMU.getAccelY_mss();
      az = IMU.getAccelZ_mss();
      gx = IMU.getGyroX_rads();
      gy = IMU.getGyroY_rads();
      gz = IMU.getGyroZ_rads();

      directionTowardsLevel = 1;
      EP = 75 * directionTowardsLevel; // Set motor to spin a certain direction
      tempZMeas = az;
      Serial.println(String(calibrationStage) + "\t" + String(az) + "\t" + String(tempZMeas));
      Serial.println("Actuating...");
      ACTUATE_ELEVATOR();
      calibrationStage = 1;
      totalInterrupts1 = 0;
      break;
    case 1: // Find the direction towards the horizontal orientation
      if (totalInterrupts1 > 100)
      {
        IMU.readSensor();

        ax = IMU.getAccelX_mss();
        ay = IMU.getAccelY_mss();
        az = IMU.getAccelZ_mss();
        gx = IMU.getGyroX_rads();
        gy = IMU.getGyroY_rads();
        gz = IMU.getGyroZ_rads();

        if (abs(tempZMeas) >= abs(az))
        {
          directionTowardsLevel *= -1;
          EP = 75 * directionTowardsLevel;
          Serial.println("Switching direction!");
        }

        Serial.println(String(calibrationStage) + "\t" + String(az) + "\t" + String(tempZMeas));
        ACTUATE_ELEVATOR();
        calibrationStage = 2;
        totalInterrupts1 = 0;
      }
      break;
    case 2: // Perform a small step
      EP = 75 * directionTowardsLevel;
      ACTUATE_ELEVATOR();
      calibrationCounter = 0;
      if (totalInterrupts1 > 50)
      {
        STOP_ELEVATOR();
        totalInterrupts1 = 0;
        calibrationStage = 3; // Stop the motor, wait for IMU stabilization, then measure
      }
      break;
    case 3:
      if (totalInterrupts1 > 40)
      {
        if (abs(tempZMeas) > abs(az))
        {
          STOP_ELEVATOR();
          calibrationStage = 4;
          Serial.println("Successfully leveled!");
        }
        else
        {
          tempZMeas = az;
          Serial.println(String(calibrationStage) + "\t" + String(az) + "\t" + String(tempZMeas));
          calibrationStage = 1;
        }
      }
      break;
    case 4: // Calibrate the sensors
      STOP_ELEVATOR();
      EP = 0;
      IMU.calibrateGyro();
      IMU.calibrateAccel();
      calibrated = true;
      calibrating = false;
      break;
    default: // If some error occurs, restart calibration
      calibrationStage = 0;
      break;
  }
}

void HTTP_RECEIVE_MSG()
{
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED)
    {
      PARSE_ORIENTATION(httpGET(orientationChannel));
      previousMillis = currentMillis;
    }
    else
    {
      Serial.println("Disconnected");
    }
  }
}

void HTTP_TRY_CONNECTION()
{
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  if (!httpConnecting)
  {
    WiFi.begin(ssid, password);
    Serial.println("Initiated connection attempt");
    httpConnecting = true;
  }
  else if (currentMillis - previousMillis >= interval * 10) // Every 2.5 seconds, if no connection, restart connection
  {
    WiFi.begin(ssid, password);
    Serial.println("Retrying connection...");
    httpConnecting = true;
    previousMillis = millis();
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("");
    Serial.print("Connected to remote with IP Address: ");
    Serial.println(WiFi.localIP());
    httpConnecting = false;
  }
}

void DETERMINE_MOVEMENT_REQ()
{
  pitch_des = 0;
  yaw_des = 0;

  // For the traverse, we simply set motor command equal to the roll value of the remote...
  if (b_Pitch > 0)
  {
    TP = min(6 * b_Pitch, NOM_PWM_VOLTAGE);
  }
  else if (b_Pitch < 0)
  {
    TP = max(6 * b_Pitch, -NOM_PWM_VOLTAGE);
  }
  else
  {
    TP = LOW;
  }

  if (b_Roll > 0)
  {
    pitch_rate_des = min(b_Roll / 2, 20);
    //EP = -min(b_Roll * 3, NOM_PWM_VOLTAGE);
  }
  else if (b_Roll < 0)
  {
    pitch_rate_des = max(b_Roll / 2, -20)  ;
    //EP = -max(b_Roll * 2, -NOM_PWM_VOLTAGE);
  }
  else
  {
    //EP = LOW;
    pitch_rate_des = 0;
  }

  //  // Make sure that the pitch does not go over a safe limit
  //  if (pitch_des < 30 && pitch_des > -30)
  //  {
  //
  //  }
  //  else if (pitch_des <= -30)
  //  {
  //    pitch_des = -30;
  //  }
  //  else if (pitch_des >= 30)
  //  {
  //    pitch_des = 30;
  //  }
  //
  if (abs(pitch) < 50)
  {
    // Set desired pitch rate
    //pitch_rate_des = 0.75f * (pitch - pitch_des);
    if (b_Roll > 0)
    {
      EP = -7 * (pitch_rate_des - PITCH_RATE()); // We use an asymmetric controller configuration because of the extra torque needed to elevate the phone
    }
    else
    {
      EP = -4 * (pitch_rate_des - PITCH_RATE());
    }
    if (EP > NOM_PWM_VOLTAGE)
    {
      EP = NOM_PWM_VOLTAGE;
    }
    else if (EP < -NOM_PWM_VOLTAGE)
    {
      EP = -NOM_PWM_VOLTAGE;
    }
  }
  else
  {
    EP = 0; // If pitch > 50 then something is clearly wrong with the sensor, since the maximum declination of the device is 45 degrees
  }


  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(pitch_des);
  Serial.print("\t");
  Serial.print(PITCH_RATE());
  Serial.print("\t");
  Serial.print(pitch_rate_des);
  Serial.print("\t");
  Serial.println(EP);
}

//EVENT CHECKERS--------------------------------
bool CONNECTED_HTTP()
{
  return WiFi.status() == WL_CONNECTED;
}

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

// UTILITY FUNCTIONS --------------------------
/*
   This is used for acquiring the data from the http address
*/
String httpGET(const char* httpAddress) {
  WiFiClient client;
  HTTPClient httpClient;

  // Examines the information at given http address
  httpClient.begin(client, httpAddress);

  // Get http message (useful for debugging error messages etc)
  int httpResponseCode = httpClient.GET();

  String payload = "--";

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = httpClient.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
    httpClient.end();
    return "";
  }
  // Free resources
  httpClient.end();

  return payload;
}

/**
   This is used for reading the input from the remote control and converting it into useful data
*/
void PARSE_ORIENTATION(String orientationMessage)
{
  if (orientationMessage != "" && orientationMessage != "--")
  {
    //Using Strings
    int prevInd = 0;

    for (int i = 0; i <= 3; i++) {
      //Serial.println(input.indexOf(',',1));
      prevInd = commaIndex;
      if (i == 3) {
        commaIndex = orientationMessage.length();
      }
      else {
        commaIndex = orientationMessage.indexOf(',', commaIndex + 1);
      }
      if (i != 0) {
        val = orientationMessage.substring(prevInd + 1, commaIndex);
      }
      else {
        val = orientationMessage.substring(prevInd, commaIndex);
      }
      //vars[i] = val.toInt();
      //Serial.println(commaIndex);
      //Serial.println(val);
      switch (i)
      {
        case 0:
          b_State = val.toInt();
          break;
        case 1:
          b_Yaw = val.toFloat();
          break;
        case 2:
          b_Pitch = val.toFloat();
          if (b_Pitch > 180)
          {
            b_Pitch -= 360;
          }
          break;
        case 3:
          b_Roll = val.toFloat();
          if (b_Roll > 180)
          {
            b_Roll -= 360;
          }
          break;
      }
    }


    Serial.print("Received data");
    Serial.print("\t");
    Serial.print(b_State);
    Serial.print("\t");
    Serial.print(b_Yaw);
    Serial.print("\t");
    Serial.print(b_Pitch);
    Serial.print("\t");
    Serial.print(b_Roll);
    Serial.print("\t");
    prevInd = 0;
    commaIndex = 0;
  }
}

/*
   The estimation of rate of change of pitch
   (in degrees)
*/
float PITCH_RATE ()
{
  if (!CALIBRATED())
    return gx * RAD_TO_DEG;
  else
    return gx * RAD_TO_DEG;
}

/*
   The estimated rate of change of yaw, it seems we may need to account for the pitch of the device...
   (in degrees)
*/
float YAW_RATE ()
{
  if (!CALIBRATED())
    return -gz * RAD_TO_DEG;
  else
    return -gz * cos(float(pitch) * DEG_TO_RAD) * RAD_TO_DEG;
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
