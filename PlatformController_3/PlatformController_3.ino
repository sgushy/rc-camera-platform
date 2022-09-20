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

// I/O variables ----------------------------
#define SWTC_1 39  // On-Off switch
#define EMD_1 26  // Traverse motor driver - direction 1
#define EMD_2 25  // Traverse motor driver - direction 2
#define TMD_1 33  // Elevator motor driver - direction 2
#define TMD_2 15  // Elevator motor driver - direction 2
#define LED_13 13  // YELLOW LED
#define LED_12 12  // RED LED

SF fusion;
// PLATFORM state variables ---------------------------------
float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
int pitch, roll, yaw;
float deltat;
int pitch_des, yaw_des; // 2 axis camera platform can have pitch and yaw be controlled
int pitch_rate_des, yaw_rate_des; // 2 axis camera platform can have pitch and yaw be controlled

int state = 0;
TaskHandle_t Calibration;

// Set up accelerometer sensor
MPU9250 IMU(Wire, 0x68);
int status;

// IMU cclibration variables ------------------------
volatile bool calibrated = false; // Calibration completed?
volatile bool calibrating = false; // Calibration in process?
int calibrationStage = 0; // 0 = find z-offset, 1 = level z, 2 = find x,y offset, 3 = calibrate gyros, 4 = calibrate mag sensor
int calibrationCounter = 0; // Tracks number of times calibration measurements have been taken

// Platform calibration variables
short directionTowardsLevel = 1; // Indicates the direction of elevation offset from a level surface; -1 for below horizon and 1 for above
float tempZMeas = 00000.0f;
float maxGravZ = 0.0f; // The maximum value of gravity detected along z-axis; this is the position where the platform is fully level
const float g = 9.81f; // The gravitational acceleration on Earth (we will subtract this from maxGravZ to find the accelerometer z-offset, which we then use to auto-level the platform)


// BEACON state variables ---------------------------
float b_X, b_Y, b_Z = 0;
int b_Pitch, b_Roll, b_Yaw = 0;
int b_State = 0;

int commaIndex = 0; // For parsing the HTTP message
String val = "";

const long interval = 125; // Will check for updates every 1/8 second
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

//

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
    state = -1;
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
    case 0: // Switch is off
      digitalWrite(LED_12, LOW);
      digitalWrite(LED_13, LOW);
      STOP_ELEVATOR();
      STOP_TRAVERSE();
      if (SWITCH_ON())
      {
        state = 1;
      }
      break;
    case 1: // Calibrate our platform
      digitalWrite(LED_12, HIGH);
      digitalWrite(LED_13, LOW);

      if (!CALIBRATING())
      {
        xTaskCreatePinnedToCore(Calibration_Coroutine, "Calibration", 10000, NULL, 1, &Calibration, 0); // Start calibration on core 0 of processor
      }


      if (CALIBRATED() && SWITCH_ON())
      {
        STOP_ELEVATOR();
        state = 2;
      }

      if (!SWITCH_ON())
      {
        state = 0;
        digitalWrite(LED_12, LOW);
        digitalWrite(LED_13, LOW);
        STOP_ELEVATOR();
        STOP_TRAVERSE();
        ESP.restart(); // Restart the micro controller to clear any variables in RAM
      }
      break;
    case 2: // Try to connect to HTTP channel
      HTTP_TRY_CONNECTION();

      BLINK_LED_1S(LED_13);
      digitalWrite(LED_12, HIGH);
      if (!SWITCH_ON())
      {
        state = 0;
        digitalWrite(LED_12, LOW);
        digitalWrite(LED_13, LOW);
        STOP_ELEVATOR();
        STOP_TRAVERSE();
        ESP.restart(); // Restart the micro controller to clear any variables in RAM
      }
      else if (CONNECTED_HTTP())
      {
        state = 3;
      }
      break;

    case 3:
      // Receiving data
      digitalWrite(LED_12, HIGH);
      digitalWrite(LED_13, HIGH);

      HTTP_RECEIVE_MSG(); // Receive data from the remote

      POSITIONAL_UPDATE(); // Update own position

      DETERMINE_MOVEMENT_REQ(); // Calculate required movement characteristics

      ACTUATE_TRAVERSE(); // Actuate traverse
      ACTUATE_ELEVATOR(); // Actuate elevator

      if (!CONNECTED_HTTP())
      {
        state = 1;
      }
      if (!SWITCH_ON())
      {
        state = 0;
        digitalWrite(LED_12, LOW);
        digitalWrite(LED_13, LOW);
        STOP_ELEVATOR();
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
        STOP_ELEVATOR();
        STOP_TRAVERSE();
        ESP.restart(); // Restart the micro controller to clear any variables in RAM
      }
      break;
  }
}

// Run IMU calibration on second core, to prevent interruptions
void Calibration_Coroutine( void * pvParameters ) {
  Serial.print("Calibration of IMU running on core ");
  Serial.println(xPortGetCoreID());

  while (!CALIBRATED())
  {
    if (!SWITCH_ON())
    {
      delay(50);
      break; // Stop calibration if switch turned off
    }
    POSITIONAL_CALIBRATE();
  }
  delay(5);
  Serial.println("Calibration finished");
  for (;;) {}
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
  else
  {
    ledcWrite(ledChannel_3, abs(EP));
    ledcWrite(ledChannel_4, LOW);
  }
}

void ACTUATE_TRAVERSE() {
  if (TP > 0)
  {
    ledcWrite(ledChannel_1, LOW);
    ledcWrite(ledChannel_2, abs(TP));
  }
  else
  {
    ledcWrite(ledChannel_1, abs(TP));
    ledcWrite(ledChannel_2, LOW);
  }
}

/**
   Use MPU data to determine the state of this device
  Store data as global variable
*/
void POSITIONAL_UPDATE()
{
  IMU.readSensor();

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

  deltat = fusion.deltatUpdate();
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //Estimate the quaternion orientation of the platform

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
  if (!CALIBRATING())
  {
    calibrating = true;
    Serial.println("Calibration started...");
    bool leveled = false;
    int index = 0;
    while (index < 50)
    {
      index++;
      if (!SWITCH_ON())
      {
        return;
      }

      delay(10);

      IMU.readSensor();
      float aaz = IMU.getAccelZ_mss();

      EP = 88 * directionTowardsLevel;

      ACTUATE_ELEVATOR();

      if (abs(aaz) > abs(tempZMeas))
      {
        tempZMeas = aaz;
      }

      else if (abs(aaz) == abs(tempZMeas))
      {
        calibrationCounter++;
      }
      if (calibrationCounter > 5)
      {
        directionTowardsLevel *= -1;
      }

      Serial.print("\t");
      Serial.print(aaz);
    }
    delay(25);
    IMU.calibrateGyro();
    IMU.calibrateAccel();
    calibrated = true;
    calibrating = false;
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
      //Serial.println(httpGET(statusChannel));
      //Serial.println(httpGET(positionChannel));
      //Serial.println(httpGET(orientationChannel));

      //PARSE_STATE(httpGET(statusChannel));
      PARSE_ORIENTATION(httpGET(orientationChannel));
      //PARSE_POSITION(httpGET(positionChannel));

      previousMillis = currentMillis;
    }
    else
    {
      Serial.println("WiFi Disconnected");
    }
  }
}

void HTTP_TRY_CONNECTION()
{
  if (!httpConnecting)
  {
    WiFi.begin(ssid, password);
    Serial.println("Initiated connection attempt");
    httpConnecting = true;
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address: ");
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
    TP = min(6 * b_Pitch, MAX_PWM_VOLTAGE);
  }
  else if (b_Pitch < 0)
  {
    TP = max(6 * b_Pitch, -MAX_PWM_VOLTAGE);
  }
  else
  {
    TP = LOW;
  }

  if (b_Roll > 0)
  {
    pitch_des = min(b_Roll, 25);
  }
  else if (b_Roll < 0)
  {
    pitch_des = max(b_Roll, -25)  ;
  }
  else
  {
    pitch_des = 0;
  }
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
    state = 1;
  }
  // Free resources
  httpClient.end();

  return payload;
}

//void PARSE_STATE(String stateMessage)
//{
//
//}

void PARSE_ORIENTATION(String orientationMessage)
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

float PITCH_RATE ()
{
  if (!CALIBRATED())
    return -gx;
  else
    return -gx;
}

float YAW_RATE ()
{
  if (!CALIBRATED())
    return -gz;
  else
    return -gz * cos(float(pitch) * DEG_TO_RAD);
}
//void PARSE_POSITION(String positionMessage)
//{
//
//}

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
