#include <SensorFusion.h>
#include <MPU9250.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>

// HTTP broadcaster variables -------------------
const char* ssid = "FS-Beacon";
const char* password = "888888888";
const char* statusChannel = "http://192.168.4.1/status";
const char* positionChannel = "http://192.168.4.1/position";
const char* orientationChannel = "http://192.168.4.1/orientation";

// I/O variables ----------------------------
#define SWTC_1 39  // On-Off switch
#define TMD_1 26  // Traverse motor driver - direction 1
#define TMD_2 25  // Traverse motor driver - direction 2
#define EMD_1 33  // Traverse motor driver - direction 2
#define EMD_2 15  // Traverse motor driver - direction 2
#define LED_13 13  // RED LED
#define LED_12 12  // YELLOW LED

// PLATFORM state variables
float pitch, roll, yaw = 0;

// BEACON state variables
float b_X, b_Y, b_Z = 0;
float b_Pitch, b_Roll, b_Yaw = 0;
int b_State = 0;

const long interval = 1000; 
unsigned long previousMillis = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
    WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // put your main code here, to run repeatedly:
 unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis >= interval) 
  {
     // Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED )
    {
      Serial.println(httpGET(statusChannel)); 
      Serial.println(httpGET(positionChannel)); 
      Serial.println(httpGET(orientationChannel)); 
      previousMillis = currentMillis;
    }
    else 
    {
      Serial.println("WiFi Disconnected");
    }
  }
}

/**
 * Use MPU data to determine the state of this device
  Store data as global variable
 */
void POSITIONAL_UPDATE()
{
  
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
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = httpClient.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  httpClient.end();

  return payload;
}

void DecodeMessage(char httpMessage)
{
  
}
