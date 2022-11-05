#include <ESP8266WiFi.h>
#include <MAX3010x.h>
#include "filters.h"
#include"Adafruit_MQTT.h"
#include"Adafruit_MQTT_Client.h"
#define WLAN_SSID "vss"
#define WLAN_PASS "12345678"

#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME "vikramadityasinghs"
#define AIO_KEY "aio_FrPn049dLkDJIWBxEx1gE025mptm"

MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);
Adafruit_MQTT_Publish mtemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/mtemp");
Adafruit_MQTT_Publish fpulse = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/fpulse");
Adafruit_MQTT_Publish mpulse = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/mpulse");

int outputpin= A0;
int fepulse = 50;
int sound_digital = 15;

void setup() {  
  
  Serial.begin(115200);
    pinMode(sound_digital, INPUT); 
  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) { 
    Serial.println("Sensor initialized");
  }
  else {
    Serial.println("Sensor not found");  
    while(1);
  }
  Serial.println(F("Adafruit IO Example")); 
  // Connect to WiFi access 
  Serial.print(F("Connecting to ")); 
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS); 
  while (WiFi.status() != WL_CONNECTED)
  {    delay(500); 
  Serial.print(F("."));   
  } 
  Serial.println(); 
  Serial.println(F("WiFi connected")); 
  Serial.println(F("IP address: ")); 
  Serial.println(WiFi.localIP());  
  // connect to adafruit io  
  connect();

  }// connect to adafruit io via MQTT
  void connect() { 
    Serial.print(F("Connecting to Adafruit IO... ")); 
    int8_t ret; 
    while ((ret = mqtt.connect()) != 0) {  
      switch (ret) {   
        case 1: Serial.println(F("Wrong protocol")); break;      
        case 2: Serial.println(F("ID rejected")); break;     
        case 3: Serial.println(F("Server unavail")); break;   
        case 4: Serial.println(F("Bad user/pass")); break;    
        case 5: Serial.println(F("Not authed")); break;      
        case 6: Serial.println(F("Failed to subscribe")); break; 
        default: Serial.println(F("Connection failed")); break;   
        }    
        if(ret >= 0)  
        mqtt.disconnect(); 
        Serial.println(F("Retrying connection..."));  
        delay(10000); 
        } 
        Serial.println(F("Adafruit IO Connected!"));}
        

LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// R value to SpO2 calibration factors
// See https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6845.html
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;


void loop() {
  // put your main code here, to run repeatedly:
if(! mqtt.ping(3)){
  if(! mqtt.connected())
  connect();}
 int bpm;
 int val_digital = digitalRead(sound_digital);
int analogValue = analogRead(outputpin);
float millivolts = (analogValue/1024.0) * 3300; //3300 is the voltage provided by NodeMCU
float celsius = millivolts/10;
float metemp =((celsius * 9)/5 + 32);
Serial.print("in DegreeF=   ");
Serial.println(metemp);
auto sample = sensor.readSample(1000);
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;

   if (val_digital == HIGH)
  {
    fepulse = 1;
    }
   else{
    fepulse=0;
    }
  // Detect Finger using raw sensor value
  if(sample.red > kFingerThreshold) {
    if(millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  }
  else {
    // Reset values if the finger is removed
    differentiator.reset();
    averager_bpm.reset();
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();
    
    finger_detected = false;
    finger_timestamp = millis();
  }

  if(finger_detected) {
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);

    // Statistics for pulse oximetry
    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);

    // Heart beat detection using value for red LED
    float current_value = high_pass_filter.process(current_value_red);
    float current_diff = differentiator.process(current_value);

    // Valid values?
    if(!isnan(current_diff) && !isnan(last_diff)) {
      
      // Detect Heartbeat - Zero-Crossing
      if(last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }
      
      if(current_diff > 0) {
        crossed = false;
      }
  
      // Detect Heartbeat - Falling Edge Threshold
      if(crossed && current_diff < kEdgeThreshold) {
        if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          // Show Results
          bpm = 60000/(crossed_time - last_heartbeat);
          float rred = (stat_red.maximum()-stat_red.minimum())/stat_red.average();
          float rir = (stat_ir.maximum()-stat_ir.minimum())/stat_ir.average();
          float r = rred/rir;
          float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
          
          if(bpm > 50 && bpm < 250) {
            // Average?
            if(kEnableAveraging) {
              int average_bpm = averager_bpm.process(bpm);
              int average_r = averager_r.process(r);
              int average_spo2 = averager_spo2.process(spo2);
  
              // Show if enough samples have been collected
              if(averager_bpm.count() >= kSampleThreshold) {
                Serial.print("Time (ms): ");
                Serial.println(millis()); 
                Serial.print("Heart Rate (avg, bpm): ");
                Serial.println(average_bpm);
                Serial.print("R-Value (avg): ");
                Serial.println(average_r);  
                Serial.print("SpO2 (avg, %): ");
                Serial.println(average_spo2);  
              }
            }
            else {
              Serial.print("Time (ms): ");
              Serial.println(millis()); 
              Serial.print("Heart Rate (current, bpm): ");
              Serial.println(bpm);  
              Serial.print("R-Value (current): ");
              Serial.println(r);
              Serial.print("SpO2 (current, %): ");
              Serial.println(spo2);   
            }
          }

          // Reset statistic
          stat_red.reset();
          stat_ir.reset();
        }
  
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }

    last_diff = current_diff;
    Serial.println(bpm);
  }
    
delay(10000);

if(! mtemp.publish(metemp)){
  Serial.println("FAILED!");}
  else{
  Serial.println("PASSED!");}
  
  if(!fpulse.publish(fepulse))
  {
    Serial.println("FAILED!");
  }
 else{
  Serial.println("PASSED!");}
  
  if(!mpulse.publish(bpm))
  {
    Serial.println("FAILED!");
  }
 else{
  
  Serial.println("PASSED!");}

  
  
}
