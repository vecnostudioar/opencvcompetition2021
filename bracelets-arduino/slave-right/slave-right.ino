/*
  Mentions of documentations used: https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
*/

#define Threshold 105  // Greater the value, more the sensitivity -- 100 Ok for battery operation 
#define relay  19 

//bool autoOffFlag = 0;
int autoOffCounter = 0;
int autoOffValue = 20; //Segundos.


int stepDelay = 30;
bool sync = 0;

const int led6 = 12; //Red blinking LED
 
//RTC_DATA_ATTR int bootCount = 0;
touch_pad_t touchPin;


#include <esp_now.h>
#include <WiFi.h>

#include "CThreadsLib.h"

int timeOn = 0;
int timeOff = 0;


// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int b;
    int c;
} struct_message;

// Create a struct_message called myData
struct_message myData;

  // callback function that will be executed when data is received
  void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.print(myData.b);
    Serial.print(" ,");
    Serial.println(myData.c);
  
  autoOffCounter = 0;
  
  valueMap();

}


//*************************************************
//#define relayExtra  16 //5
//#define relay  22 //5

int driveOutput() {
  //...
  // start thread  
  CThreadBegin();

  while(1){

      if(timeOn == 0){
        digitalWrite(relay, LOW);
        digitalWrite(LED_BUILTIN, HIGH);
        //Serial.println("OFF");
        break;
      }
      
      if(timeOff == 0){
        digitalWrite(relay, LOW);
        digitalWrite(LED_BUILTIN, HIGH);
        //Serial.println("ON");
        break;
      }
  

      digitalWrite(relay, LOW);
      digitalWrite(LED_BUILTIN, HIGH);

      CThreadSleep(timeOff);
      
      digitalWrite(relay, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
 
      CThreadSleep(timeOn);    

    
  }
  
  // end of thread
  CThreadEnd();   
} 

//*************************************************


 
void setup() {
  // Initialize Serial Monitor

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
    
  pinMode(relay, OUTPUT);
  digitalWrite(relay,HIGH);
  delay(150);
  digitalWrite(relay,LOW);

  myData.c = -1;

  
  pinMode(led6, OUTPUT);

  
  //digitalWrite(led6, LOW);
  digitalWrite(led6, HIGH);
  
  Serial.begin(19200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  print_wakeup_reason();
  print_wakeup_touchpad();

  //Setup interrupt on Touch Pad 3 (GPIO15)
  touchAttachInterrupt(T0, callback, Threshold);

  //Configure Touchpad as wakeup source
  esp_sleep_enable_touchpad_wakeup();

  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  driveOutput();

  if(myData.c == -1){
  autoOffSleep();}else{
  autoOffSleepBlink();}

  sleepByApp();
  turnLight();
}


void valueMap(){


  if (myData.c == 0){
     timeOn = 0;
     timeOff = 0; 
     }

  if (myData.c == 1){
     timeOn = 120;
     timeOff = 1500; 
     }

   if (myData.c == 2){
     timeOn = 150;
     timeOff = 600; 
     }

   if (myData.c == 3){
     timeOn = 200;
     timeOff = 200; 
     }

}





/*
  Method to print the reason by which ESP32
  has been awaken from sleep
*/
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

/*
  Method to print the touchpad by which ESP32
  has been awaken from sleep
*/
void print_wakeup_touchpad() {
  touchPin = esp_sleep_get_touchpad_wakeup_status();

  switch (touchPin)
  {
    case 0  : Serial.println("Touch detected on GPIO 4"); break;
    case 1  : Serial.println("Touch detected on GPIO 33"); break;
    //  case 9  : Serial.println("Touch detected on GPIO 32"); break;

    default : Serial.println("Wakeup not by touchpad"); break;
  }

}

void callback() {

  /*//placeholder callback function
   digitalWrite(relay, HIGH);
   delay(200);
   digitalWrite(relay, LOW);
*/
 
}
