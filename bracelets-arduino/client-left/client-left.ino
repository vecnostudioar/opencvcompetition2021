/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#define Threshold 105  // Greater the value, more the sensitivity -- 100 Ok for battery operation (Ale)
#define relay  19 //built in LED 22 lolin

//bool autoOffFlag = 0;
int autoOffCounter = 0;
int autoOffValue = 20; //Segundos.

/*
34 y 35 tienen * qué será??
 */

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
//    char a[32];
    int b;
    int c;
//    String d;
//    bool e;
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
  /*    
      if(sync == 0){
      sync = 1;
      autoOffSleep();
      }
  */
  valueMap();

}


//*************************************************
//#define relayExtra  16 //5
//#define relay  22 //5

int driveOutput() {
  //...
  // Inicio del thread  
  CThreadBegin();

  // bucle infinito del thread
  while(1){

      if(timeOn == 0){
        digitalWrite(relay, LOW);
        digitalWrite(LED_BUILTIN, HIGH);
        //Serial.println("OFF");
        break;
      }
      
      if(timeOff == 0){          // Me parece que en este caso no tiene sentido usar timeOff = 0
        digitalWrite(relay, LOW);
        digitalWrite(LED_BUILTIN, HIGH);
        //Serial.println("ON");
        break;
      }
  
      //getTime(); // No hace falta que lo haga acá si lo hago en el loop.
      digitalWrite(relay, LOW);
      digitalWrite(LED_BUILTIN, HIGH);
   // Serial.print("Apagado   "); Serial.print(ciclo); Serial.print(" "); Serial.println(timeOff);
      CThreadSleep(timeOff);
      
      //getTime(); // No hace falta que lo haga acá si lo hago en el loop.
      digitalWrite(relay, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
   // Serial.print("Encendido "); Serial.print(ciclo); Serial.print(" "); Serial.println(timeOn);    
      CThreadSleep(timeOn);    

    
  }
  
  // fin del thread
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
    /*
        case 1  : Serial.println("Touch detected on GPIO 0"); break;
        case 2  : Serial.println("Touch detected on GPIO 2"); break;
        case 3  : Serial.println("Touch detected on GPIO 15"); break;
        case 4  : Serial.println("Touch detected on GPIO 13"); break;
        case 5  : Serial.println("Touch detected on GPIO 12"); break;
        case 6  : Serial.println("Touch detected on GPIO 14"); break;
        case 7  : Serial.println("Touch detected on GPIO 27"); break;
    */
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
