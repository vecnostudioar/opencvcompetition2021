/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#define Threshold 105 /* Greater the value, more the sensitivity */
#define relay  19 //built in LED 22 lolin

//bool autoOffFlag = 0;
int autoOffCounter = 0;
int autoOffValue = 90; //Segundos.

//RTC_DATA_ATTR int bootCount = 0;
touch_pad_t touchPin;

#include "BluetoothSerial.h"
#include "CThreadsLib.h"

int timeOn = 0;
int timeOff = 0;
int stepDelay = 30;

const int led6 = 12; //Red blinking LED

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;


#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xC4, 0x4F, 0x33, 0x6B, 0xF4, 0x69};
//{0x80, 0x7D, 0x3A, 0xC5, 0x3A, 0x9C};

//esp32 jorge
//C4:4F:33:6B:F4:69

//80:7D:3A:C5:39:48

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  //  char a[32];
  int b;
  int c;
  //  String d;
  //  bool e;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


//*************************************************

int driveOutput() {
  //...
  // Inicio del thread
  CThreadBegin();

  // bucle infinito del thread
  while (1) {

    if (timeOn == 0) {
      digitalWrite(relay, LOW);
      digitalWrite(LED_BUILTIN, HIGH);
      //Serial.println("OFF");
      break;
    }

    if (timeOff == 0) {
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
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
 
  pinMode(relay, OUTPUT);
  digitalWrite(relay,HIGH);
  delay(150);
  digitalWrite(relay,LOW);

  myData.b = -1;

  pinMode(led6, OUTPUT);
  //digitalWrite(led6, LOW);
  digitalWrite(led6, HIGH);
  

   // Init Serial Monitor
  Serial.begin(19200);
  SerialBT.begin("D-Gloves"); //Bluetooth device name
 
  //digitalWrite(relay, LOW); //El built in es active LOW


  //Print the wakeup reason for ESP32 and touchpad too
  print_wakeup_reason();
  print_wakeup_touchpad();


  //Setup interrupt on Touch Pad 3 (GPIO15)
  touchAttachInterrupt(T0, callback, Threshold);

  //Configure Touchpad as wakeup source
  esp_sleep_enable_touchpad_wakeup();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {

  //digitalWrite(relay,HIGH);
  //delay(150);
  //digitalWrite(relay,LOW);

  getBT();
  driveOutput();

      if(myData.b == -1){
      autoOffSleep();}else{
      autoOffSleepBlink();}
 
  turnLight();
  
}


void getBT() {
  while (SerialBT.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    myData.b = SerialBT.parseInt();
    // do it again:
    myData.c = SerialBT.parseInt();

    // look for the newline. That's the end of your sentence:
    if (SerialBT.read() == ';') {
      Serial.print(myData.b);
      Serial.print(",");
      Serial.println(myData.c);
      // Send message via ESP-NOW
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }

      autoOffCounter = 0;
      valueMap();

    } //Cuando termina de entrar la data
  }

}



void valueMap() {

  if (myData.b == 0) {
    timeOn = 0;
    timeOff = 0;
  }

  if (myData.b == 1) {
    timeOn = 120;
    timeOff = 1500;
  }

  if (myData.b == 2) {
    timeOn = 150;
    timeOff = 600;
  }

  if (myData.b == 3) {
    timeOn = 200;
    timeOff = 200;
    
  }

  if (myData.b == 6) {

    for(int k = 0 ; k < 4 ; k++){
      digitalWrite(led6, HIGH);
      delay(150);
      digitalWrite(led6,LOW);
      delay(150);
      }
    esp_deep_sleep_start();
    Serial.println("This will never be printed");

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
    case 0  : Serial.println("Touch on GPIO 4"); break;
    case 1  : Serial.println("Touch on GPIO 33"); break;
    //  case 9  : Serial.println("Touch on GPIO 32"); break;

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
