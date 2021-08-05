void sleepByApp(){
if (myData.c == 6) {

      timeOn = 0;
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
