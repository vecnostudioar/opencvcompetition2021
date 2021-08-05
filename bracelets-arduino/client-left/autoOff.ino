
// int autoOffcounter = 0;
// int autoOffValue = 10; //Segundos.



int autoOffSleep() {
  //...
  // Inicio del thread
  CThreadBegin();

  // bucle infinito del thread
  while (1) {

    //digitalWrite(led6, HIGH);
    CThreadSleep(200);
    //digitalWrite(led6, LOW);
    CThreadSleep(800); //200+800=1000mS
    autoOffCounter++;
    Serial.println(autoOffCounter);
    

      if (autoOffCounter == autoOffValue){
        digitalWrite(led6, LOW);
        esp_deep_sleep_start();
      }

     }
  
  // fin del thread
  CThreadEnd();
}


int autoOffSleepBlink() {
  //...
  // Inicio del thread
  CThreadBegin();

  // bucle infinito del thread
  while (1) {

    digitalWrite(led6, HIGH);
    CThreadSleep(200);
    digitalWrite(led6, LOW);
    CThreadSleep(800); //200+800=1000mS
    autoOffCounter++;
    Serial.println(autoOffCounter);
    

    if (autoOffCounter == autoOffValue)
    esp_deep_sleep_start();

     }
  
  // fin del thread
  CThreadEnd();
}
