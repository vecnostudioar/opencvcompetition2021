int turnLight() {
  //...
  // Inicio del thread  
  CThreadBegin();
  
  // bucle infinito del thread
  while(1){

if(myData.b !=3){


//Serial.println("OFF");
break;
}

if(myData.b == 3){
    //getTime(); // No hace falta que lo haga acá si lo hago en el loop.

     // Serial.print("Apagado   "); Serial.print(ciclo); Serial.print(" "); Serial.println(timeOff);
    CThreadSleep(300);
    
    //getTime(); // No hace falta que lo haga acá si lo hago en el loop.

     // Serial.print("Encendido "); Serial.print(ciclo); Serial.print(" "); Serial.println(timeOn);    
    CThreadSleep(600);    
  }
    
  }
  
  // fin del thread
  CThreadEnd();   
} 
