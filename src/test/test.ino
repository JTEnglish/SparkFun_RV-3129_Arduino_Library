/*
  Test Arduino sketch for RV3129 RTC module.
*/

#include "SparkFun_RV3129.h"

RV3129 rtc;

int hund = 0;
int sec = 2;
int minute = 18;
int hour = 21;
int date = 25;
int month = 5;
int year = 14;
int day = 5;

unsigned long startTime;

void setup() {

  Wire.begin();

  Serial.begin(9600);
  Serial.println("Read Time from RTC Example");

  if (rtc.begin() == false) {
    Serial.println("Something went wrong, check wiring");
  }

  rtc.set24Hour();

  //Use the time from the Arduino compiler (build time) to set the RTC
  //Keep in mind that Arduino does not get the new compiler time every time it compiles. to ensure the proper time is loaded, open up a fresh version of the IDE and load the sketch.
  if (rtc.setToCompilerTime() == false) {
    Serial.println("Something went wrong setting the time");
  }
  
  //Uncomment the below code to set the RTC to your own time
  if (rtc.setTime(sec, minute, hour, date, month, year, day) == false) {
    Serial.println("Something went wrong setting the time");
  }
  
  Serial.println("RTC online!");
}

void loop() {
  if (rtc.updateTime() == false) { //Updates the time variables from RTC
    Serial.println("RTC failed to update");
  }
  else {
    String currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format
    String currentTime = rtc.stringTime(); //Get the time
    
    Serial.print(currentDate);
    Serial.print(" ");
    Serial.println(currentTime);
  }

  delay(1000);
}