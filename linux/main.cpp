/**
 * g++ llv3.cpp lidarlite_v3.cpp -I . -o llv3.out
 * @file       main.cpp
 * @authors    John Arena, Shamim Babul, Rona Kosumi, Ionut Rotariu
 * @license    This project is NOT released under any license.
 * @date       Sept 2019-May 2020
 */

//#define BLYNK_DEBUG
#define BLYNK_PRINT stdout
#ifdef RASPBERRY
  #include <BlynkApiWiringPi.h>
#else
  #include <BlynkApiLinux.h>
#endif
#include <BlynkSocket.h>
#include <BlynkOptionsParser.h>


static BlynkTransportSocket _blynkTransport;
BlynkSocket Blynk(_blynkTransport);

static const char *auth, *serv;
static uint16_t port;

#include <BlynkWidgets.h>

/* OUR FUNCTIONS, VARIABLES, ETC BELOW*/

#ifndef OUR_FUNCTION_HEADERS_
#define OUR_FUNCTION_HEADERS_
void readSpeedometerSignal();
void wheelRevolutionFunction();
void speedometerReadingCalculation(double totalTime);
void turnOnLeftTurnSignal();
void turnOnRightTurnSignal();
void UpdateLidar();
#endif

WidgetLED led1(V16);
WidgetLED led2(V19);

#include <time.h> /* Will be used for MPH */
#include <stdio.h> 
#include <cmath>
#include <iomanip>      
#include <linux/types.h>
#include <cstdio>
#include <lidarlite_v3.h>
#include <iostream>
#include <fstream>
#include <chrono>	
#include <unistd.h>
#include <wiringPi.h>

using namespace std;

int wheelSensorGoLowCounter = 1;
double timeDifferenceSeconds = 0.0, milesPerHour = 0.0;
double totalTime;
std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_1, currentTime_2;
std::chrono::duration<double> totalDuration;

int gpioSpeedometer = 12;
int gpioRightTurnSignal = 16;
int gpioLeftTurnSignal = 19;

LIDARLite_v3 myLidarLite;
BlynkTimer tmr;

BLYNK_WRITE(V1)
{
    printf("Got a value: %s\n", param[0].asStr());
}

void setup()
{
    Blynk.begin(auth, serv, port);
    tmr.setInterval(1000, [](){
      Blynk.virtualWrite(V0, BlynkMillis()/1000);
    });
	
	pinMode(12, INPUT); // GPIO 12, pin 32
	pinMode(16, INPUT); // GPIO 16, pin 36
	pinMode(19, INPUT); // GPIO 19, pin 35
	//tmr.setInterval(50L,readSpeedometerSignal); // Call every .05 seconds

	//GPIO.add_event_detect(12, GPIO_FALLING, bouncetime=930); // Testing interrupt
	//if (GPIO.event_detected(12)) {
	//	readSpeedometerSignal();
	//}

	//attachInterrupt(digitalPinToInterrupt(12), readSpeedometerSignal, FALLING);
	wiringPiISR(12, INT_EDGE_FALLING, &readSpeedometerSignal);
	wiringPiISR(16, INT_EDGE_BOTH, &turnOnRightTurnSignal);
	wiringPiISR(19, INT_EDGE_BOTH, &turnOnLeftTurnSignal);

	myLidarLite.i2c_init();     // Initialize i2c peripheral in the cpu core
    myLidarLite.configure(0);    // Optionally configure LIDAR-Lite

}

void loop()
{
    Blynk.run();
    tmr.run();
    //UpdateLidar();
}

/* DECLARE GLOBAL VARIABLES, LIBRARIES AND PIN MODES ABOVE HERE. WRITE FUNCTIONS BELOW */
int count = 0;
void readSpeedometerSignal(){
  if(digitalRead(gpioSpeedometer) == 0){ // Active Low Hall Sensor
	  wheelRevolutionFunction();
	  //std::cout << "GPIO PIN is LOW - count: " << count << std::endl;
	  //count++;
	  //delay(928);
    }
}

void wheelRevolutionFunction(){
  if(wheelSensorGoLowCounter == 1){
	currentTime_1 = std::chrono::high_resolution_clock::now();
	std::cout << "wheelSensorGoLow:" << wheelSensorGoLowCounter << std::endl;
	wheelSensorGoLowCounter++;
  }
  else if (wheelSensorGoLowCounter > 1 && wheelSensorGoLowCounter < 10) {
	  currentTime_2 = std::chrono::high_resolution_clock::now();
	  totalDuration = currentTime_2 - currentTime_1;
	  totalTime = std::chrono::duration<double>(totalDuration).count();
	  cout << "debounce timing : " << totalTime << endl;
	  if (totalTime > .0927) { // Debouncing protections (92.7ms)
		  std::cout << "wheelSensorGoLow:" << wheelSensorGoLowCounter << std::endl;
		  wheelSensorGoLowCounter++;
	  }
	  else {
		  std::cout << "Debounce error detected, not counting" << endl;
	  }
  }
  else if(wheelSensorGoLowCounter == 10){
	currentTime_2 = std::chrono::high_resolution_clock::now();
	totalDuration = currentTime_2 - currentTime_1;
	totalTime = std::chrono::duration<double>(totalDuration).count();
	std::cout << "wheelSensorGoLow:" << wheelSensorGoLowCounter << std::endl;
	std::cout << "timeDifferenceSeconds: " << totalTime << std::endl;
    speedometerReadingCalculation(totalTime);
	std::cout << "MPH:" << milesPerHour << std::endl;
    Blynk.virtualWrite(V12,milesPerHour);
	wheelSensorGoLowCounter = 1;
  }
  else{}
}

void speedometerReadingCalculation(double totalTime){
	milesPerHour = (5* 2 * M_PI*(1.083) * 60 * 60) / (5280 * totalTime);
}

void turnOnRightTurnSignal() {
	if (digitalRead(gpioRightTurnSignal) == 0) {
		led1.off();
	}
	else {
		led1.on();
	}
}

void turnOnLeftTurnSignal() {
	if (digitalRead(gpioLeftTurnSignal) == 0) {
		led2.off();
	}
	else {
		led2.on();
	}
}

void UpdateLidar()
{
	__u16 distance;
	__u8  busyFlag;
	busyFlag = myLidarLite.getBusyFlag();

	if (busyFlag == 0x00)
	{
		myLidarLite.takeRange();
		distance = myLidarLite.readDistance();
		cout << "Distance: " << distance << " cm" << endl;
		Blynk.virtualWrite(V20, (int)distance);
	}
}

int main(int argc, char* argv[])
{
    parse_options(argc, argv, auth, serv, port);

    setup();
    while(true) {
        loop();
    }

    return 0;
}

