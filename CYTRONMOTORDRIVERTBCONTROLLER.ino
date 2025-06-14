/*
Program: TB controller program
Motor Driver: Cytron 13Amp 6V-30V DC Motor Driver
Date: 07/05/2025
Author: Kaden Van Domselaar
Wiring: Refer to pin definitions
*/

#include "CytronMotorDriver.h"

// Pin definitions
#define PEDAL_PIN A0  //currently POT input (full size signal)
#define PEDAL_PIN2 A1  //Half size signal pedal sensor
#define TPS_PIN A2 // TPS1 - Blue grey 
#define TPS_PIN2 A3 // TPS2 - Yellow white
#define PWM 3
#define DIR 4
#define Setsim 13 //simulate WOT
CytronMD motor(PWM_DIR, 3, 4); //define PWM and DIR pins to driver


struct {
  float Kp, Ki, Kd, Kff; //PID coefficents
  float integralofe, derivativeofTPS, derivativeofe; //Integral and derivative action stuff
  unsigned long PreviousTime, CurrentTime; //Used for Integral Action
  float deltaT; 
  float PreviousSetpoint, DeltaSetpoint; //Used for Kff
  float PreviousTPS, Previouserror; //Used for Kd
  float Tmin;
  float Kpe, Kie, Kde, Kffe; //PID control signal parts
  float TPSCutOffHz;
} PID;

struct {
  float Pedal1Raw, Pedal2Raw, TPS1Raw, TPS2Raw; //Raw Readings from sensors
  float TPS1Filtered;
  float TruePedal, TrueTPS; //Outputs of Filters - Used to show filter action on Raw signal
  float Pedal1Min, Pedal1Max, Pedal2Min, Pedal2Max, TPS1Min, TPS1Max, TPS2Min, TPS2Max; //Mina and max analog values for all 4 sensors
  float IDLEPERCENT; //percent of opening for idle
  float prevTPS1Filtered;
}IN;


int dutycyclelow = -255, dutycyclehigh = 255;

float u = 0; //control signal
float e = 0; //error

void setup() {
  TCCR2B = TCCR2B & 0b11111000 | 0x01; //PWM to 32kHz
  pinMode(PEDAL_PIN, INPUT); //
  pinMode(PEDAL_PIN2, INPUT); //

  pinMode(TPS_PIN, INPUT); //
  pinMode(TPS_PIN2, INPUT); //

  pinMode(PWM, OUTPUT); // 0-255
  pinMode(DIR, OUTPUT); // High or Low
  Serial.begin(38400);

  // PID gains
  PID.Kp = 10; // 1:1 ratio (1 percent error = 1 unit of signal) //8.5 for 0.1uF and 10Kohm //10 for nothing

  PID.Ki = 1; // 1:1 ratio (1 percent error = 1 unit of signal)

  PID.Kd = 90; // 1000 = 1:1 ratio //80

  PID.Kff = 0; // 1000 = 1:1 ratio NOT BEING USED
  
  PID.Tmin = 25; //35

  PID.PreviousTime = 0; 
  PID.deltaT = 0;
  PID.integralofe = 0;
  PID.derivativeofTPS = 0;
  PID.derivativeofe = 0;
  PID.Previouserror = 0;
  PID.PreviousSetpoint = 0;
  PID.PreviousTPS = 0;
  PID.DeltaSetpoint = 0;
  PID.CurrentTime = 0;
  PID.Kpe = 0;
  PID.Kie = 0;
  PID.Kde = 0;
  PID.Kffe = 0;
  PID.TPSCutOffHz = 10; //for low pass filter
 
  IN.IDLEPERCENT = 5;
  IN.TruePedal = 0;
  IN.TrueTPS = 0;
  IN.TPS1Filtered = 0;
  IN.prevTPS1Filtered = 0;

  IN.Pedal1Min = 148;
  IN.Pedal1Max = 534;

  IN.Pedal2Min = 0; //Not being used right now
  IN.Pedal2Max = 0;

  IN.TPS1Min = 790; //790
  IN.TPS1Max = 360; //360

  IN.TPS2Min = 0; //not being used right now
  IN.TPS2Max = 0;

}


void loop() {

  //Calculate Delta Time
  PID.CurrentTime = micros();
  unsigned long deltaMicros = PID.CurrentTime - PID.PreviousTime;
  PID.deltaT = fmax(deltaMicros / 1000000.0, 0.0001);
  PID.PreviousTime = PID.CurrentTime;

  //Scan Pedal Signals
  //IN.Pedal1Raw = map(analogRead(PEDAL_PIN),IN.Pedal1Min,IN.Pedal1Max,0,100); //Pedal one
  IN.Pedal1Raw = map(analogRead(PEDAL_PIN),0,1020,IN.IDLEPERCENT,100); //Reading Pot for testing
  IN.Pedal1Raw = min(IN.Pedal1Raw,100);
  IN.Pedal1Raw = max(IN.Pedal1Raw,0);

  IN.Pedal2Raw = map(analogRead(PEDAL_PIN2),235,500,IN.IDLEPERCENT,100); //Read pedal two
  IN.Pedal2Raw = min(IN.Pedal2Raw,100);
  IN.Pedal2Raw = max(IN.Pedal2Raw,0);

  IN.TruePedal = IN.Pedal1Raw;



  IN.TPS1Raw = map(analogRead(TPS_PIN), IN.TPS1Min, IN.TPS1Max, 0, 100); //read TPS one
  //IN.TPS1Raw = max(IN.TPS1Raw,3);
  IN.TPS2Raw = map(analogRead(TPS_PIN2), 777, 987, 0, 100); //read TPS two

  //Filtering TPS signal
  /*
  float alpha = (2.0 * PI * PID.TPSCutOffHz * PID.deltaT) / (2.0 * PI * PID.TPSCutOffHz * PID.deltaT + 1.0);
  IN.TPS1Filtered = (1.0 - alpha) * IN.prevTPS1Filtered + alpha * IN.TPS1Raw;
  IN.prevTPS1Filtered = IN.TPS1Filtered; 

  IN.TrueTPS = IN.TPS1Filtered;
  */
  
  IN.TrueTPS = IN.TPS1Raw;
 




  if(digitalRead(13)) {
     IN.TruePedal = 90;  
  } //force WOT (for testing PID loop)



  //ERROR CALC
  e = IN.TruePedal - IN.TrueTPS;


  //Kp error calculaton
  PID.Kpe = PID.Kp * e;


  //Ki error calculation
  if(abs(e) < 5)
  {
    PID.integralofe += e * PID.deltaT; //If we are within 5 percent error then start integrating the error
  }
  else
  {
    PID.integralofe = 0;
  }


  PID.Kie = PID.Ki * PID.integralofe;


  //Kd error calculation
  //Slope of error derivative
  PID.derivativeofe = (e - PID.Previouserror)/PID.deltaT;
  PID.Previouserror = e;
  PID.Kde = PID.Kd * PID.derivativeofe / 1000;
  
  
  //Slope of TPS derivative
  /*
  PID.derivativeofTPS = (PID.PreviousTPS - IN.TrueTPS)/PID.deltaT;
  PID.PreviousTPS = IN.TrueTPS;
  PID.Kde =- PID.Kd * PID.derivativeofTPS / 1000;
  */

  //Kf calculations NOT BEING USED
  /*
  if(e>0) {PID.Kff = PID.KffPos;} 
   else {PID.Kff = PID.KffNeg;}
  PID.DeltaSetpoint = (IN.TruePedal - PID.PreviousSetpoint);
  PID.PreviousSetpoint = IN.TruePedal;
  PID.Kffe = PID.DeltaSetpoint * PID.Kff / 1000;
  */

  //PID summing
  u = PID.Kpe + PID.Kie + PID.Kde + PID.Kffe + PID.Tmin;



  //Cap PWM signal
  u = constrain(u, dutycyclelow, dutycyclehigh);

  //Drive Motor
  motor.setSpeed(u);


  //Tuning stuff for the graph
  Serial.print(PID.Kpe);
  Serial.print(" ");

  Serial.print(PID.Kie);
  Serial.print(" ");

  Serial.print(PID.Kde);
  Serial.print(" ");

  Serial.print(IN.TruePedal);
  Serial.print(" ");
  Serial.print(IN.TrueTPS);
  Serial.print(" ");
  Serial.print(IN.TPS1Filtered);
  Serial.print(" ");
  Serial.print(PID.deltaT);
  Serial.print(" ");
  Serial.println(u);

}
