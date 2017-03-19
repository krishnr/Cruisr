///////////////////////////////////////////////////////////
//  Base Code v1
//  May 25, 2016
//  By Chris McClellan
///////////////////////////////////////////////////////////

#include <MsTimer2.h>
#include <math.h>

// PID library: playground.arduino.cc/Code/PIDLibrary

const int pingPin = 7;

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *Motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);

// Setup Motor Controller 1 Pins
static byte PWM1_Out = 4;      // Motor Controller PWM1 assigned to Digital Pin 4 
static byte PWM2_Out = 5;      // Motor Controller PWM2 assigned to Digital Pin 5 
static byte Disable_Out = 6;   // Motor Controller Disable assigned to Digital Pin 6 
static byte En_Out = 7;        // Motor Controller Enable assigned to Digital Pin 7
static byte FB_AIn = 0;        // Motor Controller Current Feedback assigned to Analog Pin 0

// Setup Encoder Counter 1 Pins
static byte SEL1 = 38;         // Select 1 output signal assigned to Pin 38
static byte OE = 39;           // Output Enable signal assinged to Pin 39
static byte RST = 40;          // Reset output signal assigned to Pin 40
static byte SEL2 = 41;         // Select 2 output signal assinged to Pin 41

// Declare Contstants
static float pi = 3.141593;

// Setup ISR Global Variables
volatile byte Timer_Go = 0;                   // Interrupt flag for timer 
volatile int Current_Velocity = 0;
volatile int Lead_Velocity = 0;
volatile signed long int Reference_Input_Encoder = 0;   // Reference Input Signal in encoder counts (R(s)*I(s))
volatile float VtoPWM = 0;                    // Conversion factor for Volt to PWM  
volatile float KdxFreq = 0;                   // Combined gain of Kd*Freq  
volatile float PeriodinSeconds = 0;           // Control Loop Period in seconds
volatile float Freq = 0;                      // Control Loop Frequency
volatile float t = 0;                         // Time counter in seconds
volatile float temp = 0;                      // temp variable used as intermediary for assigning Ramp Slope to Reference Input
volatile signed long int Old_Error = 0;       // Previous position error
volatile float Controller_Output = 0;         // Control loop Controller_Output
volatile float Controller_Old = 0;
volatile float Kp_Output = 0;                 // Proportional output
volatile float Ki_Output = 0;                 // Integral output
volatile float Integral = 0;                  // Integral error term for Integral Control 
volatile float Kd_Output = 0;                 // Derivative output

volatile float Old_Distance_M = 0;
volatile float Current_Distance_M = 0;
volatile float Duration = 0; 
volatile float Current_Velocity_MS = 0;
volatile float Lead_Velocity_MS = 0;
volatile float Target_Distance_M = 0;

// Declare PID Gains 
static float Kp = -46.9346806407297;                // Set Proportioanl Gain     
static float Ki = -2.66438460805086;                // Set Integral Gain
static float Kd = 17.4556171048772;                // Set Derivative Gain

// Declare Control Loop Period in milli seconds (1-1000)
static unsigned long Period = 10;

// Stop Motor Function
void Stop_Motor()
{
  // Disable Motor Controller thereby disabling Motor 
  Motor1->setSpeed(0);
  Motor2->setSpeed(0);
}

volatile int counter = 0;

// Set Motor Function
void Set_Motor(int Motor_Val)
{ 
  if(Motor_Val > 255)                 // Limit Controller_Output to 0 - 255
    Motor_Val = 255;
  if (Motor_Val < 50)
    Motor_Val = 0;
    
  Motor1->setSpeed(Motor_Val);
  Motor2->setSpeed(Motor_Val);
  Motor1->run(FORWARD); 
  Motor2->run(BACKWARD);

//  Motor1->run(RELEASE);
//  Motor2->run(RELEASE);
}

// Closed Loop Step Function with PID Controller
// Note: it is assumed that every Controller has a Proportional term
void CLStep(float Error)
{           
  //Controller_Output = Controller_Output + Error * -200.0 + (Error - Old_Error) * 20.0;

  if (abs(Error) > 0.3) {
    Controller_Output = MS_To_Motor(Lead_Velocity_MS) + Error * -40;
  } else {
    Controller_Output = MS_To_Motor(Lead_Velocity_MS);
  }

  if (Controller_Output < 0) {
    Controller_Output = 0;
  } else if (Controller_Output > 255) {
    Controller_Output = 255;
  } 

  Controller_Output = Controller_Old * 0.8 + Controller_Output * 0.2;

  if (MS_To_Motor(Lead_Velocity_MS) < 20 && Error < 0 && Error > -0.5) {
    Controller_Output = 70;
  }

  //if ((Controller_Output - Controller_Old) > 5) {
    //Controller_Output = Controller_Old + 5;
  //} else if ((Controller_Output - Controller_Old) < -5) {
    //Controller_Output = Controller_Old - 5;
  //}
  
  Controller_Old = Controller_Output;
  Old_Error = Error;
} 

//////////////////////////////////////////////////////////////////////////////////////////
// User Defined Subroutines/Functions
//////////////////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// Main Setup Routine
void setup() 
{
  Serial.begin(9600);            // Setup serial output to 115,200 bps (bits per second)

     
  // Setup Data input pins from Encoder as inputs
  AFMS.begin();                     // Set pins 22-29 as inputs
  
  Motor1->run(FORWARD);
  Motor1->run(RELEASE);
  
  Motor2->run(BACKWARD);
  Motor2->run(RELEASE);

  Stop_Motor();
  
  // Setup Timer for Control Loop Frequency
  MsTimer2::set(Period, Timer_ISR);   // Set Control Loop Frequency (Hz) to 1000/Period   
}

void loop()
{  
  // Reset Global Variables     
  Old_Error = 0;                        // Reset previous position error 
  Integral = 0;                         // Reset Integral error term for PID Control  
  temp = 0;                             // Reset temp variable

  // and the distance result in inches and centimeters:
  long duration, inches, cm, total_duration;
  int i;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);


  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  
  //////////////////////////////////////////////////////////////////////////////////////
  // User Defined Local Variables and Global Variable Reset
  //////////////////////////////////////////////////////////////////////////////////////
  
  
  //////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////  

  // Precalculate Loop Variables to speed up execuation time
  PeriodinSeconds = (float)Period/1000; // Control Loop Period converted to seconds from milliseconds
  Freq = 1/PeriodinSeconds;             // Calculate Control Loop Frequency in Hz
  KdxFreq = Kd*Freq;                    // Combined gain of Kd*Freq 
  
  //MsTimer2::start(); // Start timer
  
  while(true)     
  { 
    
    while(true)            // Wait for next Timer iteration`      
    {  
      Timer_ISR();
      delay(2);
    }
    
    Timer_Go = 0;                   // Reset Timer flag

    Serial.println("Current Distance: ");
    Serial.println(Current_Distance_M);
    Serial.println("Lead Velocity: ");
    Serial.println(Lead_Velocity_MS);
    Serial.println("Controller Output: ");
    Serial.println(int(Controller_Output));   
  }
  
  Stop_Motor();                  // Stop motor  
  MsTimer2::stop();              // Stop timer
  delay(100);                    // Set delay of 100ms to debounce switch
}

long read_sensor() {
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);


  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);

  return pulseIn(pingPin, HIGH); 
}

volatile float Old_Velocity_MS = 0;
volatile float Old_Duration = 0;

// Timer Interrupt Service Routine trigger by MsTimer2 function
void Timer_ISR()
{ 
  Duration = Old_Duration * 0.8 + read_sensor() * 0.2;

  if (Duration - Old_Duration > 200) {
    Duration = Old_Duration + 200;
  } else if (Duration - Old_Duration < -200) {
    Duration = Old_Duration - 200;
  }
  
  Old_Duration = Duration;
  Current_Distance_M = microsecondsToMeters(Duration); // TODO: Smooth this out over several measurements
  
  Lead_Velocity_MS = (Current_Distance_M - Old_Distance_M) / ((float)Period / 1000.0);
  Lead_Velocity_MS = Old_Velocity_MS * 0.99 + Lead_Velocity_MS * 0.01;

  if (abs(Lead_Velocity_MS - Old_Velocity_MS) > 0.2) {
    Lead_Velocity_MS = Old_Velocity_MS;
  } 

  Old_Velocity_MS = Lead_Velocity_MS;

  Current_Distance_M = Old_Distance_M * 0.9 + Current_Distance_M * 0.1;

  if (Current_Distance_M - Old_Distance_M > 0.05) {
    Current_Distance_M = Old_Distance_M + 0.05;
  } else if (Current_Distance_M - Old_Distance_M < -0.05) {
    Current_Distance_M = Old_Distance_M - 0.05;
  }
  
  Old_Distance_M = Current_Distance_M;

  Target_Distance_M = Lead_Velocity_MS;
  
  // Closed Loop Step Mode with PID Controller
  Target_Distance_M = 0.5;
  
  CLStep(Target_Distance_M - Current_Distance_M); 
  Serial.println(Controller_Output);    

  if (Current_Distance_M < 0.2) {
    Controller_Output = 0;
  }
  
  Set_Motor(int(Controller_Output));   // Call Move Motor Function 
//
    Old_Distance_M = Current_Distance_M;
//  Timer_Go = 1;                      // Set Timer flag
}

float Motor_To_MS(int Motor_Val) {
  if (Motor_Val < 30) {
    return 0;
  } else {
    return (float)Motor_Val * 0.0066 - 0.0751;
  }
}

int MS_To_Motor(float MS) {
  if (MS < 0) {
    return 0;
  } else {
    return int(MS/0.0156 + 11.38);
  }
}

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

float microsecondsToMeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return (float)microseconds / 100.0 / 29.0 / 2.0;
}
