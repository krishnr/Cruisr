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
volatile float Kp_Output = 0;                 // Proportional output
volatile float Ki_Output = 0;                 // Integral output
volatile float Integral = 0;                  // Integral error term for Integral Control 
volatile float Kd_Output = 0;                 // Derivative output

volatile float Current_Distance_M = 0;
volatile float Duration = 0; 
volatile float Current_Velocity_MS = 0;
volatile float Lead_Velocity_MS = 0;
volatile float Target_Distance_M = 0;

// Declare PID Gains 
static float Kp = 0;                // Set Proportioanl Gain     
static float Ki = 0;                // Set Integral Gain
static float Kd = 0;                // Set Derivative Gain

// Declare Control Loop Period in milli seconds (1-1000)
static unsigned int Period = 10;

// Stop Motor Function
void Stop_Motor()
{
  // Disable Motor Controller thereby disabling Motor 
  Motor1->setSpeed(0);
  Motor2->setSpeed(0);
}

// Set Motor Function
void Set_Motor(int Motor_Val)
{
  if(Motor_Val > 255)                 // Limit Controller_Output to 0 - 255
    Motor_Val = 255;
  if (Motor_Val < 0)
    Motor_Val = 0;
  Motor1->setSpeed(Motor_Val);
  Motor2->setSpeed(Motor_Val);
}

// Closed Loop Step Function with PID Controller
// Note: it is assumed that every Controller has a Proportional term
void CLStep(float Error)
{           
  // Calculate new error value 
  Kp_Output = Kp*(float)Error;                        // Calculate Proportional Output 
  Controller_Output = Kp_Output;                      // Set Controller Output to Proportional Output
  
  if(Ki > 0)
  {  
    Integral += float(Error)*PeriodinSeconds;         // Calculate Integral error 
    Ki_Output = Ki*Integral;                          // Calculate Integral Output 
    Controller_Output += Ki_Output;                   // Add Integral Output to Controller Output 
  }
  if(Kd > 0)
  {
    Kd_Output = (float(Error-Old_Error))*KdxFreq;     // Calculate Derivative Output
    Controller_Output += Kd_Output;                   // Add Derivative Output to Controller Output
  }
 
  Old_Error = Error;                                  // Save old error value
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

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  
  //////////////////////////////////////////////////////////////////////////////////////
  // User Defined Local Variables and Global Variable Reset
  //////////////////////////////////////////////////////////////////////////////////////
  
  
  //////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////  

  // Precalculate Loop Variables to speed up execuation time
  PeriodinSeconds = (float)Period/1000; // Control Loop Period converted to seconds from milliseconds
  Freq = 1/PeriodinSeconds;             // Calculate Control Loop Frequency in Hz
  KdxFreq = Kd*Freq;                    // Combined gain of Kd*Freq 
  
  // Send out initial settings
  Serial.println(Freq);                 // Send Freq value out serially
     
  MsTimer2::start();             // Start timer  
  
  while(true)     
  { 
    i = 0;
    total_duration = 0;
    duration = 0;
    
    while(Timer_Go == 0)            // Wait for next Timer iteration`      
    {  
      // Continuously read in and accumulate Motor Controller Current Feedback while waiting for timer
      // in an attempt to reduce noise
      total_duration += pulseIn(pingPin, HIGH);
      i++;
    }
    
    Timer_Go = 0;                   // Reset Timer flag

    if(i > 0)
      duration = total_duration/i;   // Divide by number of samples to get average
    
    Serial.println(microsecondsToCentimeters(duration));    // Send Current_Feedback value out serially    
  }
  
  Stop_Motor();                  // Stop motor  
  MsTimer2::stop();              // Stop timer
  delay(100);                    // Set delay of 100ms to debounce switch
} 

// Timer Interrupt Service Routine trigger by MsTimer2 function

void Timer_ISR()
{ 
  Duration = pulseIn(pingPin, HIGH);
  Current_Distance_M = microsecondsToCentimeters(Duration) / 100; // TODO: Smooth this out over several measurements
  
  Current_Velocity_MS = Motor_To_MS(int(Controller_Output));     // Read in current encoder position
  Lead_Velocity_MS = Current_Velocity_MS + (Current_Distance_M - Old_Distance_M) / ((float)Period / 1000); 

  Target_Distance_M = Lead_Velocity_MS * 2;

  // Closed Loop Step Mode with PID Controller
  CLStep(Target_Distance_M - Current_Distance_M);         
    
  Set_Motor(int(Controller_Output));   // Call Move Motor Function 

  old_distance_m = current_distance_m;
  Timer_Go = 1;                      // Set Timer flag
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
    return int(MS/0.0066 + 11.38)
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
  return (float)microseconds / 100 / 29 / 2;
}
