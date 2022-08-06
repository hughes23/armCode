/*
 * 
 * https://forum.arduino.cc/t/log-pot-to-linear-when-you-have-to-use-a-log-pot/117050
 * 
 * board manager: esp32 - 1.0.2
 * 
 */


// Improve I2C communication
#include <Wire.h>
// Servo Library
#include <Adafruit_PWMServoDriver.h>
#include <Ps3Controller.h>

// Define object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// DroneBot's Servo constant Parameters (180 servo)
#define MIN_PUSLSE_WIDTH  650
#define MAX_PUSLSE_WIDTH  2350
#define MID_POINT         174
#define FREQUENCY         50      // Standard for most servos

/*
// Define Potentiometer input pins
int controlA = A0;
int controlB = A1;  
int controlC = A2;  
int controlD = A3;  
int controlE = A4;  
*/

// Define PWM output pins
int motorA = 0;
int motorB = 3;
int motorC = 4;
int motorD = 7;
int motorE = 8;


// Setup ESP32 variables
int player = 1;  // code wants to loop through player number... NO.
int battery = 0;

void setup() {
  Serial.println("Setup Loop");
  
  Serial.begin(115200); // baud rate: number of bits of data sent per second

  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("b8:08:cf:20:70:a6");

  Ps3.setPlayer(player);
  
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  // Move servos to start position
  //  startingPosition();

  Serial.println("Ready.");
}

void loop() {
  // Serial.println("Looping...");

  if(!Ps3.isConnected()) {
    return;
  }
  // Control Motor A
  // runMotor(controlA, motorA);

  // Control Motor B
  // runMotor(controlB, motorB);

  // Control Motor C
  // runMotor(controlC, motorC);

  // Control Motor D
  // runMotor(controlD, motorD);

  // Control Motor E
  // runMotor(controlE, motorE);

  delay(2000);
}

 
void runMotor(int controlIn, int motorOut) {
  // Serial.println("runMotor Loop");
  
  int pulseWide;  // Scales the analog range into the range the servos accept 
  int pulseWidth; // Converts into a 12 bit scale to reach the max/min range of the pwm pin
  int potValue; // Read the current potentiometer value and assign to variable

  potValue = potReading( controlIn );
  
  Serial.println(potValue);
  
  pulseWide = map(potValue, 0, 1023, MIN_PUSLSE_WIDTH, MAX_PUSLSE_WIDTH);
  pulseWidth = int(float(pulseWide) / 1000000 * FREQUENCY * 4096);
  
  
  // Set location of the servo motor
  pwm.setPWM(motorOut, 0, pulseWidth);
}


int potReading(int controlIn) {
  // Eliminate potentiometer noise through looking at difference in two readings (if difference >5 move servo)
  int potValue1;
  int potValue2; 

  potValue1 = analogRead(controlIn);
  potValue1 = log(potValue1) / log(1000) * 1023;
  delay(10);
  potValue2 = analogRead(controlIn);
  potValue2 = log(potValue2) / log(1000) * 1023;

  if ( abs(potValue1 - potValue2) > 40 ) {
    return potValue2;
  }

  return potValue1;
}


void startingPosition() {
  Serial.println("Start Position");
  /*
  // Set motors to 90 degrees
  //pwm.setPWM(motorA, 0, analogRead(controlA));
  //delay(1000);
  
  pwm.setPWM(motorB, 0, analogRead(controlB));
  delay(1000);
  
  
  pwm.setPWM(motorC, 0, analogRead(controlC));
  delay(1000);
  pwm.setPWM(motorD, 0, analogRead(controlD));
  delay(1000);
  pwm.setPWM(motorE, 0, analogRead(controlE));
  */
}

void onConnect(){
    Serial.println("Connected.");
}

void notify()
{

    //--------------- Digital D-pad button events --------------
    if( Ps3.event.button_down.up )
        Serial.println("Started pressing the up button");
    if( Ps3.event.button_up.up )
        Serial.println("Released the up button");

    if( Ps3.event.button_down.right )
        Serial.println("Started pressing the right button");
    if( Ps3.event.button_up.right )
        Serial.println("Released the right button");

    if( Ps3.event.button_down.down )
        Serial.println("Started pressing the down button");
    if( Ps3.event.button_up.down )
        Serial.println("Released the down button");

    if( Ps3.event.button_down.left )
        Serial.println("Started pressing the left button");
    if( Ps3.event.button_up.left )
        Serial.println("Released the left button");

   //--------------- Analog D-pad button events ----------------
   if( Ps3.event.analog_changed.button.up ) {
       Serial.print("Pressing the up button: ");
       Serial.println(Ps3.data.analog.button.up, DEC);
       runMotor(Ps3.data.analog.button.up, motorA);
   }

   if( Ps3.event.analog_changed.button.right ) {
       Serial.print("Pressing the right button: ");
       Serial.println(Ps3.data.analog.button.right, DEC);
       runMotor(Ps3.data.analog.button.right, motorB);
   }

   if( Ps3.event.analog_changed.button.down ) {
       Serial.print("Pressing the down button: ");
       Serial.println(Ps3.data.analog.button.down, DEC);
       runMotor(Ps3.data.analog.button.down, motorA);
   }

   if( Ps3.event.analog_changed.button.left ) {
       Serial.print("Pressing the left button: ");
       Serial.println(Ps3.data.analog.button.left, DEC);
       runMotor(Ps3.data.analog.button.left, motorB);
   }

   //---------------------- Battery events ---------------------
    if( battery != Ps3.data.status.battery ){
        battery = Ps3.data.status.battery;
        Serial.print("The controller battery is ");
        if( battery == ps3_status_battery_charging )      Serial.println("charging");
        else if( battery == ps3_status_battery_full )     Serial.println("FULL");
        else if( battery == ps3_status_battery_high )     Serial.println("HIGH");
        else if( battery == ps3_status_battery_low)       Serial.println("LOW");
        else if( battery == ps3_status_battery_dying )    Serial.println("DYING");
        else if( battery == ps3_status_battery_shutdown ) Serial.println("SHUTDOWN");
        else Serial.println("UNDEFINED");
    }
}
