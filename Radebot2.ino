#include <PS2X_lib.h>
#include <Servo.h>

#define SERVO_MINIMUM_ANGLE 0
#define SERVO_MAXIMUM_ANGLE 180

#define DEADBAND_ABSOLUTE 2
#define PS2_CONTROLLER_MINIMUM_VALUE 0.0
#define PS2_CONTROLLER_MAXIMUM_VALUE 255.0

#define PS2_CONTROLLER_NEUTRAL_VALUE 127.5
#define GUN 12

#define CANDY_SHOOTER_MOTOR_PIN 8
#define LEFT_DRIVE_MOTOR_PIN 9
#define RIGHT_DRIVE_MOTOR_PIN 10

#define CANDY_SHOOTER_MOTOR_STATE_OFF 0
#define CANDY_SHOOTER_MOTOR_STATE_ON 1

#define CANDY_SHOOTER_MOTOR_SPEED 110

// Pins to be determined, current pins might be inadequate. 
#define LEFT_BALL_CATCHER_MOTOR_PIN 11
#define RIGHT_BALL_CATCHER_MOTOR_PIN 12
#define CATCHER_LIFT_MOTOR_PIN 13 // Motor to move both left and right arms up and down, will be stopped mechanically by limit switches.

#define LIFT_UPPER_LIMIT_PIN 14 // Pins for the limit switches
#define LIFT_LOWER_LIMIT_PIN 15

#define IS_DPAD_UP_PRESSED false

PS2X ps2x;

byte vibrate = 0;
double RY = 0;
double LX = 0;
int candyShooterMotorState = CANDY_SHOOTER_MOTOR_STATE_OFF;

// initialize the library with the numbers of the interface pins
//LiquidCrystal lcd(2, 3, 22, 24, 26, 28);

Servo rightDriveMotor, leftDriveMotor, candyShooterMotor;

/**************************************************************
   setup()
 **************************************************************/
void setup() {
  Serial.begin(9600);
  
  // Attach the motors to their respective pins
  candyShooterMotor.attach(CANDY_SHOOTER_MOTOR_PIN);
  leftDriveMotor.attach(LEFT_DRIVE_MOTOR_PIN);
  rightDriveMotor.attach(RIGHT_DRIVE_MOTOR_PIN);

  // Set the motors to their "stopped" position
  candyShooterMotor.write(SERVO_MAXIMUM_ANGLE / 2);
  leftDriveMotor.write(SERVO_MAXIMUM_ANGLE / 2);
  rightDriveMotor.write(SERVO_MAXIMUM_ANGLE / 2);

  // Set the pin modes for catcher
  pinMode(LIFT_UPPER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(LIFT_LOWER_LIMIT_PIN, INPUT_PULLUP);
  
  // Setup controller pins and settings:
  // GamePad(clock, command, attention, data, Pressures?, Rumble?)
  ps2x.config_gamepad(13, 32, 30, 38, true, true);
}



/**************************************************************
   loop()
 **************************************************************/
void loop() {

  // Read PS2 Controller
  ps2x.read_gamepad(false, vibrate);

  // Drive Motor Handling 
  int Throttle = PS2_CONTROLLER_NEUTRAL_VALUE;
  int Turn = PS2_CONTROLLER_NEUTRAL_VALUE;
  float TurnPercent = 0.4;
  
  Throttle = ps2x.Analog(PSS_LY); // Reading Left stick Y axis 0 - 255
  Turn = ps2x.Analog(PSS_RX); // Reading Right stick X axis 0 - 255
  
  if(Throttle > PS2_CONTROLLER_NEUTRAL_VALUE + DEADBAND_ABSOLUTE)
    Throttle = map(Throttle, PS2_CONTROLLER_NEUTRAL_VALUE + DEADBAND_ABSOLUTE, 255, 90, 90 + (0.9 * GUN));
  else if(Throttle < PS2_CONTROLLER_NEUTRAL_VALUE - DEADBAND_ABSOLUTE)
    Throttle = map(Throttle, 0, PS2_CONTROLLER_NEUTRAL_VALUE - DEADBAND_ABSOLUTE, 90 - (0.9 * GUN), 90);
  else {
    Throttle = 90;
    TurnPercent *= 2;
  }
  if(Turn > PS2_CONTROLLER_NEUTRAL_VALUE + DEADBAND_ABSOLUTE)
    Turn = map(Turn, PS2_CONTROLLER_NEUTRAL_VALUE + DEADBAND_ABSOLUTE, 255, 90, 90 + (0.9 * GUN * TurnPercent));
  else if(Turn < PS2_CONTROLLER_NEUTRAL_VALUE - DEADBAND_ABSOLUTE)
    Turn = map(Turn, 0, PS2_CONTROLLER_NEUTRAL_VALUE - DEADBAND_ABSOLUTE, 90 - (0.9 * GUN * TurnPercent), 90);
  else Turn = 90;

  int rightPower = constrain(Throttle + (Turn - 90), 0, 180);
  int leftPower = constrain(Turn - (Throttle - 90), 0, 180);

  
  Serial.print(" Right: ");
  Serial.print(rightPower);
  Serial.print(" Left: ");
  Serial.print(leftPower);
  Serial.print(" Throttle: ");
  Serial.print(Throttle);
  Serial.print(" Turn: ");
  Serial.print(Turn);
  Serial.println();
  //*/

  leftDriveMotor.write(leftPower);
  rightDriveMotor.write(rightPower);


  //  Candy Shooter Motor Handling
  if (ps2x.NewButtonState()) { // will be TRUE if any button changes state

    if (ps2x.Button(PSB_R2)) // will be TRUE as long RED circle button is pressed
    {
      if (candyShooterMotorState == CANDY_SHOOTER_MOTOR_STATE_OFF)
      {
        candyShooterMotorState = CANDY_SHOOTER_MOTOR_STATE_ON;
        Serial.print("Candy Shooter Motor On -->");
        Serial.println();
      }
      else
      {
        candyShooterMotorState = CANDY_SHOOTER_MOTOR_STATE_OFF;
        Serial.print("<-- Candy Shooter Motor Off");
        Serial.println();
      }

      // I think we need this delay to debouce the button presses
      // but the value is in question.
      delay(100);
    }
  }

  if (candyShooterMotorState == CANDY_SHOOTER_MOTOR_STATE_OFF)
  {
    candyShooterMotor.write(SERVO_MAXIMUM_ANGLE / 2);
  }
  else
  {
    candyShooterMotor.write(CANDY_SHOOTER_MOTOR_SPEED);
  }


}
