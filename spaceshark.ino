// library to move the stepper motor
// Space Shark microcontroller firmware for Particle Photon
// See project pages at http://github.com/spaceshark
#include "TinyStepper_28BYJ_48.h"
#include <math.h>

// Define stepper motor pin connections, these are the digital pins
// on the Particle Photon to which stepper motor will be connected.
const int MOTOR_IN1_PIN = 0;
const int MOTOR_IN2_PIN = 1;
const int MOTOR_IN3_PIN = 2;
const int MOTOR_IN4_PIN = 3;


// Constant values defining the value ranges for the alt-az coordinate system:
// A normal servo only rotates from +90 degrees to -90 degrees a total of 180 degrees
// All stepper motor moves from 0 to 360 degrees.
const float alt_min = -90.0;
const float alt_max = 90.0;
const float az_min = 0.0;
const float az_max = 360.0;

// Create servo and stepper motor instances:
Servo servo_alt;
TinyStepper_28BYJ_48 stepper_az;

// These are used to track how long ago each motor had its pointing updated:
float lastUpdate_alt = millis();
float lastUpdate_az = millis();
float lastUpdate_sys = millis();


// The following values are particular to the hardware. Every servo motor is
// a bit different, so the alt and az motors need to be calibrated for their
// maximum and minimum angles. The values below are the ones given to the servo
// 'write' function, which attempts to send control signals matching the angles
// but will be slightly off. These need to be found empirically, e.g. by
// setting the initial pointing to the min and max alt/az angles and then
// nudging these limits either side of their nominal values.
const float limit_alt_lo = 0; // Nominally 90.0
const float limit_alt_hi = 180;   // Nominally 0.0
const float limit_az_lo = 0;   // Nominally 0.0
const float limit_az_hi = 2048;  // Nominally 360.0

// Set the intial pointing and track rate to use when the Shark is powered on:
float posVal_sky_alt = 0.0; // 0 degrees is horizon, 90 degrees is zenith
float posVal_sky_az = 0.0;  // 0 degrees is north, 90 degrees is east
float trackRate_alt = 0.0;  // in degrees per second
float trackRate_az = 0.0;   // in degrees per second

//
float stepperPos_deg = 0;

// Set homing inputs and variables
int optoInt_Val = 0;
bool hasHomed = false;

void setup()
{
    // Define IO pins and register cloud functions:
    pinMode(A0,INPUT);
    servo_alt.attach(A5);
    stepper_az.connectToPins(MOTOR_IN1_PIN, MOTOR_IN2_PIN, MOTOR_IN3_PIN, MOTOR_IN4_PIN);

    Particle.function("point_alt_az", point_alt_az);
    Particle.function("track_alt", track_alt);
    Particle.function("track_az", track_az);
    Serial.begin(9600);


}

int counter = 0;
signed int x = 0;
void loop()
{
    if (hasHomed == true)  // checking if the device is already in home position
    {
      //This is the main loop, which never stops updating the pointning angles
      //based on the current tracking rate.

      float now = millis();
      if ((now - lastUpdate_sys) > 1000)
      {
        lastUpdate_sys = now;
        update_pointing();
        set_pos(posVal_sky_alt, posVal_sky_az);
      }
    }
    else
    {
      // Read the value at opto interrupter (min: 0, max: 4096)
      optoInt_Val = analogRead(A0);
      Serial.println(optoInt_Val);
      // Check if the opto-interruptor has been triggered (when it
      // drops below 3800, and max value is 4096)
      if (optoInt_Val <= 3800)
      {
        // Stepper motor has homed, reset speed and acceleration to slower
        // settings and exit homing sequence
        stepper_az.setSpeedInStepsPerSecond(128);
        stepper_az.setAccelerationInStepsPerSecondPerSecond(128);

        // Disable the motor after homing to avoid overheating/stalling the motor
        stepper_az.disableMotor();

        hasHomed = true;
        counter = 0;
      }
      else
      {
      // We need to home the stepper motor, so move it in 64 step increments
      // in one direction for 180 degrees, then go the other direction.
      // This is to avoid tangling the wires (it homes in the shortest direction)
        if (counter>1024)
        {
          x = -64 ;
        }
        else
        {
          x = 64;
        }

        stepper_az.setSpeedInStepsPerSecond(128);
        stepper_az.setAccelerationInStepsPerSecondPerSecond(4096);
        stepper_az.moveRelativeInSteps(x);
        counter += 64;
      }
    }
}

void update_pointing()
{
    // Change the current pointing angles based on the current tracking rates
    // We are taking the angular velocity and multiplying with time to get the new angle
    // and updating the current altitude or azimuthal angle to the new altitude
    // or azimuthal angle respectively

    if (trackRate_alt != 0)
    {
        float now = millis();
        float elapsedTime_alt = now - lastUpdate_alt;
        posVal_sky_alt = posVal_sky_alt + (trackRate_alt*(elapsedTime_alt/1000.0));
        if (posVal_sky_alt > alt_max)
        {
            posVal_sky_alt = alt_max;
        }
        lastUpdate_alt = now;
    }
    else
    {
        posVal_sky_alt = posVal_sky_alt + 0;
    }

    if (trackRate_az != 0)
    {
        float now = millis();
        float elapsedTime_az = now - lastUpdate_az;
        posVal_sky_az = posVal_sky_az + (trackRate_az*(elapsedTime_az/1000.0));
        if (posVal_sky_az > az_max)
        {
            posVal_sky_az = az_max;
        }
        lastUpdate_az = now;
    }
    else
    {
        posVal_sky_az = posVal_sky_az + 0;
    }
}

float diff_move = 0;
int set_pos(float alt, float az)
{
    float move = 0;
    move = get_stepper_move();
    float posVal_servo_alt = convert_alt(posVal_sky_alt);
    float posVal_stepper_az = convert_az(move);

    if (fabs(posVal_stepper_az) > 1)
    {

      // Take current pointing angles, convert them, and move motors:

      Serial.println(stepperPos_deg);
      Serial.println(posVal_sky_az);
      Serial.println(move);

      stepper_az.moveRelativeInSteps(posVal_stepper_az);
      stepper_az.disableMotor();
      stepperPos_deg = posVal_sky_az;
    }
    servo_alt.write(posVal_servo_alt);

    return 0;
}


float get_stepper_move()
{
    // get_stepper_move() computes the required move (in degrees) that the
    // stepper motor should take to go from current position (stepperPos_deg) to
    // the desired position (posVal_sky_az), without going past the opto-interruptor
    float move = 0;  // actual move for stepper motor to take
    float theta = 0; // angular distance between current pos and desired

    // First, compute theta, the angular distance to our desired move
    // There are two cases to consider so that we get the sign correct
    if (stepperPos_deg > posVal_sky_az)
    {
      theta = posVal_sky_az + 360 - stepperPos_deg;
    }
    else if (posVal_sky_az > stepperPos_deg)
    {
      theta = -1*(stepperPos_deg + 360 - posVal_sky_az);
    }

    // Given theta, we need to ensure moving theta degrees will not
    // go below 0 or above 360 degrees (either of which will cross the
    // stepper motor). This logic will check the two conditions (0 or 360)
    // and reverse theta if necessary
    if (theta+stepperPos_deg > 360)
    {
      move = -1 * (360 - theta);
    }
    else if (theta+stepperPos_deg < 0)
    {
      move = 360 + theta;
    }
    else
    {
      move = theta;
    }

    // This depends on how the stepper motor is wired, and what is defined
    // as clockwise and counterclockwise. In our case, we need to multiply by
    // -1 to ensure it moves the right direction.
    return -1*move;
}


// The following functions convert 'sky' coordinates to 'servo' coordinates,
// i.e. converting from degrees into steps or servo position
float convert_alt(float alt)
{
    return sky_to_servo(alt, alt_min, alt_max, limit_alt_lo, limit_alt_hi);
}

float convert_az(float az)
{
    return sky_to_stepper(az, az_min, az_max, limit_az_lo, limit_az_hi);
}

float sky_to_servo(
    float sky,
    float sky_min,
    float sky_max,
    float servo_limit_lo,
    float servo_limit_hi)
{
    if (sky < sky_min)
        return sky_min;
    if (sky > sky_max)
        return sky_max;
    float scale = (sky-sky_min) / (sky_max-sky_min);

    return scale * (servo_limit_hi - servo_limit_lo) + servo_limit_lo;
}

float sky_to_stepper(
    float sky,
    float sky_min,
    float sky_max,
    float servo_limit_lo,
    float servo_limit_hi)
{
    float scale = (sky-sky_min) / (sky_max-sky_min);
    return scale * (servo_limit_hi - servo_limit_lo) + servo_limit_lo;
}

// The following fuctions are exposed to the outside world (the cloud):
int point_alt_az(String posString)
{
    int sepIndex = posString.indexOf(',');
    String posString_alt = posString.substring(0,sepIndex);
    String posString_az = posString.substring(sepIndex+1);
    posVal_sky_alt = posString_alt.toFloat();
    posVal_sky_az = posString_az.toFloat();
    float now = millis();
    lastUpdate_alt = now;
    return 0;
}

int track_alt(String rate)
{
    trackRate_alt = rate.toFloat();
    return 0;
}

int track_az(String rate)
{
    trackRate_az = rate.toFloat();
    return 0;
}
