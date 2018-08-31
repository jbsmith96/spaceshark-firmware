// Space Shark microcontroller firmware for Particle Photon
// See project pages at http://github.com/spaceshark
#include "TinyStepper_28BYJ_48.h"

// Define stepper motor pin connections
const int MOTOR_IN1_PIN = 1;
const int MOTOR_IN2_PIN = 2;
const int MOTOR_IN3_PIN = 3;
const int MOTOR_IN4_PIN = 4;


// Constant values defining the value ranges for the alt-az coordinate system:
const float alt_min = -90.0;
const float alt_max = 90.0;
const float az_min = 0.0;
const float az_max = 360.0;

// Create servo motor instances:
Servo servo_alt;
TinyStepper_28BYJ_48 stepper_az;
// These are used to track how long ago each motor had its pointing updated:
float lastUpdate_alt = millis();
float lastUpdate_az = millis();

float stepper_pos = 0;

// The following values are particular to the hardware. Every servo motor is
// a bit different, so the alt and az motors need to be calibrated for their
// maximum and minimum angles. The values below are the ones given to the servo
// 'write' function, which attempts to send control signals matching the angles
// but will be slightly off. These need to be found empirically, e.g. by
// setting the initial pointing to the min and max alt/az angles and then
// nudging these limits either side of their nominal values.
const float limit_alt_lo = 103.0; // Nominally 90.0
const float limit_alt_hi = 9.0;   // Nominally 0.0
const float limit_az_lo = 0;   // Nominally 0.0
const float limit_az_hi = 2048;  // Nominally 360.0

// Set the intial pointing and track rate to use when the Shark is powered on:
float posVal_sky_alt = 0.0; // 0 degrees is horizon, 90 degrees is zenith
float posVal_sky_az = 0.0;  // 0 degrees is north, 90 degrees is east
float trackRate_alt = 0.0;  // in degrees per second
float trackRate_az = 0.0;   // in degrees per second

void setup()
{
    // Define servo output pins and register cloud functions:
    servo_alt.attach(A4);
    stepper_az.connectToPins(MOTOR_IN1_PIN, MOTOR_IN2_PIN, MOTOR_IN3_PIN, MOTOR_IN4_PIN);

    Particle.function("point_alt_az", point_alt_az);
    Particle.function("track_alt", track_alt);
    Particle.function("track_az", track_az);
}

void loop()
{
    // This is the main loop, which never stops updating the pointning angles
    // based on the current tracking rate.
    update_pointing();
    set_pos(posVal_sky_alt, posVal_sky_az);
    delay(50);
}

void update_pointing()
{
    // Change the current pointing angles based on the current tracking rates
    if (trackRate_alt > 0)
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

    if (trackRate_az > 0)
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


int set_pos(float alt, float az)
{
    // Take current pointing angles, convert them, and move motors:
    float posVal_servo_alt = convert_alt(posVal_sky_alt);
    float posVal_servo_az = convert_az(posVal_sky_az);
    servo_alt.write(posVal_servo_alt);

    stepper_az.setSpeedInStepsPerSecond(156);
    stepper_az.setAccelerationInStepsPerSecondPerSecond(512);
    //float posVal_servo_az_Steps = posVal_servo_az*2048/360;
    float diff_move = stepper_pos- posVal_servo_az;
    stepper_az.moveRelativeInSteps(diff_move);
    stepper_pos = posVal_servo_az;
    return 0;
}


// The following functions convert 'sky' coordinates to 'servo' coordinates,
// which account for the motor calibration offsets
float convert_alt(float alt)
{
    return sky_to_servo(alt, alt_min, alt_max, limit_alt_lo, limit_alt_hi);
}

float convert_az(float az)
{
    return sky_to_servo(az, az_min, az_max, limit_az_lo, limit_az_hi);
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
