#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"
#include "globalConstants.h"

// This class allows assigning two pins to the inputs of the motor and
// provides an interface for making the motors spin at a certain
// direction and speed.

class Motor{
public:
    // Constructor:
    // Sets the 2 pwm pins for a motor
    // Sets the period for pwm, and the maximum pwm that
    // would be allowed for the operation of the motor
    Motor(const PinName& pwmInPin1, const PinName& pwmInPin2, const int& period, const int& maxPWM);
    
    // returns the motor's pwm period
    const int& getPeriod() const;
    
    // changes the motor's pwm period
    void changePeriod(const int& period);
    
    // Set's the motor's pwm period 
    // and sets the pwm/duty cycle to 0
    // Useful if we want to apply a new
    // pwm period
    void initialize();
    
    // Assigns a new duty cycle / "ON" pulse duration
    // to the motor. Also sets the direction for motor
    // movement. 
    //
    // Ideally, dir 1 = forwards, dir 0 = backwards
    //
    // dir = 0 means the pin assinged to pwmIn1 will get an "ON" pulse that lasts 0 uS
    //         and the pin assinged to pwmIn2 will get an "ON" pulse that lasts <pulseLength> uS
    // dir = 1 reverses this assignment
    // example:
    //
    // Motor rightMotor(PA_0, PA_1, 200, 200);
    // rightMotor has pwm period 200 micro seconds (max duty cycle 200 uS / 200 uS)
    //
    // run (100, 0);
    // sets a duty cycle of 100uS / 200uS to PA_0 (pwmIn1)
    // and sets a duty cycle of 0uS / 200uS to PA1 (pwmIn2)
    void run(int pulseLength, int dir);
    
    // makes the motor turn one direction at 10% duty cycle for one second
    // then makes it turn the other direction at 10% duty cycle for another second
    void test();
    
    
private:
    // pwm pins controlling the motor
    PwmOut pwmIn1;
    PwmOut pwmIn2;
    
    // pwm period
    int m_period;
    
    // maximum length of time that the motor can be turned on within a pwm period
    int m_maxPLen;
};

#endif
