#ifndef BASICCONTROLLER_H
#define BASICCONTROLLER_H

#include "mbed.h"
#include "globalConstants.h"
#include "Motor.h"
#include "QEI.h"
#include "IRSensor.h"
#include "PID.h"



// Given a set of motor, IR, and encoder objects, this class
// provides an interface for making a robot move forward or 
// rotate at a certain speed and direction. It also provides
// the option to add PID control based on either encoder readings
// or IR readings. 


class BasicController {
public:
    // Constructor: Sets the "right" and "left" motors 
    // that the basic controller can work with
    BasicController(Motor* motr, Motor* motl);
    
    // Sets the duty cycle of both motors to speed
    // speed can be from 0 to 100
    // Also sets both motors to have the same dir.
    // dir = 1 usually means forward
    void fwd(int speed, int dir);


    //** IR sensor related **//
    
    // Sets the IRSensors that the controller can use
    void setIRs(IRSensor* irll, IRSensor* irfl, IRSensor* irfr, IRSensor* irrr);
     
    void setIREquil();
    
    //** Encoder Relation **//
    
    // Sets the EncoderPairs that this controller can use
    void setEncP(QEI* encr, QEI* encl);
    
    // reset encoder pulses
    void resetEncPulses();
    
    // In General:
    // Makes the motor's move forward at an effective duty cycle of <speed> towards <dir> direction
    // but uses IR PID to adjust left/right motor duty cycle so that the robot doesn't run into walls
    // mode 0 = both walls
    // mode 1 = right wall
    // mode 2 = left wall
    void fwdir(int speed, int dir, int mode); 
    void fwdir(int speed, int dir, int mode, long long dis);
    
    void fwdenc(int speed, int dir);
    void fwdenc(int speed, int dir, long long dis);
    
    void rotate(int speed, int dir, int degrees);
     
     float getIREquilL(){   return m_irequilll; }
     float getIREquilR(){   return m_irequilrr; }
     float getIREquilFL(){   return m_irequilfl; }
     float getIREquilFR(){   return m_irequilfr; }

protected:

    Motor* m_motorr;
    Motor* m_motorl;
    
    IRSensor* m_irll;
    IRSensor* m_irfl;
    IRSensor* m_irfr;
    IRSensor* m_irrr;

    QEI* m_encr;
    QEI* m_encl;
    
    
    //** PID OBJECTS **//
    PID<float> irPID;
    PID<long long> encPID;
    PID<long long> rotSyncPID;
    PID<long long> rotTargPID;
    
    // ** IR PID STUFF **//
    float m_irequilll;
    float m_irequilrr;
    float m_irequilfl;
    float m_irequilfr;
};




#endif
