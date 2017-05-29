#include "BasicController.h"


///////////// SETUP FUNCTIONS

BasicController::BasicController(Motor* motr, Motor* motl) 
    :   m_motorr(motr), m_motorl(motl), 
    
        m_irll(NULL),
        m_irfl(NULL),
        m_irfr(NULL),
        m_irrr(NULL),
        
        irPID(IR_KD_VALUE, IR_KI_VALUE, IR_INT_CON_DECAY_FACTOR),
        m_irequilll(0),
        m_irequilrr(0),
        m_irequilfl(0),
        m_irequilfr(0),
         
        m_encr(NULL),
        m_encl(NULL),
        
        encPID(ENC_KD_VALUE, ENC_KI_VALUE, ENC_INT_CON_DECAY_FACTOR),
        rotSyncPID(ENC_KD_VALUE, ENC_KI_VALUE, ENC_INT_CON_DECAY_FACTOR),
        rotTargPID(ROT_TARG_KD_VALUE, ROT_TARG_KI_VALUE, ROT_TARG_DEC_FAC)

{
    irPID.reset();
    encPID.reset();
    rotTargPID.reset();
    rotSyncPID.reset();
    m_motorr->initialize();
    m_motorl->initialize();
}


/////////// IR SENSOR SETTING


void BasicController::setIRs(IRSensor* irll, IRSensor* irfl, IRSensor* irfr, IRSensor* irrr){
    m_irll = irll;
    m_irfl = irfl;
    m_irfr = irfr;
    m_irrr = irrr;
}


void BasicController::setIREquil () {
    for(int i=0;i<10;i++)
    {   m_irll->read(); m_irrr->read(); m_irfl->read(); m_irfr->read();}
    
    wait_ms(200);
    m_irequilll = m_irll->read();
    wait_ms(200);
    m_irequilrr = m_irrr->read();
    wait_ms(200);
    m_irequilfl = m_irfl->read();
    wait_ms(200);
    m_irequilfr = m_irfr->read();
}

// ENCODER SETTING 

void BasicController::setEncP(QEI* encr, QEI* encl){
    m_encr = encr;
    m_encl = encl;
    resetEncPulses();
}

void BasicController::resetEncPulses(){
    m_encr->reset();
    m_encl->reset();
}


/////////////////// FORWARD FUNCTIONS

void BasicController::fwd(int speed, int dir){
    // Moves Robot forward/backward by turning both motors on and using Proportional Control
    // 1 = forward, 0 = backward
    
    m_motorr->run(speed, dir);
    m_motorl->run(speed, dir);
    
    // Ensures that a whole pwm cycle passes
}

void BasicController::fwdir(int speed, int dir, int mode){

    float lirread = m_irll->read();
    float rirread = m_irrr->read();
    
    rirread = rirread - m_irequilrr;
    lirread = lirread - m_irequilll;
    
    switch(mode){
        case FOLLOW_BOTH:                
        {
            // Moves Robot forward/backward by turning both motors on and using PID control
            // 1 = forward, 0 = backward
            
            // PROPORTIONAL CONTROL
            float error =  LL_NEARNESS_SCALING_FACTOR * lirread - RR_NEARNESS_SCALING_FACTOR * rirread;
                    
            error = IR_KP_VALUE * error + irPID.updateAndGet(error);    
            // PID = adjusts motor powers to keep robot moving straight
            //encoderPID(&motorPulseWidthRight,&motorPulseWidthLeft);
            //irPID(pwmr,pwml);
            
            m_motorr->run(speed-error, dir);
            m_motorl->run(speed+error, dir);  
            // Ensures that a whole pwm cycle passes   
        }
        break;   
        //////////////////////////////////////
        case FOLLOW_LEFT:
        {
            // Moves Robot forward/backward by turning both motors on and using PID control
            // 1 = forward, 0 = backward
            
            // PROPORTIONAL CONTROL
            float error =  LL_NEARNESS_SCALING_FACTOR * lirread;
                    
            error = IR_KP_VALUE * error + irPID.updateAndGet(error);    
            // PID = adjusts motor powers to keep robot moving straight
            //encoderPID(&motorPulseWidthRight,&motorPulseWidthLeft);
            //irPID(pwmr,pwml);
            
            m_motorr->run(speed-error, dir);
            m_motorl->run(speed+error, dir);  
            // Ensures that a whole pwm cycle passes   
            
        }
        break;   
        //////////////////////////////////////
        case FOLLOW_RIGHT:
        {
            // Moves Robot forward/backward by turning both motors on and using PID control
            // 1 = forward, 0 = backward
                     
            // PROPORTIONAL CONTROL
            float error =  RR_NEARNESS_SCALING_FACTOR * rirread;
                    
            error = IR_KP_VALUE * error + irPID.updateAndGet(error);    
            // PID = adjusts motor powers to keep robot moving straight
            //encoderPID(&motorPulseWidthRight,&motorPulseWidthLeft);
            //irPID(pwmr,pwml);
            
            m_motorr->run(speed+error, dir);
            m_motorl->run(speed-error, dir);  
            // Ensures that a whole pwm cycle passes   
                
        }
        break;   
        //////////////////////////////////////
        default:
        
        break;
    }
}


void BasicController::fwdir(int speed, int dir, int mode, long long dis)
{
    m_encr->reset();
    m_encl->reset();
    
    long long lencread = m_encl->getPulses();
    long long rencread = m_encr->getPulses();
    
    while(lencread < dis || rencread < dis )
    {
        fwdir(speed,dir,mode);  
        lencread = m_encl->getPulses();
        rencread = m_encr->getPulses(); 
    }    
    fwd(0,0);    
}

void BasicController::fwdenc(int speed, int dir){
           
    
    // Moves Robot forward/backward by turning both motors on and using PID control
    // 1 = forward, 0 = backward
    long long lencread = m_encl->getPulses();
    long long rencread = m_encr->getPulses();
    
    // PROPORTIONAL CONTROL
    long long error = lencread - rencread ;
    
    if(!dir)
        error = -error;
            
    error = ENC_KP_VALUE * error + encPID.updateAndGet(error);    
    // PID = adjusts motor powers to keep robot moving straight
    //encoderPID(&motorPulseWidthRight,&motorPulseWidthLeft);
    //irPID(pwmr,pwml);
    
    m_motorr->run(speed+error, dir);
    m_motorl->run(speed-error, dir);  
    // Ensures that a whole pwm cycle passes   

}

void BasicController::fwdenc(int speed, int dir, long long dis)
{

    m_encr->reset();
    m_encl->reset();
    
    long long lencread = m_encl->getPulses();
    long long rencread = m_encr->getPulses();
    
    while(abs(lencread) < dis || abs(rencread) < dis )
    {
        fwdenc(speed,dir);  
        lencread = m_encl->getPulses();
        rencread = m_encr->getPulses(); 
    }    
    fwd(0,0);
}

void BasicController::rotate(int speed, int dir, int degrees)
{
    // CURRENT ENCODER VALUES RESET
    m_encl->reset();
    m_encr->reset();
    
    long long lencread = m_encl->getPulses();
    long long rencread = m_encr->getPulses();
    
    // TARGET ENCODER VALUES 
    long long ltarg = ((degrees * NUM_PULSES_PER_NINETY) / 90);
    
    if(!dir)
        ltarg = -ltarg;
    Timer t;
    t.start();
    while(t.read_ms()<500){
                        
       // PROPORTIONAL CONTROL
        long long syncError = abs(lencread) - abs(rencread) ;
        long long targError = ltarg - lencread;
        
        if(!dir)
            targError = - targError;
                
        syncError = ENC_KP_VALUE * syncError + rotSyncPID.updateAndGet(syncError);    
        targError = ROT_TARG_KP_VALUE * targError + rotTargPID.updateAndGet(targError);
        
        m_motorr->run(targError+syncError, !dir);
        m_motorl->run(targError-syncError, dir);  
        // Ensures that a whole pwm cycle passes   
        
        lencread = m_encl->getPulses();
        rencread = m_encr->getPulses();
    }
    m_motorr->run(0,0);
    m_motorl->run(0,0);
}   

