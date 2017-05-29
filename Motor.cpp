#include "mbed.h"
#include "Motor.h"

//** Motor Control **//

Motor::Motor(const PinName& pwmInPin1, const PinName& pwmInPin2, const int& period, const int& maxPWM)
: pwmIn1(pwmInPin1), pwmIn2(pwmInPin2), m_period(period), m_maxPLen(maxPWM)
{
    initialize();
}

const int& Motor::getPeriod() const{
    return m_period;
}

void Motor::initialize(){
    pwmIn1.period_us(m_period);
    pwmIn2.period_us(m_period);
    pwmIn1.pulsewidth_us(0);
    pwmIn2.pulsewidth_us(0);
}

void Motor::changePeriod(const int& period){
    m_period = period;   
    initialize();
}

void Motor::run(int pulseLength, int dir){
    if (pulseLength > m_maxPLen)
        {pulseLength = m_maxPLen;}
    else if (pulseLength < 0)
    {
        pulseLength = -pulseLength;
        dir = !dir;
    }

    //xd.printf("%d ", pulseLength);
    //if forward == 1 , direction move forward, run motors forward         
    if (dir) {
        pwmIn1.pulsewidth_us(0);
        pwmIn2.pulsewidth_us(pulseLength);
    }
    // if forward!=1, direction = backwards
    else {
        pwmIn1.pulsewidth_us(pulseLength);
        pwmIn2.pulsewidth_us(0);
    }
}

void Motor::test(){
    run(.1*m_maxPLen,1);    
    wait_ms(1000);
    run(.1*m_maxPLen,0);    
    wait_ms(1000);
}
