#ifndef IRSENSOR_H
#define IRSENSOR_H

// This class allows the assignment of two pins for the IR LED's power
// and the IR phototransitor's reading pin and provides an interface 
// for making an IR LED-Transistor pair perform a single reading

class IRSensor {
public:
    // Constructor: Sets IR LED pin and IR transistor pin
    // Inititalizes stored reading to 0
    IRSensor(const PinName& led,const PinName& trans): m_led(led), m_trans(trans), m_reading(0){}
    
    // Powers IR LED, reads transistor Voltage, and then turns off the LED as fast as possible
    // Additionally, records the value of reading so it can still be accessed later
    float read(){ 
        m_led.write(1);
        wait_ms(1);
        m_reading = m_trans.read();
        m_led.write(0);
        wait_ms(1);
        return m_reading;
    }
    
    // returns stored reading from previous read()
    float prevRead(){
        return m_reading;
    }
    
private:
    DigitalOut m_led;
    AnalogIn m_trans;
    float m_reading;
};

#endif
