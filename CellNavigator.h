#ifndef ENCODERPAIR_H
#define ENCODERPAIR_H

#include "mbed.h"
#include "globalConstants.h"

// This class allows the assignment of two pins for each of 
// the hall sensors of an encoder. It provides an interface
// for retriving the current recorded number of pulses
// and for resetting the pulse count
// Also allows the number of pulses to be scaled by an integer

class EncoderPair{
public:
    // Constructor:
    // sets Encoder pins for channel A(1) and channel B(2)
    // of a Quadrature Encoder Pair
    // intializes pulse count to 0
    EncoderPair(const PinName& enc1, const PinName& enc2)
    : m_count(0), m_enc1(enc1), m_enc2(enc2)
    {        
        // These lines each assign a function to call whenever
        // the signal from the encoder goes from low to high (rise)
        // or high to low (fall). In this case, a transition will call
        // the increment() function which increases the overall pulse count
        m_enc1.rise(this, &EncoderPair::increment);
        m_enc1.fall(this, &EncoderPair::increment);
        m_enc2.rise(this, &EncoderPair::increment);
        m_enc2.fall(this, &EncoderPair::increment);
    }
    
    // Constructor:
    // Same as above, but adds a "scale factor"
    // Scale factor indicates how much the pulse count
    // must be incremented everytime a voltage transition
    // is read from the encoder pins
    // Using the default constructor means that this Scale factor is 1
    EncoderPair(const PinName& enc1, const PinName& enc2,int scaleFactor)
    : m_count(0), m_enc1(enc1), m_enc2(enc2), m_scaleFactor(scaleFactor)
    {        
        m_enc1.rise(this, &EncoderPair::incrementScaled);
        m_enc1.fall(this, &EncoderPair::incrementScaled);
        m_enc2.rise(this, &EncoderPair::incrementScaled);
        m_enc2.fall(this, &EncoderPair::incrementScaled);
    }
    
    // Returns the current number of pulses recorded by the encoder so far
    unsigned long long getPulses() {return m_count;}
    
    // Resets pulse count to 0
    void resetPulses(){m_count = 0;}
    
private:
    // Helper functions
    void increment(){m_count++;}
    void incrementScaled(){m_count+=m_scaleFactor;}
    
    // self explanatory data members
    volatile unsigned long long m_count;
    unsigned long long m_scaleFactor;
    
    // Encoder pins for channels A(1) and B(2)
    InterruptIn m_enc1;
    InterruptIn m_enc2;
};



#endif
