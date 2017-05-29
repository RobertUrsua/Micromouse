
template <typename ItemType> class PID {
public:
    PID(int kdval, int kival, float dec): prevError(0), accError(0), kd(kdval), ki(kival), decayFactor(dec) { reset(); }
    
    ItemType updateAndGet(ItemType curError) {
        ItemType correction = prevError - curError;
        accError = accError + curError;
        
        correction = kd * correction + ki * accError + curError;
        accError = accError * decayFactor;
        prevError = curError;
        return correction;
    }
    
    void reset() { prevError = 0; accError = 0; }

private:
    ItemType prevError;
    ItemType accError;
    
    float decayFactor;
    int kd;
    int ki;    
};
