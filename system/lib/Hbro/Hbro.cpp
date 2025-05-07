#include <Hbro.hpp>

void PulseMag(float scalar, int pinCW, int pinCCW){
    if(scalar <= 0){
        analogWrite(pinCCW, LOW);
        analogWrite(pinCW, (int)(255.0f * fabs(scalar)));
    }
    else{
        analogWrite(pinCW, LOW);
        analogWrite(pinCCW, (int)(255.0f * fabs(scalar)));
    }
}