#ifndef L9110S_DRIVE_H
#define L9110S_DRIVE_H

#include <Arduino.h>

class L9110SDrive
{
public:
    L9110SDrive(int bIa, int bIb, int aIa, int aIb, bool invertB = false, bool invertA = false);

    void begin();
    void stop();

    void motorBForward();
    void motorBBackward();

    void motorAForward();
    void motorABackward();

    void forward();
    void backward();
    void turnLeft();
    void turnRight();

private:
    int _bIa;
    int _bIb;
    int _aIa;
    int _aIb;
    bool _invertB;
    bool _invertA;

    void driveMotor(int in1, int in2, bool forward, bool invert);
};

#endif