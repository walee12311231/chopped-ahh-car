#include <Arduino.h>
#include "L9110S_Drive.h"

L9110SDrive::L9110SDrive(int bIa, int bIb, int aIa, int aIb, bool invertB, bool invertA)
    : _bIa(bIa),
      _bIb(bIb),
      _aIa(aIa),
      _aIb(aIb),
      _invertB(invertB),
      _invertA(invertA)
{
}

void L9110SDrive::begin()
{
    pinMode(_bIa, OUTPUT);
    pinMode(_bIb, OUTPUT);
    pinMode(_aIa, OUTPUT);
    pinMode(_aIb, OUTPUT);
    stop();
}

void L9110SDrive::stop()
{
    digitalWrite(_bIa, LOW);
    digitalWrite(_bIb, LOW);
    digitalWrite(_aIa, LOW);
    digitalWrite(_aIb, LOW);
}

void L9110SDrive::driveMotor(int in1, int in2, bool forward, bool invert)
{
    bool actualForward = invert ? !forward : forward;

    if (actualForward)
    {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
}

void L9110SDrive::motorBForward()
{
    driveMotor(_bIa, _bIb, true, _invertB);
}

void L9110SDrive::motorBBackward()
{
    driveMotor(_bIa, _bIb, false, _invertB);
}

void L9110SDrive::motorAForward()
{
    driveMotor(_aIa, _aIb, true, _invertA);
}

void L9110SDrive::motorABackward()
{
    driveMotor(_aIa, _aIb, false, _invertA);
}

void L9110SDrive::forward()
{
    motorBForward();
    motorAForward();
}

void L9110SDrive::backward()
{
    motorBBackward();
    motorABackward();
}

void L9110SDrive::turnLeft()
{
    motorBBackward();
    motorAForward();
}

void L9110SDrive::turnRight()
{
    motorBForward();
    motorABackward();
}