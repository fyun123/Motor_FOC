#ifndef POSITIONSENSOR_MA730_H
#define POSITIONSENSOR_MA730_H

#include "mbed.h"
#include "PositionSensor.h"


class PositionSensorMA730: public PositionSensor{
public:
    PositionSensorMA730(SPI *spi, DigitalOut *cs, int CPR, int ppairs);
		virtual void init();
    virtual void Sample(float dt);
    virtual float GetMechPosition();
    virtual float GetMechPositionFixed();
    virtual float GetElecPosition();
    virtual float GetMechVelocity();
    virtual float GetElecVelocity();
    virtual int GetRawPosition();
    virtual void ZeroPosition();
    virtual void SetElecOffset(float offset);
    virtual void SetMechOffset(float offset);
    virtual int GetCPR(void);
    virtual void WriteLUT(int new_lut[128]);
private:
    float position, ElecPosition, ElecOffset, MechPosition, MechOffset, modPosition, oldModPosition, MechVelocity, ElecVelocity, ElecVelocityFilt;
    int raw, _CPR, rotations, old_counts, _ppairs, first_sample;
		float delta_pos, vel_count;
    SPI *_spi;
    DigitalOut *_cs;
    int readAngleCmd;
    int offset_lut[128];

};

#endif
