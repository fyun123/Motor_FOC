#ifndef POSITIONSENSOR_H
#define POSITIONSENSOR_H
class PositionSensor
{
public:
		virtual void init() = 0;
    virtual void Sample(float dt) = 0;
    virtual float GetMechPosition() { return 0.0f; }
    virtual float GetMechPositionFixed() { return 0.0f; }
    virtual float GetElecPosition() { return 0.0f; }
    virtual float GetMechVelocity() { return 0.0f; }
    virtual float GetElecVelocity() { return 0.0f; }
    virtual void ZeroPosition(void) = 0;
    virtual int GetRawPosition(void) = 0;
    virtual void SetElecOffset(float offset) = 0;
    virtual int GetCPR(void) = 0;
		virtual void WriteLUT(int new_lut[128]) = 0;
};

#endif
