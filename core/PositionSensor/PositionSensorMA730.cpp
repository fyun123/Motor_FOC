#include "PositionSensorMA730.h"
#include "FastMath.h"

PositionSensorMA730::PositionSensorMA730(SPI *spi, DigitalOut *cs, int CPR, int ppairs){
    //_CPR = CPR;
    _CPR = CPR;           //Counts Per Revolution
    _ppairs = ppairs;
	  _spi = spi;
		_spi->format(16, 1);                // mbed v>127 breaks 16-bit spi, so transaction is broken into 2 8-bit words
		_spi->frequency(25000000);
    _cs = cs;
    }

		
void PositionSensorMA730::init()
{
    ElecOffset = 0;
    rotations = 0;
		MechOffset = 0;
		_cs->write(1);
    
    modPosition = 0;
    oldModPosition = 0;
    raw = 0;
    first_sample = 0;
		delta_pos = 0;
		vel_count = 0;
    for (int i = 0; i < 100; i++) // Initial measurement is really noisy
    {
        _cs->write(0);
        wait(100);
    }
}
    
void PositionSensorMA730::Sample(float dt){
		// 读取磁编码器的值
    _cs->write(0);                       
    raw = _spi->write(0);
    raw = raw>>2;                 //Extract last 14 bits
    _cs->write(1);     
		// 线性插值获取补偿值
    int off_1 = offset_lut[raw>>7];
    int off_2 = offset_lut[((raw>>7)+1)%128];
    int off_interp = off_1 + ((off_2 - off_1)*(raw - ((raw>>7)<<7))>>7);        // Interpolate between lookup table entries
    int angle = raw + off_interp;                                               // Correct for nonlinearity with lookup table from calibration
		// 计圈
    if (first_sample)
    {
        if (raw - old_counts > _CPR / 2)
        {
            rotations -= 1;
        }
        else if (raw - old_counts < -_CPR / 2)
        {
            rotations += 1;
        }
    }
    if (!first_sample)
    {
        first_sample = 1;
    }
    
    old_counts = angle;
    oldModPosition = modPosition;
	
    modPosition = ((2.0f*PI * ((float) angle))/ (float)_CPR);
    position = (2.0f*PI * ((float) angle+(_CPR*rotations)))/ (float)_CPR;
    MechPosition = position - MechOffset;
    float elec = ((2.0f*PI/(float)_CPR) * (float) ((_ppairs*angle)%_CPR)) + ElecOffset;
    if(elec < 0) elec += 2.0f*PI;
    else if(elec > 2.0f*PI) elec -= 2.0f*PI ; 
    ElecPosition = elec;
    
		// 计算速度
		vel_count++;
    if ((modPosition - oldModPosition) < -3.0f)
    {
        delta_pos += (modPosition - oldModPosition + 2.0f * PI);
    }
    else if ((modPosition - oldModPosition) > 3.0f)
    {
        delta_pos += (modPosition - oldModPosition - 2.0f * PI);
    }
    else
    {
        delta_pos += (modPosition - oldModPosition);
    }
		
		if(vel_count >= 40)
		{
			vel_count = 0;
			MechVelocity = 0.99f * MechVelocity + 0.01f * delta_pos * 500.0f;
			delta_pos = 0;
		}
		
    ElecVelocity = MechVelocity * _ppairs;
	
    }

int PositionSensorMA730::GetRawPosition(){
    return raw;
    }

float PositionSensorMA730::GetMechPositionFixed(){
    return MechPosition+MechOffset;
    }
    
float PositionSensorMA730::GetMechPosition(){
    return MechPosition;
    }

float PositionSensorMA730::GetElecPosition(){
    return ElecPosition;
    }

float PositionSensorMA730::GetElecVelocity(){
    return ElecVelocity;
    }

float PositionSensorMA730::GetMechVelocity(){
    return MechVelocity;
    }

void PositionSensorMA730::ZeroPosition(){
    rotations = 0;
    MechOffset = 0;
    Sample(.00005f);
    MechOffset = GetMechPosition();
    }
    
void PositionSensorMA730::SetElecOffset(float offset){
    ElecOffset = offset;
    }
void PositionSensorMA730::SetMechOffset(float offset){
    MechOffset = offset;
    }

int PositionSensorMA730::GetCPR(){
    return _CPR;
    }


void PositionSensorMA730::WriteLUT(int new_lut[128]){
    memcpy(offset_lut, new_lut, 128*4);
    }
    
