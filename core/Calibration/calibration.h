/*
 * File Name: Calibration.h
 * Description: Calibration Class
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "CurrentSensor.h"
#include "PositionSensor.h"
#include "CurrentController.h"
#include "current_controller_config.h"
#include "PreferenceWriter.h"
#include "Math.h"

class Calibration
{
public:
    Calibration(float i_d, float v_d);
    void order_phases(PositionSensor *ps, CurrentController *currentController);
    void closeLoopCalibrate(CurrentSensor *cs, PositionSensor *ps, CurrentController *currentController);
		void openLoopCalibrate(CurrentController *cc, PositionSensor *ps, PreferenceWriter *prefs);
    bool getPhaseOrder();
    float getElecOffset();

private:
    float _i_d, _i_q, _v_d, _v_q;
    float elec_offset;
};

#endif

/*
 * Current Version: 1.0
 * Author: Fang Yun
 * Create Time: 2022/1/21
 */
