/*
 * File Name: SpeedController.h
 * Description: Inherit from Controller class
 */

#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include "Controller.h"
#include "PositionSensor.h"
#include "CurrentController.h"
class SpeedController : public Controller
{
public:
    SpeedController(PIDRegulator pidRegulator);
    virtual void initController();
    void setSpeedRef(float speed_reference);
    void doSpeedLoop(float speed_mech, CurrentController *cc);
private:
    float speed_ref;
};

#endif

/*
 * Current Version: 1.0
 * Author: Fang Yun
 * Create Time: 2022/1/21
 */
