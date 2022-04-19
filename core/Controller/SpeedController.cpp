/*
 * File Name: SpeedController.cpp
 * Description: Controller of Speed Loop.
 */

#include "SpeedController.h"

/* 
 * Call the constructor of the parent class
 */
SpeedController::SpeedController(PIDRegulator pidRegulator) : Controller(pidRegulator){};

/*
 * Initialize attribute of SpeedController
 */
void SpeedController::initController(){
    speed_ref = 0;
}

/*
 * Set reference of speed
 */
void SpeedController::setSpeedRef(float speed_reference){
    speed_ref = speed_reference;
}

/*
 * Speed Close Control. 
 * The PID output of speed loop is the reference current of q-axis
 */
void SpeedController::doSpeedLoop(float speed_mech, CurrentController *cc)
{
    cc->setIqRef(_pidRegulator.computePID(speed_ref, speed_mech));
}

/*
 * Current Version: 1.0
 * Author: Fang Yun
 * Create Time: 2022/1/21
 */
