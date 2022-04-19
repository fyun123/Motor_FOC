/*
 * File Name: Controller.h
 * Description: Controller Class
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "PIDRegulator.h"
#include "user_config.h"
class Controller
{
public:
    PIDRegulator _pidRegulator;
    Controller(PIDRegulator pidRegulator);
    virtual void initController() = 0;
};

#endif

/*
 * Current Version: 1.0
 * Author: Fang Yun
 * Create Time: 2022/1/21
 */
