#ifndef CAN_COM_H
#define CAN_COM_H

#include "user_config.h"
#include "CurrentController.h"
#include "SpeedController.h"

void pack_reply(CANMessage *msg, float p, float v, float t);
void unpack_cmd(CANMessage msg, CurrentController * cc, SpeedController * sc);


#endif
