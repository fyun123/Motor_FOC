#include "CAN_com.h"
#include "FastMath.h"

 #define P_MIN -95.5f
 #define P_MAX 95.5f
 #define V_MIN -60.0f
 #define V_MAX 60.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -18.0f
 #define T_MAX 18.0f
 

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void pack_reply(CANMessage *msg, float p, float v, float t){
    int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(t, -T_MAX, T_MAX, 12);
    msg->data[0] = CAN_ID;
    msg->data[1] = p_int>>8;
    msg->data[2] = p_int&0xFF;
    msg->data[3] = v_int>>4;
    msg->data[4] = ((v_int&0xF)<<4) + (t_int>>8);
    msg->data[5] = t_int&0xFF;
    }
    
/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void unpack_cmd(CANMessage msg, CurrentController * cc, SpeedController * sc){
        int v_int = (msg.data[0]<<8)|msg.data[1];
        int v_kp = (msg.data[2]<<4)|(msg.data[3]>>4);
        int v_ki = ((msg.data[3]&0xF)<<8)|msg.data[4];
        int c_kp = (msg.data[5]<<4)|(msg.data[6]>>4);
        int c_ki = ((msg.data[6]&0xF)<<8)|msg.data[7];
        sc->setSpeedRef(uint_to_float(v_int, P_MIN, P_MAX, 16));
				sc->_pidRegulator.setKp(uint_to_float(v_kp, V_MIN, V_MAX, 12));
        sc->_pidRegulator.setKi(uint_to_float(v_ki, KP_MIN, KP_MAX, 12));
        cc->_pidRegulator.setKp(uint_to_float(c_kp, KD_MIN, KD_MAX, 12));
        cc->_pidRegulator.setKi(uint_to_float(c_ki, T_MIN, T_MAX, 12));
//    printf("Received   ");
//    printf("p_des=%.3f  v_des=%.3f  kp=%.3f  kd=%.3f  tff=%.3f   iq=%.3f\r\n", controller->p_des, controller->v_des, controller->kp, controller->kd, controller->t_ff, controller->i_q_ref);
    
    }

