/// high-bandwidth 3-phase motor control, for robots
/// Written by benkatz, with much inspiration from Bayley Wang, Nick Kirkby, Shane Colton, David Otten, and others
/// Hardware documentation can be found at build-its.blogspot.com
/// Written for the STM32F446, but can be implemented on other STM32 MCU's with some further register-diddling
/// Version for the TI DRV8323 Everything Chip
                  
#define REST_MODE 0
#define CALIBRATION_MODE 1
#define MOTOR_MODE 2
#define SETUP_MODE 4
#define ENCODER_MODE 5

#define VERSION_NUM "1.0"


float __float_reg[64];                                                          // Floats stored in flash
int __int_reg[256];                                                             // Ints stored in flash.  Includes position sensor calibration lookup table


#include "PositionSensorMA730.h"
#include "Calibration.h"
#include "hw_setup.h"
#include "math.h" 
#include "current_controller_config.h"
#include "hw_config.h"
#include "motor_config.h"
#include "stm32f4xx_flash.h"
#include "FlashWriter.h"
#include "user_config.h"
#include "PreferenceWriter.h"
#include "CAN_com.h"
#include "DRV.h"
#include "FastMath.h"
 
PreferenceWriter prefs(6);

GPIOStruct gpio;
PIDRegulator currentPid(0.05f, 0.001, 0, 0.1, 13.8f, -13.8f);
CurrentController currentController(currentPid);

PIDRegulator speedPid(0.05f, 0.001, 0, 0.1, 40, -40);
SpeedController speedController(speedPid);
Serial pc(USART_TX_PINNAME, USART_RX_PINNAME);

CAN          can(CAN_RX_PINNAME, CAN_TX_PINNAME, CAN_FREQUENCY);      // CAN Rx pin name, CAN Tx pin name,bus frequency
CANMessage   rxMsg;
CANMessage   txMsg;

//DRV832x pin
SPI drv_spi(DRV_MOSI_PINNAME, DRV_MISO_PINNAME, DRV_SCLK_PINNAME);  //SPI(mosi,miso,sclk)
DigitalOut drv_cs(DRV_CS_PINNAME);
DRV832x drv(&drv_spi, &drv_cs);

CurrentSensor currentSensor(I_SCALE);
    
SPI ps_spi(ENCODER_MOSI_PINNAME, ENCODER_MISO_PINNAME, ENCODER_SCLK_PINNAME);
DigitalOut ps_cs(ENCODER_CS_PINNAME);
PositionSensorMA730 posSensor(&ps_spi, &ps_cs,ENCODER_CPR, NPP); 


Calibration calibration(1.0, 0.15);

volatile int count = 0;
volatile int state = REST_MODE;
volatile int state_change;

int print_flag = 0;
int print_count = 0;
int speed_loop_count = 0;
void onMsgReceived() {
    can.read(rxMsg);  //read can message
    if((rxMsg.id == CAN_ID)){
        currentController.timeout = 0;  //watch dog
        if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) & (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFC))){
            state = MOTOR_MODE;
            state_change = 1;
            }
        else if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFD))){
            state = REST_MODE;
            state_change = 1;
            gpio.led->write(0);
            }
        else if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFE))){
            posSensor.ZeroPosition();  
				    M_OFFSET = posSensor.GetMechPosition();
				    if (!prefs.ready()) prefs.open();
                        prefs.flush();                                                  // Write new prefs to flash
                        prefs.close();    
                        prefs.load(); 
                    posSensor.SetMechOffset(M_OFFSET);
            }
        else if(state == MOTOR_MODE){
            unpack_cmd(rxMsg, &currentController, &speedController);   //can command pack,16 bit position command, between -4*pi and 4*pi
                                              // 12 bit velocity command, between -30 and + 30 rad/s
                                              // 12 bit kp, between 0 and 500 N-m/rad
                                              // 12 bit kd, between 0 and 100 N-m*s/rad
                                              // 12 bit feed forward torque, between -18 and 18 N-m
            }
				//CAN Reply Packet Structure.16 bit position(-4*pi and 4*pi),12 bit velocity(-30 and + 30 rad/s),12 bit current(-40 and 40)
        pack_reply(&txMsg, posSensor.GetMechVelocity(), currentController.getId(), currentController.getIq());
        can.write(txMsg);
        }
    
}

void enter_menu_state(void){
    drv.disable_gd();
    //gpio.enable->write(0);
    printf("\r\n\r\n\r\n");
    printf(" Commands:\r\n");
    wait_us(10);
    printf(" m - Motor Mode\r\n");
    wait_us(10);
    printf(" c - Calibrate Encoder\r\n");
    wait_us(10);
    printf(" s - Setup\r\n");
    wait_us(10);
    printf(" e - Display Encoder\r\n");
    wait_us(10);
    printf(" z - Set Zero Position\r\n");
    wait_us(10);
    printf(" Esc - Exit to Menu\r\n");
    wait_us(10);
    state_change = 0;
    gpio.led->write(0);
    }

void enter_setup_state(void){
    printf("\r\n\r\n Configuration Options \r\n\r\n");
    wait_us(10);
    printf(" %-4s %-31s %-5s %-8s %-2s\r\n\r\n", "prefix", "parameter", "min", "max", "current value");
    wait_us(10);
    printf(" %-4s %-31s %-5s %-8s %.1f\r\n", "b", "Current Bandwidth (Hz)", "100", "2000", I_BW);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-8s %-5i\r\n", "i", "CAN ID", "0", "127", CAN_ID);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-8s %-5i\r\n", "m", "CAN Master ID", "0", "127", CAN_MASTER);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-8s %.1f\r\n", "l", "Current Limit (A)", "0.0", "40.0", I_MAX);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-8s %.1f\r\n", "f", "FW Current Limit (A)", "0.0", "33.0", I_FW_MAX);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-8s %d\r\n", "t", "CAN Timeout (cycles)(0 = none)", "0", "100000", CAN_TIMEOUT);
    wait_us(10);
    printf("\r\n To change a value, type 'prefix''value''ENTER' \t i.e. 'b1000''ENTER'\r\n\r\n");
    wait_us(10);
    state_change = 0;
    }
    
void enter_torque_mode(void){
    drv.enable_gd();   //enable DRV823x
    //gpio.enable->write(1);
    currentController.initController();                                                    // Tesets integrators, and other control loop parameters
    wait(.001);
    currentController.setIdRef(0);
    currentController.setIqRef(0);                                                  // Current Setpoints
    gpio.led->write(1);                                                     // Turn on status LED
    state_change = 0;
    printf("\r\n Entering Motor Mode \r\n");
    }
    
void calibrate(void){
    drv.enable_gd();
    gpio.led->write(1);     // Turn on status LED
		calibration.order_phases(&posSensor,&currentController);
		calibration.openLoopCalibrate(&currentController, &posSensor, &prefs);
    gpio.led->write(0);                                                    // Turn off status LED
    wait(.2);
    printf("\r\n Calibration complete.  Press 'esc' to return to menu\r\n");
    drv.disable_gd();
     state_change = 0;
    }
    
void print_encoder(void){
    printf(" Mechanical Angle:  %f    Electrical Angle:  %f    Raw:  %d\r\n", posSensor.GetMechPosition(), posSensor.GetElecPosition(), posSensor.GetRawPosition());
    wait(.001);
    }

/// Current Sampling Interrupt ///
/// This runs at 40 kHz, regardless of of the mode the controller is in ///
extern "C" void TIM1_UP_TIM10_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF ) {
		
		print_count++;
		
        ///Sample current always ///
        ADC1->CR2  |= 0x40000000;                                               // Begin sample and conversion

        posSensor.Sample(DT);  // sample position sensor
				currentSensor.currentSample();
		
        /// Check state machine state, and run the appropriate function ///
        switch(state){
            case REST_MODE:                                                     // Do nothing
                if(state_change){
                    enter_menu_state();
                    }
                break;
            
            case CALIBRATION_MODE:                                              // Run encoder calibration procedure
                if(state_change){
                    calibrate();
                    }
                break;
             
            case MOTOR_MODE:                                                   // Run torque control
                if(state_change){
                    enter_torque_mode();
                    count = 0;
                    }
                else{
										if((currentController.timeout > CAN_TIMEOUT) && (CAN_TIMEOUT > 0)){
												currentController.setIdRef(0);
												currentController.setIqRef(0);
												currentPid.setKp(0);
												currentPid.setKi(0);
                    } 
								

										speed_loop_count++;
										if(speed_loop_count >= 10)
										{
												speedController.doSpeedLoop(posSensor.GetMechVelocity(), &currentController);
										}
										currentController.doFoc(&currentSensor, posSensor.GetElecPosition());
									 


										currentController.timeout++;
										count++; 
            
                }     
                break;
            case SETUP_MODE:
                if(state_change){
                    enter_setup_state();
                }
                break;
            case ENCODER_MODE:
                print_encoder();
                break;
                }                 
      }
  TIM1->SR = 0x0;                            // reset the status register
			
			
}


char cmd_val[8] = {0};
char cmd_id = 0;
char char_count = 0;

/// Manage state machine with commands from serial terminal or configurator gui ///
/// Called when data received over serial ///
void serial_interrupt(void){
    while(pc.readable()){
        char c = pc.getc();
        if(c == 27){            //Esc
                state = REST_MODE;
                state_change = 1;
                char_count = 0;
                cmd_id = 0;
                gpio.led->write(0);; 
                for(int i = 0; i<8; i++){cmd_val[i] = 0;}
                }
        if(state == REST_MODE){
            switch (c){
                case 'c':
                    state = CALIBRATION_MODE;
                    state_change = 1;
                    break;
                case 'm':
                    state = MOTOR_MODE;
                    state_change = 1;
                    break;
                case 'e':
                    state = ENCODER_MODE;
                    state_change = 1;
                    break;
                case 's':
                    state = SETUP_MODE;
                    state_change = 1;
                    break;
                case 'z':
                    posSensor.SetMechOffset(0);
                    posSensor.Sample(DT);
                    wait_us(20);
                    M_OFFSET = posSensor.GetMechPosition();
                    if (!prefs.ready()) prefs.open();
                        prefs.flush();                                                  // Write new prefs to flash
                        prefs.close();    
                        prefs.load(); 
                    posSensor.SetMechOffset(M_OFFSET);
                    printf("\r\n  Saved new zero position:  %.4f\r\n\r\n", M_OFFSET);
                    
                    break;
                }
                
                }
        else if(state == SETUP_MODE){
            if(c == 13){                        //CR
                switch (cmd_id){
                    case 'b':
                        I_BW = fmaxf(fminf(atof(cmd_val), 2000.0f), 100.0f);
                        break;
                    case 'i':
                        CAN_ID = atoi(cmd_val);
                        break;
                    case 'm':
                        CAN_MASTER = atoi(cmd_val);
                        break;
                    case 'l':
                        I_MAX = fmaxf(fminf(atof(cmd_val), 40.0f), 0.0f);
                        break;
                    case 'f':
                        I_FW_MAX = fmaxf(fminf(atof(cmd_val), 33.0f), 0.0f);
                        break;
                    case 't':
                        CAN_TIMEOUT = atoi(cmd_val);
                        break;
                    default:
                        printf("\r\n '%c' Not a valid command prefix\r\n\r\n", cmd_id);
                        break;
                    }
                    
                if (!prefs.ready()) prefs.open();
                prefs.flush();                                                  // Write new prefs to flash
                prefs.close();    
                prefs.load();                                              
                state_change = 1;
                char_count = 0;
                cmd_id = 0;
                for(int i = 0; i<8; i++){cmd_val[i] = 0;}
                }
            else{
                if(char_count == 0){cmd_id = c;}
                else{
                    cmd_val[char_count-1] = c;
                    
                }
                pc.putc(c);
                char_count++;
                }
            }
        else if (state == ENCODER_MODE){
            switch (c){
                case 27:
                    state = REST_MODE;
                    state_change = 1;
                    break;
                    }
            }
        else if (state == MOTOR_MODE){
            switch (c){
                case 'd':
										currentController.setIdRef(0);
                    currentController.setIqRef(0);
                }
            }
            
        }
    }
       
int main() {
    Init_All_HW(&gpio);                                                         // Setup PWM, ADC, GPIO
    wait(.1);
    
    gpio.enable->write(1);
    wait_us(100);
    drv.calibrate();
    wait_us(100);
	//  3x PWM mode£»clear latched fault bits.This bit automatically resets after being writen.
    drv.write_DCR(0x0, 0x0, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
    wait_us(100);
	//Sense amplifier reference voltage is VREF divided by 2£»40-V/V shunt amplifier gain£»Sense OCP 1 V
    drv.write_CSACR(0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x0, 0x0, 0x0, SEN_LVL_1_0);
    wait_us(100);
	//VDS_OCP and SEN_OCP retry time is 4 ms£»200-ns dead time£»Overcurrent causes an automatic retrying fault£»
	//Overcurrent deglitch of 8 us£»1.88 V
    drv.write_OCPCR(TRETRY_4MS, DEADTIME_400NS, OCP_NONE, OCP_DEG_8US, VDS_LVL_1_88);
	
		wait_us(100);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
		drv.write_LSR(0x1, TDRIVE_4000NS, IDRIVEP_LS_120MA, IDRIVEN_LS_240MA);
		wait_us(100);
		drv.write_HSR(LOCK_OFF, IDRIVEP_HS_120MA, IDRIVEN_HS_240MA);
		
    
    //drv.enable_gd();
		currentSensor.zeroCurrent();            // Measure current sensor zero-offset
    drv.disable_gd();   //put all MOSFETs in the Hi-Z state

    wait(.1);
    currentController.initController();                                              // Reset current controller                                              // Reset observer
    TIM1->CR1 ^= TIM_CR1_UDIS;                                                  //Update disable
    //TIM1->CR1 |= TIM_CR1_UDIS; //enable interrupt
    
    wait(.1);
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2);                  // commutation > communication£¬TIM1 Update Interrupt and TIM10 global interrupt
    
    NVIC_SetPriority(CAN1_RX0_IRQn, 3);                   //CAN1 RX0 Interrupt
		//Specifies the filter identification number(CN_ID<<26),Specifies the filter mask number or identification number(0xFC000080),CANStandard format,initial filter 0
    can.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0); 
                                                                    
    // If preferences haven't been user configured yet, set defaults 
    prefs.load();                                                               // Read flash
    if(isnan(E_OFFSET)){E_OFFSET = 0.0f;}              //Returns TRUE if x is a NaN.Encoder electrical offset
    if(isnan(M_OFFSET)){M_OFFSET = 0.0f;}              //Encoder mechanical offset
    if(isnan(I_BW) || I_BW==-1||I_BW==0){I_BW = 1000;}          // Current loop bandwidth
    if(isnan(I_MAX) || I_MAX ==-1||I_MAX==0){I_MAX=40;}          // Torque limit (current limit = torque_limit/(kt*gear ratio))
    if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=0;}  // Maximum field weakening current
    if(isnan((float)CAN_ID) || CAN_ID==-1){CAN_ID = 1;}       // CAN bus ID
    if(isnan((float)CAN_MASTER) || CAN_MASTER==-1){CAN_MASTER = 0;}                 // CAN bus "master" ID
    if(isnan((float)CAN_TIMEOUT) || CAN_TIMEOUT==-1){CAN_TIMEOUT = 0;}              // CAN bus timeout period
		
		txMsg.id = CAN_MASTER;               //CAN bus "master" ID
    txMsg.len = 6;
    rxMsg.len = 8;
    can.attach(&onMsgReceived);                  // attach 'CAN receive-complete' interrupt handler    
    
		posSensor.SetElecOffset(E_OFFSET);                                             // Set position sensor offset
    posSensor.SetMechOffset(M_OFFSET);
    int lut[128] = {0};
    memcpy(lut, &ENCODER_LUT, 128*4);
    posSensor.WriteLUT(lut);                                                          // Set potision sensor nonlinearity lookup table

    pc.baud(921600);                                                            // set serial baud rate
    wait(.01);
    pc.printf("\r\n\r\n Movens \r\n\r\n");
    wait(.01);
    printf("\r\n Debug Info:\r\n");
    printf(" Firmware Version: %s\r\n", VERSION_NUM);
    printf(" ADC1 Offset:  %d\tADC2 Offset:  %d\r\n", currentSensor.getAdc1Offset(), currentSensor.getAdc2Offset());
    printf(" Position Sensor Electrical Offset:   %.4f\r\n", E_OFFSET);
    printf(" Output Zero Position:  %.4f\r\n", M_OFFSET);
    printf(" CAN ID:  %d\r\n", CAN_ID);
    

    
    pc.attach(&serial_interrupt);                                               // attach serial interrupt
    
    state_change = 1;


    while(1) {
        //drv.print_faults();
	     //wait(.1);
			
        if(state == MOTOR_MODE)
        {
            wait(0.01);
        }
		
      
    }
}
