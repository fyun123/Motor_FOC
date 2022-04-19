#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#define PIN_U PA_10
#define PIN_V PA_9
#define PIN_W PA_8
#define ENABLE_PIN PA_11        // Enable gate drive pin
#define LED         PC_5        // LED Pin
#define I_SCALE 0.02014160156f  // Amps per A/D Count
#define DTC_MAX 0.498f          // Max phase duty cycle
#define DTC_MIN 0.002f          // Min phase duty cycle
#define PWM_ARR 0x1194           /// timer autoreload value 20kHz

#define USART_TX_PINNAME PA_2
#define USART_RX_PINNAME PA_3

#define CAN_RX_PINNAME PB_8
#define CAN_TX_PINNAME PB_9
#define CAN_FREQUENCY 1000000

#define DRV_MOSI_PINNAME PA_7
#define DRV_MISO_PINNAME PA_6
#define DRV_SCLK_PINNAME PA_5
#define DRV_CS_PINNAME PA_4

#define ENCODER_MOSI_PINNAME PC_12
#define ENCODER_MISO_PINNAME PC_11
#define ENCODER_SCLK_PINNAME PC_10
#define ENCODER_CS_PINNAME PA_15
#define ENCODER_CPR 16384		// Counts Per Revolution

static float inverter_tab[16] = {2.5f, 2.4f, 2.3f, 2.2f, 2.1f, 2.0f, 1.9f, 1.8f, 1.7f, 1.6f, 1.59f, 1.58f, 1.57f, 1.56f, 1.55f, 1.5f};


#endif
