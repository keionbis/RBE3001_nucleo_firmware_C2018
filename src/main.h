#ifndef MAIN_H
#define MAIN_H

#include "AS5050.h"
#include "mbed.h"
#if !defined(DEVICE_ANALOGOUT)
#define DEVICE_ANALOGOUT
#endif
#include <AnalogIn.h>
#include <AnalogOut.h>

#include "../mbed-os/targets/TARGET_STM/TARGET_STM32F7/TARGET_STM32F746xG/TARGET_NUCLEO_F746ZG/PinNames.h"
#include "../mbed-os/drivers/AnalogIn.h"
#include "../mbed-os/drivers/AnalogOut.h"
#include "../mbed-os/drivers/SPI.h"
#include "drivers/Clock.h"
#include "Servo.h"
#include "drivers/MyPid.h"
#include "drivers/DummyPID.h"
#include "drivers/HIDPacket.h"
#include "USBHID.h"
#include "RunEvery.h"
/*
 *Coms
 *Place additional "coms" here
*/
#include "coms/PidServer.h"
#include "coms/PidConfigServer.h"
#include "coms/PDVelocityConfigServer.h"
#include "coms/VelocityTarget.h"

#define RELEASE
// DEFINES
#define MOSI PB_5 // HDMI 16
#define CLK PB_3 // HDMI 13
#define MISO PB_4 // HDMI 15
#if defined(RELEASE)
//GROUND     HDMI 19,1,2,3
//3.3 volts  HDMI 18
//Motor 8.4v HDMI 6,5,4 
// HDMI 9
#define SERVO_1 PE_9
#define SERVO_2 PE_11
#define SERVO_3 PE_13
//Full turn PCB values
//HDMI 17
#define ENC_1 PC_8
#define ENC_2 PC_9
#define ENC_3 PC_10
//HDMI 12
#define LOAD_1 PA_3
#define LOAD_2 PC_0
#define LOAD_3 PC_3
#else
// Plan C breakout boards
#define PLAN_C_BOARD
#define SERVO_1 PC_6
#define SERVO_2 PB_15
#define SERVO_3 PB_13

#define ENC_1 PD_14
#define ENC_2 PD_15
#define ENC_3 PF_12

//HDMI 12
#define LOAD_1 PA_3
#define LOAD_2 PC_0
#define LOAD_3 PC_3
#endif

#endif
