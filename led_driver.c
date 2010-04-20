/**
 * Driver functions for the A6276 Led driver
 */

#ifndef LED_DRIVER_C
#define LED_DRIVER_C


#include "HardwareProfile.h"

#include "led_driver.h"

void sendBit(BYTE out)
{
    lData = out;
    lClock_On();
    
    lClock_Off();
}

void sendByte(BYTE out) {
    BYTE i;
    for (i=0; i<8; i++) {
        sendBit(out & 0x01);
        out = out >> 1;
    }
}

void setLeds(BYTE out[2]) {

    // We start with the lowest bit, and moving towards the higher
    sendByte(out[1]);
    sendByte(out[0]);

    
    lLatch_On();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    lLatch_Off();
}

#endif
