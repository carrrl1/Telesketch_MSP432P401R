/*
 * LED.hpp
 *
 *  Created on: Jul 12, 2018
 *      Author: cesolano
 */

#ifndef JOYSTICK_HPP_
#define JOYSTICK_HPP_
#define __NOP __nop
#include "msp.h"
#include "Task.hpp"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <math.h>

// Define pixel direction
#define D_C         0x00
#define D_N         0x07
#define D_W         0x16
#define D_S         0x25
#define D_E         0x34
#define D_NW        0x43
#define D_SW        0x52
#define D_NE        0x61
#define D_SE        0x70

// Define sensitivity
#define SL          0.33
#define SH          0.66

// Define max magnitude
#define MAX_MAGNITUDE          16000

// Define the commands
#define CMD_DRAW                  0x10
#define CMD_CLEAR_DISPLAY         0x20
#define CMD_CHANGE_BACKGROUND     0x30
#define CMD_CHANGE_PEN_COLOR      0x40
#define CMD_CHANGE_PEN_SIZE       0x50

class Joystick : public Task
{
    public:
        Joystick();
        virtual uint8_t run(void);
        virtual uint8_t setup(void);
        uint8_t GetCurrentCMD(void) {return m_u8CurrentCMD;};
    protected:
    private:
        uint8_t m_u8CurrentCMD;
        void SetCurrentCMD(void);

};

#endif /* JOYSTICK_HPP_ */
