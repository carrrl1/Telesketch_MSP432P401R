/*
 * LCD.hpp
 *
 *  Created on: May 22, 2018
 *      Author: cesolano
 */

#ifndef LCD_HPP_
#define LCD_HPP_
#define __NOP __nop

#include "msp.h"
#include "Task.hpp"
#include "LCD_DRIVER.hpp"

#define DISPLAY_SIZE        127
#define DISPLAY_MID_LOW        63
#define DISPLAY_MID_HIGH        64
#define EARTH_COLOR 0x663300
#define SKY_COLOR 0x3399FF
#define LINE_COLOR 0xFFFFFF    

//LCD class
class LCD : public Task
{
    public:
        LCD();
        virtual uint8_t run(void);
        virtual uint8_t setup(void);

    protected:
    private:
    	
    	/* Graphic library context */
		Graphics_Context m_sContext;
		Graphics_Display m_sDisplay;
        Graphics_Rectangle m_sSky, m_sEarth, m_sLine;
		//Graphics_Display_Functions m_sDisplay_Functions;
		//Graphics_Display m_sFontFixed;
};

#endif /* LCD_HPP_ */
