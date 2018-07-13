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

//Pixels
#define DISPLAY_SIZE        127
#define DISPLAY_MID         63

//RGB color code
#define WHITE_BACKGROUND    0xFFFFFF

#define BLACK_PEN           0x000000

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
        Graphics_Rectangle m_sPen;
        uint8_t m_u8PenSize, m_u8PenX, m_u8PenY;
        uint32_t m_u32PenColor, m_u32Background;

        void SetPenRectangle(uint8_t i_u8Min, uint8_t i_u8xMax, uint8_t i_u8yMin, uint8_t i_u8yMax);
        void SetPenLocation(uint8_t i_u8PenX, uint8_t i_u8PenY);
        void MovePen();

		//Graphics_Display_Functions m_sDisplay_Functions;
		//Graphics_Display m_sFontFixed;
};

#endif /* LCD_HPP_ */
