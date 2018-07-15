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
#define YELLOW_BACKGROUND   0xFF9999
#define BLACK_BACKGROUND    0x000000

#define BLACK_PEN           0x000000
#define WHITE_PEN           0xFFFFFF
#define BLUE_PEN            0x3333FF
#define RED_PEN             0xFF0000
#define GREEN_PEN           0x00CC00

#define PEN_COLORS          5
#define BACKGROUND_COLORS   3
#define MAX_PEN_SIZE        3

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
        const uint32_t m_u32PenColors[PEN_COLORS] = {
                                                     BLACK_PEN,
                                                     WHITE_PEN,
                                                     BLUE_PEN,
                                                     RED_PEN,
                                                     GREEN_PEN
        };
        const uint32_t m_u32BackgroundColors[BACKGROUND_COLORS] = {
                                                             WHITE_BACKGROUND,
                                                             YELLOW_BACKGROUND,
                                                             BLACK_BACKGROUND
        };
        uint8_t m_u8CurrentPenColor = 0, m_u8CurrentBackgroundColor=0;

        void SetPenRectangle(uint8_t i_u8Min, uint8_t i_u8xMax, uint8_t i_u8yMin, uint8_t i_u8yMax);
        void SetPenLocation(uint8_t i_u8PenX, uint8_t i_u8PenY);
        void MovePen();
        void ChangePenColor();
        void ChangePenSize();
        void ChangeBackgroundColor();
        void ClearDisplay();

};

#endif /* LCD_HPP_ */
