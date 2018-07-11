#include "LCD_DRIVER.hpp"

uint8_t g_u8Orientation;

//*****************************************************************************
//
// Writes a command to the CFAF128128B-0145T.  This function implements the basic SPI
// interface to the LCD display.
//
//*****************************************************************************
void HAL_LCD_writeCommand(uint8_t i_u8Command)
{
    // Set to command mode
    GPIO_setOutputLowOnPin(LCD_DC_PORT, LCD_DC_PIN);

    // USCI_B0 Busy? //
    while (UCB0STATW & UCBUSY);

    // Transmit data
    UCB0TXBUF = i_u8Command;

    // USCI_B0 Busy? //
    while (UCB0STATW & UCBUSY);

    // Set back to data mode
    GPIO_setOutputHighOnPin(LCD_DC_PORT, LCD_DC_PIN);
}


//*****************************************************************************
//
// Writes a data to the CFAF128128B-0145T.  This function implements the basic SPI
// interface to the LCD display.
//
void HAL_LCD_writeData(uint8_t i_u8Data)
{
    // USCI_B0 Busy? //
    while (UCB0STATW & UCBUSY);

    // Transmit data
    UCB0TXBUF = i_u8Data;

    // USCI_B0 Busy? //
    while (UCB0STATW & UCBUSY);
}

void SetDrawFrame(uint16_t i_u16x0, uint16_t i_u16y0, uint16_t i_u16x1, uint16_t i_u16y1)
{
    switch (g_u8Orientation) {
        case 0:
            i_u16x0 += 2;
            i_u16y0 += 3;
            i_u16x1 += 2;
            i_u16y1 += 3;
            break;
        case 1:
            i_u16x0 += 3;
            i_u16y0 += 2;
            i_u16x1 += 3;
            i_u16y1 += 2;
            break;
        case 2:
            i_u16x0 += 2;
            i_u16y0 += 1;
            i_u16x1 += 2;
            i_u16y1 += 1;
            break;
        case 3:
            i_u16x0 += 1;
            i_u16y0 += 2;
            i_u16x1 += 1;
            i_u16y1 += 2;
            break;
        default:
            break;
    }

    HAL_LCD_writeCommand(CM_CASET);
    HAL_LCD_writeData((uint8_t)(i_u16x0 >> 8));
    HAL_LCD_writeData((uint8_t)(i_u16x0));
    HAL_LCD_writeData((uint8_t)(i_u16x1 >> 8));
    HAL_LCD_writeData((uint8_t)(i_u16x1));

    HAL_LCD_writeCommand(CM_RASET);
    HAL_LCD_writeData((uint8_t)(i_u16y0 >> 8));
    HAL_LCD_writeData((uint8_t)(i_u16y0));
    HAL_LCD_writeData((uint8_t)(i_u16y1 >> 8));
    HAL_LCD_writeData((uint8_t)(i_u16y1));
}

//*****************************************************************************
//
//! Sets the LCD Orientation.
//!
//! \param orientation is the desired orientation for the LCD. Valid values are:
//!           - \b LCD_ORIENTATION_UP,
//!           - \b LCD_ORIENTATION_LEFT,
//!           - \b LCD_ORIENTATION_DOWN,
//!           - \b LCD_ORIENTATION_RIGHT,
//!
//! This function sets the orientation of the LCD
//!
//! \return None.
//
//*****************************************************************************
void SetOrientation(uint8_t i_u8Orientation)
{
    g_u8Orientation = i_u8Orientation;
    HAL_LCD_writeCommand(CM_MADCTL);
    switch (g_u8Orientation) {
        case LCD_ORIENTATION_UP:
            HAL_LCD_writeData(CM_MADCTL_MX | CM_MADCTL_MY | CM_MADCTL_BGR);
            break;
        case LCD_ORIENTATION_LEFT:
            HAL_LCD_writeData(CM_MADCTL_MY | CM_MADCTL_MV | CM_MADCTL_BGR);
            break;
        case LCD_ORIENTATION_DOWN:
            HAL_LCD_writeData(CM_MADCTL_BGR);
            break;
        case LCD_ORIENTATION_RIGHT:
            HAL_LCD_writeData(CM_MADCTL_MX | CM_MADCTL_MV | CM_MADCTL_BGR);
            break;
    }
}

//*****************************************************************************
//
//! Draws a pixel on the screen.
//!
//! \param pDisplay is a pointer to the driver-specific data for this
//! display driver.
//! \param i_i16lX is the X coordinate of the pixel.
//! \param lY is the Y coordinate of the pixel.
//! \param ulValue is the color of the pixel.
//!
//! This function sets the given pixel to a particular color.  The coordinates
//! of the pixel are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
void PixelDraw(const Graphics_Display *i_pDisplay, 
                          int16_t i_u16lX, 
                          int16_t i_u16lY, 
                          uint16_t i_u16ulValue)
{

    SetDrawFrame(i_u16lX,i_u16lY,i_u16lX,i_u16lY);

    //
    // Write the pixel value.
    //
    HAL_LCD_writeCommand(CM_RAMWR);
    HAL_LCD_writeData(i_u16ulValue>>8);
    HAL_LCD_writeData(i_u16ulValue);
}

//*****************************************************************************
//
//! Draws a horizontal sequence of pixels on the screen.
//!
//! \param pDisplay is a pointer to the driver-specific data for this
//! display driver.
//! \param i_i16lX is the X coordinate of the first pixel.
//! \param i_i16lY is the Y coordinate of the first pixel.
//! \param i_i16lX0 is sub-pixel offset within the pixel data, which is valid for 1
//! or 4 bit per pixel formats.
//! \param i_i16lCount is the number of pixels to draw.
//! \param i_i16lBPP is the number of bits per pixel; must be 1, 4, or 8.
//! \param i_pData is a pointer to the pixel data.  For 1 and 4 bit per pixel
//! formats, the most significant bit(s) represent the left-most pixel.
//! \param i_pPalette is a pointer to the palette used to draw the pixels.
//!
//! This function draws a horizontal sequence of pixels on the screen, using
//! the supplied palette.  For 1 bit per pixel format, the palette contains
//! pre-translated colors; for 4 and 8 bit per pixel formats, the palette
//! contains 24-bit RGB values that must be translated before being written to
//! the display.
//!
//! \return None.
//
//*****************************************************************************
void PixelDrawMultiple(const Graphics_Display *i_pDisplay,
                                                  int16_t i_i16lX,
                                                  int16_t i_i16lY,
                                                  int16_t i_i16lX0,
                                                  int16_t i_i16lCount,
                                                  int16_t i_i16lBPP,
                                                  const uint8_t *i_pData,
                                                  const uint32_t *i_pPalette)
{
    uint16_t Data;

    //
    // Set the cursor increment to left to right, followed by top to bottom.
    //
    SetDrawFrame(i_i16lX,i_i16lY,i_i16lX+i_i16lCount,127);
    HAL_LCD_writeCommand(CM_RAMWR);

    //
    // Determine how to interpret the pixel data based on the number of bits
    // per pixel.
    //
    switch(i_i16lBPP)
    {
        // The pixel data is in 1 bit per pixel format
        case 1:
        {
            // Loop while there are more pixels to draw
            while(i_i16lCount > 0)
            {
                // Get the next byte of image data
                Data = *i_pData++;

                // Loop through the pixels in this byte of image data
                for(; (i_i16lX0 < 8) && i_i16lCount; i_i16lX0++, i_i16lCount--)
                {
                    // Draw this pixel in the appropriate color
                    HAL_LCD_writeData((((uint32_t *)i_pPalette)[(Data >>
                                                             (7 - i_i16lX0)) & 1])>>8);
                    HAL_LCD_writeData(((uint32_t *)i_pPalette)[(Data >>
                                                             (7 - i_i16lX0)) & 1]);
                }

                // Start at the beginning of the next byte of image data
                i_i16lX0 = 0;
            }
            // The image data has been drawn

            break;
        }

        // The pixel data is in 4 bit per pixel format
        case 4:
        {
            // Loop while there are more pixels to draw.  "Duff's device" is
            // used to jump into the middle of the loop if the first nibble of
            // the pixel data should not be used.  Duff's device makes use of
            // the fact that a case statement is legal anywhere within a
            // sub-block of a switch statement.  See
            // http://en.wikipedia.org/wiki/Duff's_device for detailed
            // information about Duff's device.
            switch(i_i16lX0 & 1)
            {
                case 0:

                    while(i_i16lCount)
                    {
                        // Get the upper nibble of the next byte of pixel data
                        // and extract the corresponding entry from the palette
                        Data = (*i_pData >> 4);
                        Data = (*(uint16_t *)(i_pPalette + Data));
                        // Write to LCD screen
                        HAL_LCD_writeData(Data>>8);
                        HAL_LCD_writeData(Data);

                        // Decrement the count of pixels to draw
                        i_i16lCount--;

                        // See if there is another pixel to draw
                        if(i_i16lCount)
                        {
                case 1:
                            // Get the lower nibble of the next byte of pixel
                            // data and extract the corresponding entry from
                            // the palette
                            Data = (*i_pData++ & 15);
                            Data = (*(uint16_t *)(i_pPalette + Data));
                            // Write to LCD screen
                            HAL_LCD_writeData(Data>>8);
                            HAL_LCD_writeData(Data);

                            // Decrement the count of pixels to draw
                            i_i16lCount--;
                        }
                    }
            }
            // The image data has been drawn.

            break;
        }

        // The pixel data is in 8 bit per pixel format
        case 8:
        {
            // Loop while there are more pixels to draw
            while(i_i16lCount--)
            {
                // Get the next byte of pixel data and extract the
                // corresponding entry from the palette
                Data = *i_pData++;
                Data = (*(uint16_t *)(i_pPalette + Data));
                // Write to LCD screen
                HAL_LCD_writeData(Data>>8);
                HAL_LCD_writeData(Data);
            }
            // The image data has been drawn
            break;
        }

        //
        // We are being passed data in the display's native format.  Merely
        // write it directly to the display.  This is a special case which is
        // not used by the graphics library but which is helpful to
        // applications which may want to handle, for example, JPEG images.
        //
        case 16:
        {
            uint16_t usData;

            // Loop while there are more pixels to draw.

            while(i_i16lCount--)
            {
                // Get the next byte of pixel data and extract the
                // corresponding entry from the palette
                usData = *((uint16_t *)i_pData);
                i_pData += 2;

                // Translate this palette entry and write it to the screen
                HAL_LCD_writeData(usData>>8);
                HAL_LCD_writeData(usData);
            }
        }
    }
}

//*****************************************************************************
//
//! Draws a horizontal line.
//!
//! \param i_pDisplay is a pointer to the driver-specific data for this
//! display driver.
//! \param i_i16lX1 is the X coordinate of the start of the line.
//! \param i_i16lX2 is the X coordinate of the end of the line.
//! \param i_i16lY is the Y coordinate of the line.
//! \param i_u16Value is the color of the line.
//!
//! This function draws a horizontal line on the display.  The coordinates of
//! the line are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
void LineDrawH(const Graphics_Display *i_pDisplay,
                                          int16_t i_i16lX1,
                                          int16_t i_i16lX2,
                                          int16_t i_i16lY,
                                          uint16_t i_u16Value)
{
    SetDrawFrame(i_i16lX1, i_i16lY, i_i16lX2, i_i16lY);

    // Write the pixel value.

    HAL_LCD_writeCommand(CM_RAMWR);
    for (int i = i_i16lX1; i <= i_i16lX2; i++)
    {
        HAL_LCD_writeData(i_u16Value>>8);
        HAL_LCD_writeData(i_u16Value);
    }
}

//*****************************************************************************
//
//! Draws a vertical line.
//!
//! \param pDisplay is a pointer to the driver-specific data for this
//! display driver.
//! \param i_i16lX is the X coordinate of the line.
//! \param i_i16lY1 is the Y coordinate of the start of the line.
//! \param i_i16lY2 is the Y coordinate of the end of the line.
//! \param i_u16Value is the color of the line.
//!
//! This function draws a vertical line on the display.  The coordinates of the
//! line are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
void LineDrawV(const Graphics_Display *pDisplay,
                                          int16_t i_i16lX,
                                          int16_t i_i16lY1,
                                          int16_t i_i16lY2,
                                          uint16_t i_u16Value)
{
    SetDrawFrame(i_i16lX, i_i16lY1, i_i16lX, i_i16lY2);

    // Write the pixel value.

    HAL_LCD_writeCommand(CM_RAMWR);
    for (int i = i_i16lY1; i <= i_i16lY2; i++)
    {
        HAL_LCD_writeData(i_u16Value>>8);
        HAL_LCD_writeData(i_u16Value);
    }
}

//*****************************************************************************
//
//! Fills a rectangle.
//!
//! \param i_pDisplay is a pointer to the driver-specific data for this
//! display driver.
//! \param pRect is a pointer to the structure describing the rectangle.
//! \param ulValue is the color of the rectangle.
//!
//! This function fills a rectangle on the display.  The coordinates of the
//! rectangle are assumed to be within the extents of the display, and the
//! rectangle specification is fully inclusive (in other words, both sXMin and
//! sXMax are drawn, along with sYMin and sYMax).
//!
//! \return None.
//
//*****************************************************************************
void RectFill(const Graphics_Display *i_pDisplay,
                                         const Graphics_Rectangle *i_pRect,
                                         uint16_t i_u16Value)
{
    int16_t l_u16x0 = i_pRect->sXMin;
    int16_t l_u16x1 = i_pRect->sXMax;
    int16_t l_u16y0 = i_pRect->sYMin;
    int16_t l_u16y1 = i_pRect->sYMax;

    SetDrawFrame(l_u16x0, l_u16y0, l_u16x1, l_u16y1);

    // Write the pixel value.

    int16_t l_i16pixels = (l_u16x1 - l_u16x0 + 1) * (l_u16y1 - l_u16y0 + 1);
    HAL_LCD_writeCommand(CM_RAMWR);
    for (int i = 0; i <= l_i16pixels; i++)
    {
        HAL_LCD_writeData(i_u16Value>>8);
        HAL_LCD_writeData(i_u16Value);
    }
}

//*****************************************************************************
//
//! Translates a 24-bit RGB color to a display driver-specific color.
//!
//! \param i_pDisplay is a pointer to the driver-specific data for this
//! display driver.
//! \param i_u32Value is the 24-bit RGB color.  The least-significant byte is the
//! blue channel, the next byte is the green channel, and the third byte is the
//! red channel.
//!
//! This function translates a 24-bit RGB color into a value that can be
//! written into the display's frame buffer in order to reproduce that color,
//! or the closest possible approximation of that color.
//!
//! \return Returns the display-driver specific color.
//
//*****************************************************************************
uint32_t ColorTranslate(const Graphics_Display *i_pDisplay,
                                                   uint32_t i_u32Value)
{
    //
    // Translate from a 24-bit RGB color to a 5-6-5 RGB color.
    //
    return(((((i_u32Value) & 0x00f80000) >> 8) |
            (((i_u32Value) & 0x0000fc00) >> 5) |
            (((i_u32Value) & 0x000000f8) >> 3)));
}

//*****************************************************************************
//
//! Flushes any cached drawing operations.
//!
//! \param i_pDisplay is a pointer to the driver-specific data for this
//! display driver.
//!
//! This functions flushes any cached drawing operations to the display.  This
//! is useful when a local frame buffer is used for drawing operations, and the
//! flush would copy the local frame buffer to the display.  For the SSD2119
//! driver, the flush is a no operation.
//!
//! \return None.
//
//*****************************************************************************
void Flush(const Graphics_Display *i_pDisplay)
{
    //
    // There is nothing to be done.
    //
}

//*****************************************************************************
//
//! Send command to clear screen.
//!
//! \param i_pDisplay is a pointer to the driver-specific data for this
//! display driver.
//!
//! This function does a clear screen and the Display Buffer contents
//! are initialized to the current background color.
//!
//! \return None.
//
//*****************************************************************************
void ClearScreen (const Graphics_Display *i_pDisplay,
                                 uint16_t i_u16Value)
{
    Graphics_Rectangle l_sRect = { 0, 0, LCD_VERTICAL_MAX-1, LCD_VERTICAL_MAX-1};
    RectFill(i_pDisplay, &l_sRect, i_u16Value);
}


//
const Graphics_Display_Functions g_sDisplay_Functions =
{
    PixelDraw,
    PixelDrawMultiple,
    LineDrawH,
    LineDrawV,
    RectFill,
    ColorTranslate,
    Flush,
    ClearScreen
};

//*****************************************************************************
//
//! Provides a small delay.
//!
//! \param ui32Count is the number of delay loop iterations to perform.
//!
//! This function provides a means of generating a delay by executing a simple
//! 3 instruction cycle loop a given number of times.  It is written in
//! assembly to keep the loop instruction count consistent across tool chains.
//!
//! It is important to note that this function does NOT provide an accurate
//! timing mechanism.  Although the delay loop is 3 instruction cycles long,
//! the execution time of the loop will vary dramatically depending upon the
//! application's interrupt environment (the loop will be interrupted unless
//! run with interrupts disabled and this is generally an unwise thing to do)
//! and also the current system clock rate and flash timings (wait states and
//! the operation of the prefetch buffer affect the timing).
//!
//! For best accuracy, a system timer should be used with code either polling
//! for a particular timer value being exceeded or processing the timer
//! interrupt to determine when a particular time period has elapsed.
//!
//! \return None.
//
//*****************************************************************************
#if defined( __ICCARM__ ) || defined(DOXYGEN)
void
SysCtlDelay(uint32_t ui32Count)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   SysCtlDelay\n"
          "    bx      lr");
}
#endif
#if defined(codered) || defined( __GNUC__ ) || defined(sourcerygxx)
void __attribute__((naked))
SysCtlDelay(uint32_t ui32Count)
{
    __asm("    subs    r0, #1\n"
          "    bne     SysCtlDelay\n"
          "    bx      lr");
}
#endif
#if defined(rvmdk) || defined( __CC_ARM )
__asm void
SysCtlDelay(uint32_t ui32Count)
{
    subs    r0, #1;
    bne     SysCtlDelay;
    bx      lr;
}
#endif
