#include "LCD.hpp"
#include "Joystick.hpp"

LCD::LCD()
{
    //! The display structure that describes the driver for the Kitronix
    //! K350QVG-V1-F TFT panel with an SSD2119 controller.
    m_sDisplay =
    {
        sizeof(Graphics_Display),
        0,
        LCD_VERTICAL_MAX,
        LCD_HORIZONTAL_MAX,
    };
    
    //Set pen size and color
    m_u32PenColor = BLACK_PEN;
    m_u8PenSize = 1;

    //Set Background
    m_u32Background = WHITE_BACKGROUND;

    //Set pen position
    m_u8PenX = DISPLAY_MID;
    m_u8PenY = DISPLAY_MID;

    //Set sky, earth and line rectangles coordinates
    SetPenRectangle(m_u8PenX,
                    m_u8PenX+m_u8PenSize,
                    m_u8PenY,
                    m_u8PenY+m_u8PenSize);

}

uint8_t LCD::run()
{


    //Receive message
    st_Message * l_st_ReceiveMessage;
    l_st_ReceiveMessage=this->m_pMailbox->GetMessage(this->m_u8TaskID);
    uint32_t l_u32Data=l_st_ReceiveMessage->u32Content;

    SetPenLocation(l_u32Data);
    Graphics_fillRectangle(&m_sContext, &m_sPen);

    return(NO_ERR);
}

uint8_t LCD::setup()
{
    //LCD setup
    MAP_Interrupt_disableMaster();

    // Set up SPI Ports
    // LCD_SCK
    GPIO_setAsPeripheralModuleFunctionOutputPin(LCD_SCK_PORT, LCD_SCK_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    // LCD_MOSI
    GPIO_setAsPeripheralModuleFunctionOutputPin(LCD_MOSI_PORT, LCD_MOSI_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    // LCD_RST
    GPIO_setAsOutputPin(LCD_RST_PORT, LCD_RST_PIN);
    // LCD_RS
    GPIO_setAsOutputPin(LCD_DC_PORT, LCD_DC_PIN);
    // LCD_CS
    GPIO_setAsOutputPin(LCD_CS_PORT, LCD_CS_PIN);

    // Set up SPI 
    eUSCI_SPI_MasterConfig config =
    {
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,
        LCD_SYSTEM_CLOCK_SPEED,
        LCD_SPI_CLOCK_SPEED,
        EUSCI_B_SPI_MSB_FIRST,
        EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW,
        EUSCI_B_SPI_3PIN
    };
    SPI_initMaster(LCD_EUSCI_BASE, &config);
    SPI_enableModule(LCD_EUSCI_BASE);

    GPIO_setOutputLowOnPin(LCD_CS_PORT, LCD_CS_PIN);

    GPIO_setOutputHighOnPin(LCD_DC_PORT, LCD_DC_PIN);

    //Write first data

    GPIO_setOutputLowOnPin(LCD_RST_PORT, LCD_RST_PIN);
    HAL_LCD_delay(50);
    GPIO_setOutputHighOnPin(LCD_RST_PORT, LCD_RST_PIN);
    HAL_LCD_delay(120);

    HAL_LCD_writeCommand(CM_SLPOUT);
    HAL_LCD_delay(200);

    HAL_LCD_writeCommand(CM_GAMSET);
    HAL_LCD_writeData(0x04);

    HAL_LCD_writeCommand(CM_SETPWCTR);
    HAL_LCD_writeData(0x0A);
    HAL_LCD_writeData(0x14);

    HAL_LCD_writeCommand(CM_SETSTBA);
    HAL_LCD_writeData(0x0A);
    HAL_LCD_writeData(0x00);

    HAL_LCD_writeCommand(CM_COLMOD);
    HAL_LCD_writeData(0x05);
    HAL_LCD_delay(10);

    HAL_LCD_writeCommand(CM_MADCTL);
    HAL_LCD_writeData(CM_MADCTL_BGR);

    HAL_LCD_writeCommand(CM_NORON);

    SetDrawFrame(0, 0, 127, 127);
    HAL_LCD_writeCommand(CM_RAMWR);
    int i;
    for (i = 0; i < 16384; i++)
    {
        HAL_LCD_writeData(0xFF);
        HAL_LCD_writeData(0xFF);
    }

    HAL_LCD_delay(10);
    HAL_LCD_writeCommand(CM_DISPON);


    //Set the orientation
    SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&m_sContext, &m_sDisplay, &g_sDisplay_Functions);

    //Draw the background
    Graphics_setBackgroundColor(&m_sContext, m_u32Background);
    Graphics_clearDisplay(&m_sContext);

    Graphics_setForegroundColor(&m_sContext, m_u32PenColor);
    GrContextFontSet(&m_sContext, &g_sFontFixed6x8);
    Graphics_drawStringCentered(&m_sContext,
                                    (int8_t *)"Telesketch",
                                    AUTO_STRING_LENGTH,
                                    64,
                                    30,
                                    OPAQUE_TEXT);


    //drawTitle();
    MAP_Interrupt_enableMaster();

    return(NO_ERR);
}

void LCD::SetPenRectangle(uint8_t i_u8Min, uint8_t i_u8xMax, uint8_t i_u8yMin, uint8_t i_u8yMax)
{
    m_sPen.xMin = i_u8Min;
    m_sPen.xMax = i_u8xMax;
    m_sPen.yMin = i_u8yMin;
    m_sPen.yMax = i_u8yMax;
}

void LCD::SetPenLocation(uint8_t i_u8PenX, uint8_t i_u8PenY)
{
    //Set pen position
    if(i_u8PenX <= 127 && i_u8PenY <= 127) {
        m_u8PenX = i_u8PenX;
        m_u8PenY = i_u8PenY;
    }
}

void LCD::MovePen()
{
    //Draw pen position
    SetPenRectangle(m_u8PenX,
                    m_u8PenX+m_u8PenSize,
                    m_u8PenY,
                    m_u8PenY+m_u8PenSize);
}

void LCD::SetPenLocation(uint32_t i_u32Direction) {
    switch (i_u32Direction) {
    case D_N:
        SetPenLocation(m_u8PenX, m_u8PenY-m_u8PenSize);
        break;
    case D_W:
        SetPenLocation(m_u8PenX-m_u8PenSize, m_u8PenY);
        break;
    case D_S:
        SetPenLocation(m_u8PenX, m_u8PenY+m_u8PenSize);
        break;
    case D_E:
        SetPenLocation(m_u8PenX+m_u8PenSize, m_u8PenY);
        break;
    case D_NW:
        SetPenLocation(m_u8PenX-m_u8PenSize, m_u8PenY-m_u8PenSize);
        break;
    case D_SW:
        SetPenLocation(m_u8PenX-m_u8PenSize, m_u8PenY+m_u8PenSize);
        break;
    case D_NE:
        SetPenLocation(m_u8PenX+m_u8PenSize, m_u8PenY-m_u8PenSize);
        break;
    case D_SE:
        SetPenLocation(m_u8PenX+m_u8PenSize, m_u8PenY+m_u8PenSize);
        break;
    case D_C:
        break;
    default:
        break;
    }
    //Draw pen
    MovePen();
}

void LCD::ChangePenColor()
{
    //Change pen color
    m_u8CurrentPenColor++;
    if(m_u8CurrentPenColor==PEN_COLORS) m_u8CurrentPenColor=0;
    m_u32PenColor=m_u32PenColors[m_u8CurrentPenColor];
    Graphics_setForegroundColor(&m_sContext, m_u32PenColor);
}

void LCD::ChangePenSize()
{
    //Change pen color
    m_u8PenSize++;
    if(m_u8PenSize<MAX_PEN_SIZE) m_u8PenSize=0;
}

void LCD::ChangeBackgroundColor()
{
    //Change pen color
    m_u8CurrentBackgroundColor++;
    if(m_u8CurrentBackgroundColor==BACKGROUND_COLORS) m_u8CurrentBackgroundColor=0;
    m_u32Background=m_u32BackgroundColors[m_u8CurrentBackgroundColor];
    //Draw the background
    Graphics_setBackgroundColor(&m_sContext, m_u32Background);
    Graphics_clearDisplay(&m_sContext);
}

void LCD::ClearDisplay()
{
    //Clear the display with the background color
    Graphics_setBackgroundColor(&m_sContext, m_u32Background);
    Graphics_clearDisplay(&m_sContext);
}
