#include "Joystick.hpp"
#include <stdio.h>

Joystick::Joystick(){}

uint8_t Joystick::run()
{
    //Init local variables
    float l_fX=0;
    float l_fY=0;
    uint32_t l_u32Direction=0;

    /* Store ADC14 conversion results */
    l_fX = (double)ADC14_getResult(ADC_MEM0);
    l_fY = (double)ADC14_getResult(ADC_MEM1);

    //Normalize the vector
    l_fX = l_fX/MAX_MAGNITUDE;
    l_fY = l_fY/MAX_MAGNITUDE;

    if(l_fX >= 1) l_fX=1;
    if(l_fY >= 1) l_fY=1;

    /* Debug
    char string[20];
    sprintf(string, "X: %1.3f Y: %1.3f \n", l_fX ,l_fY);
    printf(string);
    fflush (stdout);
    */


    //Calculate the pixel direction
    /*****************************
     *          X           Y
     * N    SL TO  SH    SH TO  1
     * W     0 TO  SL    SL TO  SH
     * S    SL TO  SH     0 TO  SL
     * E    SH TO  1     SL TO  SH
     * NW    0 TO  SL    SH TO  1
     * SW    0 TO  SL     0 TO SL
     * NE   SH TO  1     SH TO  1
     * SE   SH TO  1      0 TO SL
     *****************************/

    if ( (SL <= l_fX && l_fX <= SH) && (SH <= l_fY && l_fY <= 1)         ) {
        l_u32Direction = D_N;
        goto send;

    } else if ( (0 <= l_fX && l_fX <= SL)  && (SL <= l_fY && l_fY <= SH) ) {
        l_u32Direction = D_W;
        goto send;

    } else if ( (SL <= l_fX && l_fX <= SH) && (0 <= l_fY && l_fY <= SL)  ) {
        l_u32Direction = D_S;
        goto send;

    } else if ( (SH <= l_fX && l_fX <= 1)  && (SL <= l_fY && l_fY <= SH) ) {
        l_u32Direction = D_E;
        goto send;

    } else if ( (0 <= l_fX && l_fX <= SL)  && (SH <= l_fY && l_fY <= 1)  ) {
        l_u32Direction = D_NW;
        goto send;

    } else if ( (0 <= l_fX && l_fX <= SL)  && (0 <= l_fY && l_fY <= SL)  ) {
        l_u32Direction = D_SW;
        goto send;

    } else if ( (SH <= l_fX && l_fX <= 1)  && (SH <= l_fY && l_fY <= 1)  ) {
        l_u32Direction = D_NE;
        goto send;

    } else if ( (SH <= l_fX && l_fX <= 1)  && (0 <= l_fY && l_fY <= SL)  ) {
        l_u32Direction = D_SE;
        goto send;

    } else l_u32Direction = D_C;

    //Send message to the LCD task
    send:
    st_Message l_st_SendMessage;

    l_st_SendMessage.u8Sender = this->m_u8TaskID;
    l_st_SendMessage.u8Receiver = this->m_u8LinkedTaskID;
    l_st_SendMessage.u32Content = l_u32Direction;

    this->m_pMailbox->SendMessage(l_st_SendMessage);

    return(NO_ERR);
}

uint8_t Joystick::setup()
{
    /* Configures Pin 6.0 and 4.4 as ADC input */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initializing ADC (ADCOSC/64/8) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat)
         * with internal 2.5v reference */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A15, ADC_NONDIFFERENTIAL_INPUTS);

    MAP_ADC14_configureConversionMemory(ADC_MEM1,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A9, ADC_NONDIFFERENTIAL_INPUTS);

    /* Enabling the interrupt when a conversion on channel 1 (end of sequence)
     *  is complete and enabling conversions */
    MAP_ADC14_enableInterrupt(ADC_INT1);

    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();

    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Triggering the start of the sample */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();

    return(NO_ERR);
}
