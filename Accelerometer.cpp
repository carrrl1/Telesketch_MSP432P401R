#include "Accelerometer.hpp"

Accelerometer::Accelerometer(){}

uint8_t Accelerometer::run()
{
    //Receive message
    
    if (false)
    {
        st_Message * l_st_ReceiveMessage;
        l_st_ReceiveMessage=this->m_pMailbox->GetMessage(this->m_u8TaskID);

        uint32_t l_u32Data=l_st_ReceiveMessage->u32Content;

        int16_t l_u16Y = (int16_t)(l_u32Data>>16);
        int16_t l_u16Z = (int16_t)l_u32Data;
        float l_fX = 8500;
        float l_fY = (float)(l_u16Y);
        float l_fZ = (float)(l_u16Z);
    }

    /* Store ADC14 conversion results */
    float l_fX=(float)ADC14_getResult(ADC_MEM0);
    float l_fY=(float)ADC14_getResult(ADC_MEM1);
    float l_fZ=(float)ADC14_getResult(ADC_MEM2);


    //Calculate the pitch angle from the accelerometer data
    int32_t l_i32Pitch = (int32_t)(atan2(-(l_fX),sqrt(l_fZ*l_fZ + l_fY*l_fY)) * 57.3);  

    //Send message to the LCD task
    st_Message l_st_SendMessage;

    l_st_SendMessage.u8Sender = this->m_u8TaskID;
    l_st_SendMessage.u8Receiver = this->m_u8LinkedTaskID;
    l_st_SendMessage.u32Content = l_i32Pitch;

    this->m_pMailbox->SendMessage(l_st_SendMessage);

    return(NO_ERR);
}

uint8_t Accelerometer::setup()
{
    //Accelerometer Setup
        /* Configures Pin 4.0, 4.2, and 6.1 as ADC input */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initializing ADC (ADCOSC/64/8) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8,
            0);

    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM2 (A11, A13, A14)  with no repeat)
         * with internal 2.5v reference */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A14, ADC_NONDIFFERENTIAL_INPUTS);

    MAP_ADC14_configureConversionMemory(ADC_MEM1,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);

    MAP_ADC14_configureConversionMemory(ADC_MEM2,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A11, ADC_NONDIFFERENTIAL_INPUTS);

    /* Enabling the interrupt when a conversion on channel 2 (end of sequence)
     *  is complete and enabling conversions */
    MAP_ADC14_enableInterrupt(ADC_INT2);

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
