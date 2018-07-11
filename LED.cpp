#include "LED.hpp"

LED::LED(uint16_t i_BITN)
{
	m_u16BITN = i_BITN;
}

uint8_t LED::run()
{
    //Send message
    st_Message l_st_SendMessage;

    l_st_SendMessage.u8Sender = this->m_u8TaskID;
    l_st_SendMessage.u8Receiver = this->m_u8TaskID;
    l_st_SendMessage.u32Content ^= m_u16BITN;

    this->m_pMailbox->SendMessage(l_st_SendMessage);

    //Receive message
    st_Message * l_st_ReceiveMessage;
    l_st_ReceiveMessage=this->m_pMailbox->GetMessage(this->m_u8TaskID);

    uint16_t l_u16Data=l_st_ReceiveMessage->u32Content;


    //#########################
    // Blink code Assuming PORT2
	P2->OUT = l_u16Data;
    //#########################
    return(NO_ERR);
}

uint8_t LED::setup()
{
    //LED Setup, assuming PORT2
    // - P2.0 is connected to the RGB LED
    P2->DIR |= m_u16BITN; //Red LED
    P2->OUT &= m_u16BITN; // Initialize the LED Value
    return(NO_ERR);
}
