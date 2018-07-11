#include "Mailbox.hpp"

// - Mailbox constructor
Mailbox::Mailbox()
{
    
    for(int index = 0; index < MAILBOX_SLOTS; index++)
    {
        m_aInbox[index].bReaded = true; // Init all messages readed
    }
    return;
    
}
// - The send function, inserts a message into the inbox slots.
uint8_t Mailbox::SendMessage(st_Message i_stMessage)
{
    uint8_t l_ErrorCode = NO_ERR;

    uint8_t l_u8Receiver = i_stMessage.u8Receiver;

    bool l_bReaded = m_aInbox[l_u8Receiver].bReaded;
    if(l_bReaded)
    {   
        m_aInbox[l_u8Receiver] =  i_stMessage;
        l_ErrorCode = NO_ERR;
    }
    else
    {
        l_ErrorCode = RET_ERR;
    }
    return l_ErrorCode;
}

// - The get function, get a message from the inbox slots usign the receiver ID.
st_Message * Mailbox::GetMessage(uint8_t i_u8Receiver)
{
    m_aInbox[i_u8Receiver].bReaded=true;
    return &m_aInbox[i_u8Receiver];
}

// - The get function, get a message from the inbox slots usign the receiver ID.
void Mailbox::DeleteMessage(uint8_t i_u8Receiver)
{
    //delete m_aInbox[i_u8Receiver];
    //m_aInbox[i_u8Receiver] = (uintptr_t) 0; // Point to an invalid pointer
}
