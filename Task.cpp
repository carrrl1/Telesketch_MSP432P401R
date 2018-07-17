#include "Task.hpp"

Task::Task()
{
    m_u8TaskID = m_u8NextTaskID;
    m_u8NextTaskID++;
    m_bIsFinished = false;
}

uint8_t Task::run(void){
    return(0);
}

uint8_t Task::setup(void){
    return(0);
}

uint32_t Task::ReceiveMessage(void) {
    //Receive message
    st_Message * l_st_ReceiveMessage;
    l_st_ReceiveMessage=this->m_pMailbox->GetMessage(this->m_u8TaskID);
    return l_st_ReceiveMessage->u32Content;
}

void Task::SendMessage(uint32_t i_u32Data) {
    //Send message to the linked task
    st_Message l_st_SendMessage;

    l_st_SendMessage.u8Sender = this->m_u8TaskID;
    l_st_SendMessage.u8Receiver = this->m_u8LinkedTaskID;
    l_st_SendMessage.u32Content = i_u32Data;

    this->m_pMailbox->SendMessage(l_st_SendMessage);
}
