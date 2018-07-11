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
