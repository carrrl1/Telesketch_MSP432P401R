#ifndef TASK_HPP_
#define TASK_HPP_

#include "msp.h"
#include "Mailbox.hpp"

#define NO_ERR 0
#define RET_ERR 1

class Task
{
	public:
		Task();
		uint8_t m_u8TaskID;
		virtual uint8_t     run(void);
		virtual uint8_t     setup(void);
		bool                IsTaskFinished(void){return m_bIsFinished;};
		uint8_t             GetTaskPriority(void) {return m_u8Priority;};
		void                SetTaskPriority(uint8_t i_u8NewPriority){m_u8Priority = i_u8NewPriority;};
		void                SetLinkedTask(uint8_t i_u8LinkedTaskID){m_u8LinkedTaskID = i_u8LinkedTaskID;};
		uint32_t            ReceiveMessage(void);
		void                SendMessage(uint32_t i_u32Data);
		static Mailbox * m_pMailbox; // - Pointer to the main Mailbox
	private:
	   static uint8_t m_u8NextTaskID;
	   uint8_t m_u8Priority;
	protected:
	   bool m_bIsFinished;
	   uint8_t m_u8LinkedTaskID;
};

#endif /* ASK_HPP_ */
