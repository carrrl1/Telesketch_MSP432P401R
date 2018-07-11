/*
 * Mailbox.hpp
 *
 *  Created on: 
 *      Author: cesolano
 */

#ifndef MAILBOX_HPP_
#define MAILBOX_HPP_
#include "msp.h"

#define MAILBOX_SLOTS 255
#define NULL            0

#define NO_ERR 0
#define RET_ERR 1

// - This structure defines the Message Information
struct st_Message {
    uint8_t    u8Sender;
    uint8_t    u8Receiver;
	uint32_t   u32Content;
    bool       bReaded=false;
};

class Mailbox
{
	public:
	    Mailbox();
	    st_Message *    GetMessage(uint8_t i_u8Receiver); // - Get the message form the mailbox.
	    void		    DeleteMessage(uint8_t i_u8Receiver); // - Delete the message form the mailbox.
	    uint8_t         SendMessage(st_Message i_stMessage); // - Send a message (stores it into the inbox)
	private:
	    st_Message m_aInbox[MAILBOX_SLOTS]; // - Inbox
};



#endif /* MAILBOX_HPP_ */
