#include <stdlib.h>
#include <string.h>

#ifdef DEBUG
#include <stdio.h>
#endif

#include "can.h"
#include "can-uart.h"
#include "can-encap.h"
#include "uart-host.h"
#include "crc16.h"

/*****************************************************************************
 * Global variables
 */

typedef enum {STATE_START, STATE_LEN, STATE_PAYLOAD, STATE_CRC} canu_rcvstate_t;

rs232can_msg	canu_rcvpkt;
canu_rcvstate_t	canu_rcvstate = STATE_START;
unsigned char 	canu_rcvlen   = 0;


/*****************************************************************************
 * Connection management
 */

// Initialize CAN over UART on port serial
void canu_init(char *serial)
{
	uart_init(serial);
	canu_reset();
}

// syncronize line
void canu_reset()
{
	unsigned char i;
	for(i=sizeof(rs232can_msg)+2; i>0; i--)
		uart_putc( (char)0x00 );
}

/*****************************************************************************
 * Memory Management
 */

rs232can_msg *canu_buffer_get()
{
	return (rs232can_msg *)malloc( sizeof(rs232can_msg) );
}

void canu_free(rs232can_msg *rmsg)
{
	if (rmsg != &canu_rcvpkt)
		free(rmsg);
}

/*****************************************************************************
 * rcv
 */

// Returns Message or 0 if there is no complete message.
rs232can_msg * canu_get_nb()
{
	static char *uartpkt_data;
	static unsigned int	crc;
	unsigned char c;

	while (uart_getc_nb(&c))
	{
		#ifdef DEBUG
		printf("canu_get_nb received: %02x\n", c);
		#endif
		
		switch (canu_rcvstate)
		{
			case STATE_START:
				if (c)
				{
					canu_rcvstate = STATE_LEN;
					canu_rcvpkt.cmd = c;
				}
				break;
			case STATE_LEN:
				canu_rcvlen       = c;
				if(canu_rcvlen > RS232CAN_MAXLENGTH)
				{
					canu_rcvstate = STATE_START;
					break;
				}
				canu_rcvstate     = STATE_PAYLOAD;
				canu_rcvpkt.len   = c;
				uartpkt_data      = &canu_rcvpkt.data[0];
				break;
			case STATE_PAYLOAD:
				if(canu_rcvlen--)
					*(uartpkt_data++) = c;
				else
				{
					canu_rcvstate = STATE_CRC;
					crc = c;
				}
				break;
			case STATE_CRC:
				canu_rcvstate = STATE_START;
				crc = (crc << 8) | c;
				
				#ifdef DEBUG
				printf("canu_get_nb crc: 0x%04x, 0x%04x\n", crc, crc16(&canu_rcvpkt.cmd, canu_rcvpkt.len + 2);
				#endif
				
				if(crc == crc16(&canu_rcvpkt.cmd, canu_rcvpkt.len + 2))
					return &canu_rcvpkt;

				break;
		}
	}

	return NULL;
}

rs232can_msg * canu_get(){
	int ret;
	fd_set rset;
	rs232can_msg *rmsg;

	for(;;) {
		FD_ZERO(&rset);
		FD_SET(uart_fd, &rset);

		ret = select(uart_fd + 1, &rset, (fd_set*)NULL, (fd_set*)NULL, NULL);
		debug_assert( ret >= 0, "canu_get: select failed" );

		rmsg = canu_get_nb();
		if (rmsg)
			return rmsg;
	}
}


/*****************************************************************************
 * Transmit
 */
void canu_transmit(rs232can_msg *msg)
{
	char *ptr = (char *)msg;
	unsigned char i;
	unsigned int crc = crc16(&msg->cmd, msg->len + 2);

	for (i=0; i<msg->len+2; i++) {
		uart_putc( *ptr++);
	}

	uart_putc(crc >> 8);
	uart_putc(crc & 0xFF);
}


