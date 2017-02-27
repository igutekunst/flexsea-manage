/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-manage' Mid-level computing, and networking
	Copyright (C) 2016 Dephy, Inc. <http://dephy.com/>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************
	[Lead developper] Jean-Francois (JF) Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] flexsea_board: configuration and functions for this
	particular board
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-23 | jfduval | Initial GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include <fm_uarts.h>
#include "flexsea_board.h"
#include "../../flexsea-system/inc/flexsea_system.h"
#include <fm_block_allocator.h>
#include <flexsea_comm.h>
#include <stdbool.h>

//****************************************************************************
// Variable(s)
//****************************************************************************

//<FlexSEA User>
//==============
//Board architecture. Has to be changed in all the flexsea_board files!

uint8_t board_id = FLEXSEA_MANAGE_1;		//This board
uint8_t board_up_id = FLEXSEA_PLAN_1;		//This board's master

//Slave bus #1 (RS-485 #1):
//=========================
uint8_t board_sub1_id[SLAVE_BUS_1_CNT] = {FLEXSEA_EXECUTE_1, FLEXSEA_EXECUTE_3};

//Slave bus #2 (RS-485 #2):
//=========================
uint8_t board_sub2_id[SLAVE_BUS_2_CNT] = {FLEXSEA_EXECUTE_2, FLEXSEA_EXECUTE_4};

//(make sure to update SLAVE_BUS_x_CNT in flexsea_board.h!)


//===============
//</FlexSEA User>

//ToDo what is that doing here? Remove
//extern uint8_t rx_command_4[PAYLOAD_BUFFERS][PACKAGED_PAYLOAD_LEN];
//int8_t unpack_payload(uint8_t *buf, uint8_t rx_cmd[][PACKAGED_PAYLOAD_LEN]);


uint8_t bytes_ready_spi = 0;
int8_t cmd_ready_spi = 0;
int8_t cmd_ready_usb = 0;

extern volatile PacketWrapper* fresh_packet;

//****************************************************************************
// Function(s)
//****************************************************************************
//Wrapper for the specific serial functions. Useful to keep flexsea_network
//platform independent (for example, we don't need need puts_rs485() for Plan)
void flexsea_send_serial_slave(PacketWrapper* p)
{
	uint8_t port = p->port;
	uint8_t* str = p->packed;
	size_t length = COMM_STR_BUF_LEN;

	if(port == PORT_RS485_1)
	{
		puts_rs485_1(str, length);
		slaveComm[0].transceiverState = TRANS_STATE_TX_THEN_RX;	//ToDo we do not always want to RX
		log_entry(slaveComm[0].transceiverState);
		slaveComm[0].reply_port = p->reply_port;
	}
	else if(port == PORT_RS485_2)
	{
		puts_rs485_2(str, length);
		slaveComm[1].transceiverState = TRANS_STATE_TX_THEN_RX;	//ToDo we do not always want to RX
		slaveComm[1].reply_port = p->reply_port;
	}
	else
	{
		//Unknown port, call flexsea_error()

		flexsea_error(SE_INVALID_SLAVE);
	}
	fm_pool_free_block(p);
}

void flexsea_send_serial_master(PacketWrapper* p)
{
	Port port = p->port;
	uint8_t *str = p->packed;
	uint8_t length = COMM_STR_BUF_LEN;
	int i = 0;

	if(port == PORT_SPI)
	{
		for(i = 0; i < length; i++)
		{
			comm_str_spi[i] = str[i];
		}
		//This will be sent during the next SPI transaction
	}
	else if(port == PORT_USB)
	{
		CDC_Transmit_FS(str, length);
	}
	else if(port == PORT_WIRELESS)
	{
		puts_expUart(str, length);
	}
	fm_pool_free_block(p);
}

void flexsea_receive_from_master(void)
{
	if (fresh_packet != NULL ) {
		PacketWrapper* p = fresh_packet;
		fresh_packet = NULL;

		cmd_ready_usb = unpack_payload(p->packed, p->unpaked);
		int err = fm_queue_put(&unpacked_packet_queue, p);
		if (err)
			fm_pool_free_block(p);

		PacketWrapper* new_p = fm_pool_allocate_block();
		new_p->port = PORT_USB;
		new_p->reply_port = PORT_USB;

		if (new_p == NULL)
			return; // No more blocks available. Consider reporting up the stack

		USBD_CDC_SetRxBuffer(hUsbDevice_0, new_p->packed);
		USBD_CDC_ReceivePacket(hUsbDevice_0);
	}




}

void flexsea_start_receiving_from_master(void)
{
	// start receive over SPI
	if (HAL_SPI_GetState(&spi4_handle) == HAL_SPI_STATE_READY)
	{
		PacketWrapper* p = fm_pool_allocate_block();
		if (p == NULL)
			return;
		if(HAL_SPI_TransmitReceive_IT(&spi4_handle, (uint8_t *)aTxBuffer, p->packed, COMM_STR_BUF_LEN) != HAL_OK)
		{
			// Transfer error in transmission process
			flexsea_error(SE_RECEIVE_FROM_MASTER);
		}
	}
}

//Receive data from a slave
void flexsea_receive_from_slave(void)
{
	//We only listen if we requested a reply:
	__disable_irq();
	if(slaveComm[0].transceiverState == TRANS_STATE_PREP_RX)
	{
		slaveComm[0].transceiverState = TRANS_STATE_RX;
		log_entry(slaveComm[0].transceiverState);
		__enable_irq();

		reception_rs485_1_blocking();	//Sets the transceiver to Receive
		//From this point on data will be received via the interrupt.
	}
	__enable_irq();

	//We only listen if we requested a reply:
	if(slaveComm[1].transceiverState == TRANS_STATE_PREP_RX)
	{
		slaveComm[1].transceiverState = TRANS_STATE_RX;

		reception_rs485_2_blocking();	//Sets the transceiver to Receive
		//From this point on data will be received via the interrupt.
	}

	//Did we receive new bytes?
	if(slaveComm[0].rx.bytesReady != 0)
	{
		slaveComm[0].rx.bytesReady = 0;
		//Got new data in, try to decode
		slaveComm[0].rx.cmdReady = unpack_payload_485_1();

		if(slaveComm[0].rx.cmdReady < 0)
		{
			slaveComm[1].transceiverState = TRANS_STATE_RX;
		}

	}

	//Did we receive new bytes?
	if(slaveComm[1].rx.bytesReady != 0)
	{
		slaveComm[1].rx.bytesReady = 0;
		//Got new data in, try to decode
		slaveComm[1].rx.cmdReady = unpack_payload_485_2();
	}
}

//Copies the generated comm_str to the aTxBuffer
//It will be transfered the next time the master writes to us.
//ToDo generalize, use buffers as arguments
void comm_str_to_txbuffer(void)
{
	uint8_t i = 0;

	for(i = 0; i < COMM_STR_BUF_LEN; i++)
	{
		aTxBuffer[i] = comm_str_spi[i];
	}
}
