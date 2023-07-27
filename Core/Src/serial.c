/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"
#include "debug_serial.h"
#include "stm32f1xx_hal_usart.h"
#include "usbd_cdc_if.h"

#define RX_RING_BUFFER (RX_BUFFER_SIZE+1)
//#define TX_RING_BUFFER (TX_BUFFER_SIZE+1)

uint8_t serial_rx_buffer[RX_RING_BUFFER];
uint8_t serial_rx_buffer_head = 0;
volatile uint8_t serial_rx_buffer_tail = 0;

extern volatile uint8_t host_com_port_open;
//uint8_t serial_tx_buffer[TX_RING_BUFFER];
//uint8_t serial_tx_buffer_head = 0;
//volatile uint8_t serial_tx_buffer_tail = 0;


// Returns the number of bytes available in the RX serial buffer.
uint8_t serial_get_rx_buffer_available()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (serial_rx_buffer_head-rtail)); }
  return((rtail-serial_rx_buffer_head-1));
}


void serial_init()
{
  /*
  // Set baud rate
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
  #endif
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;

  // enable rx, tx, and interrupt on complete reception of a byte
  UCSR0B |= (1<<RXEN0 | 1<<TXEN0 | 1<<RXCIE0);
  
  // defaults to 8-bit, no parity, 1 stop bit
  */

  // TODO: USB CDC init

}


// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data) {
  // Temporary: delegate to debug serial port
  //debug_serial_write_data(data);
  
  // Actual: USB CDC
  uint8_t ret = USBD_BUSY;
  uint32_t ts = millis();
  while (ret == USBD_BUSY) {
    if (host_com_port_open == 0) {
      ret = USBD_FAIL;
      return;
    }
    // A timeout is needed:
    //   When port is open without DTR/RTS signal, 
    //   there is no robust way to ensure the port is open/close on host
    if (millis() - ts > 500) {
      host_com_port_open = 0;
      return;
    }
    ret = CDC_Transmit_FS(&data, 1);
  }
}
/*
void serial_write(uint8_t data) {
  // Calculate next head
  uint8_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_RING_BUFFER) { next_head = 0; }

  // Wait until there is space in the buffer
  while (next_head == serial_tx_buffer_tail) {
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;

  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  UCSR0B |=  (1 << UDRIE0);
}


// Data Register Empty Interrupt handler
ISR(SERIAL_UDRE)
{
  uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)

  // Send a byte from the buffer
  UDR0 = serial_tx_buffer[tail];

  // Update tail position
  tail++;
  if (tail == TX_RING_BUFFER) { tail = 0; }

  serial_tx_buffer_tail = tail;

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == serial_tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}
*/

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{ 
  // Delegate to debug port for test without USB only
  //return debug_serial_read();
  
  uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial_rx_buffer[tail];

    tail++;
    if (tail == RX_RING_BUFFER) { tail = 0; }
    serial_rx_buffer_tail = tail;

    return data;
  }
}

/**
 * @brief Callback in USB CDC interrupt
 */
void serial_handle_data(uint8_t data) 
{
  //uint8_t data = UDR0;
  uint8_t next_head;
  host_com_port_open = 1; // Just in case, not needed typically

  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the main buffer, but these set system state flag bits for realtime execution.
  switch (data) {
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
    case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // Set as true
    case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // Set as true
    case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // Set as true
    default :
      if (data > 0x7F) { // Real-time control characters are extended ACSII only.
        switch(data) {
          case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // Set as true
          case CMD_JOG_CANCEL:
            if (sys.state & STATE_JOG) { // Block all other states from invoking motion cancel.
              system_set_exec_state_flag(EXEC_MOTION_CANCEL); 
            }
            break; 
          #ifdef DEBUG
            case CMD_DEBUG_REPORT: {uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;} break;
          #endif
          case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
          case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
          case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
          case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
          case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
          case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
          case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
          case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
          case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
          case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
          case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
          case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
          case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
          case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
          case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
          #ifdef ENABLE_M7
            case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
          #endif
        }
        // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
      } else { // Write character to buffer
        next_head = serial_rx_buffer_head + 1;
        if (next_head == RX_RING_BUFFER) { next_head = 0; }

        // Write data to buffer unless it is full.
        if (next_head != serial_rx_buffer_tail) {
          serial_rx_buffer[serial_rx_buffer_head] = data;
          serial_rx_buffer_head = next_head;
        }
      }
  }
}


void serial_reset_read_buffer()
{
  serial_rx_buffer_tail = serial_rx_buffer_head;
}


uint16_t calculate_CRC(uint8_t *frame, uint8_t bufferSize) 
{
    uint16_t temp, temp2, flag;
    temp = 0xFFFF;
    for (uint8_t i = 0; i < bufferSize; i++)
    {
        temp = temp ^ frame[i];
        for (uint8_t j = 1; j <= 8; j++)
        {
            flag = temp & 0x0001;
            temp >>= 1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;

    return temp;
}

void usart3_write(uint8_t data) {
  LL_USART_TransmitData8(USART3, data);
  // Blocking send
	while (!LL_USART_IsActiveFlag_TXE(USART3));
    return;
}

void modbus_query(void)
{
    uint8_t query[8] = {0};

    query[0] = 0x02; // Device address
    query[1] = 0x03; // Function code: Read Holding Registers
    query[2] = 0x00; // Starting Address High
    query[3] = 0x02; // Starting Address Low (now set to 0x02)
    query[4] = 0x00; // Quantity High
    query[5] = 0x01; // Quantity Low

    uint16_t crc = calculate_CRC(query, 6); // Calculate crc

    query[6] = crc & 0xFF; // crc Low
    query[7] = (crc >> 8) & 0xFF; // crc High

    // Write query
    for(int i = 0; i < 8; i++)
      usart3_write(query[i]);
}

uint8_t rx3_data;
uint8_t rx3_buffer[256];
uint8_t rx3_index = 0;

void usart3_rx_handler(USART_HandleTypeDef *huart)
{
  rx3_buffer[rx3_index++] = rx3_data;
  if (rx3_index >= 9) {
    printString("Received: ");
    printInteger((int)(rx3_buffer[6]));
    printString("\r\n");
  }
}
