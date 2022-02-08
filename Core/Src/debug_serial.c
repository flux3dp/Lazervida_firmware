#include "debug_serial.h"
#include "main.h"
#include "grbl.h"
#include "lwrb.h"

#define DEBUG_RX_RING_BUFFER (128+1)
//#define TX_RING_BUFFER (TX_BUFFER_SIZE+1)
uint8_t debug_serial_rx_buffer[DEBUG_RX_RING_BUFFER];
uint8_t debug_serial_rx_buffer_head = 0;
volatile uint8_t debug_serial_rx_buffer_tail = 0;

//static size_t usart_tx_dma_current_len; // The current DMA transmitting data len
//static lwrb_t usart_tx_dma_ringbuff; // handle for usart tx
//static uint8_t serial_tx_buffer[128]; // usart tx buffer (Data be appended by main program, and then be read and sent by DMA)


void debug_serial_init() {
  MX_USART1_UART_Init();
  NVIC_DisableIRQ(USART1_IRQn);
  /* === Tx send without interrupt (blocking send) === */

  /* ====================== Rx interrupt setup ====================== */
  LL_USART_EnableIT_RXNE(USART1);
  NVIC_EnableIRQ(USART1_IRQn);
}

void debug_serial_rx_handler() {
  uint8_t data;
  uint16_t next_head;

  data = (USART1->DR) & 0x1FF;

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
        next_head = debug_serial_rx_buffer_head + 1;
        if (next_head == DEBUG_RX_RING_BUFFER) { next_head = 0; }

        // Write data to buffer unless it is full.
        if (next_head != debug_serial_rx_buffer_tail) {
          debug_serial_rx_buffer[debug_serial_rx_buffer_head] = data;
          debug_serial_rx_buffer_head = next_head;
        }
      }
  }

}

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t debug_serial_read()
{
  uint16_t tail = debug_serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (debug_serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = debug_serial_rx_buffer[tail];

    tail++;
    if (tail == DEBUG_RX_RING_BUFFER) { tail = 0; }
    debug_serial_rx_buffer_tail = tail;

    return data;
  }
}

void debug_serial_write_data(uint8_t data) {
  LL_USART_TransmitData8(USART1, data);
  // Blocking send
	while (!LL_USART_IsActiveFlag_TXE(USART1));
    return;
}

// NOTE: To print debug message -> call printf(...)

/* For printf in <stdio.h> */
/*
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 1000);
  return ch;
}
int _write(int file,char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 1000);
  return len;
}
*/