/*
 * CanAPI.c
 *
 * Created: 15/03/2018 03:04:23
 *  Author: Piotr
 * NOTE: In order for CAN to work, import standard CAN driver from ASF
 */ 
#include <asf.h>
#include <string.h>
#include <conf_can.h>

#include "CanAPI.h"

//! [functions]
static void can_set_standard_filter_0(void);
//! [functions]

//! [setup]
// module inst
struct can_module can_instance;

// can filter settings
#define CAN_RX_STANDARD_FILTER_INDEX_0    0
#define CAN_RX_STANDARD_FILTER_ID_0     0x45A
#define CAN_RX_STANDARD_FILTER_ID_0_BUFFER_INDEX     2

// can tx settings
#define CAN_TX_BUFFER_INDEX    0
static uint8_t tx_message_0[CONF_CAN_ELEMENT_DATA_SIZE];
static uint8_t tx_message_1[CONF_CAN_ELEMENT_DATA_SIZE];

// can rx settings
static volatile uint32_t standard_receive_index = 0;
static struct can_rx_element_fifo_0 rx_element_fifo_0;
void (*rx_callback)(uint8_t*, uint8_t);

//! [setup]

//! [can_init_setup]
void configure_can(void) 
{
  uint32_t i;
  /* Initialize the memory. */
  for (i = 0; i < CONF_CAN_ELEMENT_DATA_SIZE; i++) {
    tx_message_0[i] = i;
    tx_message_1[i] = i + 0x80;
  }

  /* Set up the CAN TX/RX pins */
  struct system_pinmux_config pin_config;
  system_pinmux_get_config_defaults(&pin_config);
  pin_config.mux_position = CAN_TX_MUX_SETTING;
  system_pinmux_pin_set_config(CAN_TX_PIN, &pin_config);
  pin_config.mux_position = CAN_RX_MUX_SETTING;
  system_pinmux_pin_set_config(CAN_RX_PIN, &pin_config);

  /* Initialize the module. */
  struct can_config config_can;
  can_get_config_defaults(&config_can);
  can_init(&can_instance, CAN_MODULE, &config_can);

  can_start(&can_instance);

  /* Enable interrupts for this CAN module */
  system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_CAN0);
  can_enable_interrupt(&can_instance, CAN_PROTOCOL_ERROR_ARBITRATION
  | CAN_PROTOCOL_ERROR_DATA);
  
  /* Set filter */
  can_set_standard_filter_0();
}
//! [can_init_setup]

// can RX filter setup
static void can_set_standard_filter_0(void) 
{
  struct can_standard_message_filter_element sd_filter;

  can_get_standard_message_filter_element_default(&sd_filter);
  sd_filter.S0.bit.SFID1 = CAN_RX_STANDARD_FILTER_ID_0;

  can_set_rx_standard_filter(&can_instance, &sd_filter, CAN_RX_STANDARD_FILTER_INDEX_0);
  can_enable_interrupt(&can_instance, CAN_RX_FIFO_0_NEW_MESSAGE);
}

// can TX message
void can_send_standard_message(uint32_t id_value, uint8_t *data, uint32_t data_length) 
{
  uint32_t i;
  struct can_tx_element tx_element;

  can_get_tx_buffer_element_defaults(&tx_element);
  tx_element.T0.reg |= CAN_TX_ELEMENT_T0_STANDARD_ID(id_value);
  tx_element.T1.bit.DLC = data_length;
  for (i = 0; i < data_length; i++) {
    tx_element.data[i] = *data;
    data++;
  }

  can_set_tx_buffer_element(&can_instance, &tx_element,
  CAN_TX_BUFFER_INDEX);
  can_tx_transfer_request(&can_instance, 1 << CAN_TX_BUFFER_INDEX);
}

/**
 * \brief Register callback on message receive
 *
 * \param rx_callback function taking (uint8_t* buff, uint8_t size)
 */
void can_register_callback(void (*rx_handler)(uint8_t*, uint8_t)) 
{
  rx_callback = rx_handler;
}

//! [can_interrupt_handler]
void CAN0_Handler(void) 
{
  volatile uint32_t status, i;
  status = can_read_interrupt_status(&can_instance);

  if (status & CAN_RX_FIFO_0_NEW_MESSAGE) {
    can_clear_interrupt_status(&can_instance, CAN_RX_FIFO_0_NEW_MESSAGE);
    can_get_rx_fifo_0_element(&can_instance, &rx_element_fifo_0, standard_receive_index);
    can_rx_fifo_acknowledge(&can_instance, 0, standard_receive_index);
    standard_receive_index++;
    if (standard_receive_index == CONF_CAN0_RX_FIFO_0_NUM) {
      standard_receive_index = 0;
    }
    rx_callback(rx_element_fifo_0.data, rx_element_fifo_0.R1.bit.DLC); // buffer and size
    for (i = 0; i < rx_element_fifo_0.R1.bit.DLC; i++) {
      // TODO: received data handling
      // printf("  %d",rx_element_fifo_0.data[i]);
    }
  }

  if ((status & CAN_PROTOCOL_ERROR_ARBITRATION)
  || (status & CAN_PROTOCOL_ERROR_DATA)) {
    can_clear_interrupt_status(&can_instance, CAN_PROTOCOL_ERROR_ARBITRATION
    | CAN_PROTOCOL_ERROR_DATA);
    // TODO: Error handling
    // printf("Protocol error, please double check the clock in two boards. \r\n\r\n");
  }
}
//! [can_interrupt_handler]