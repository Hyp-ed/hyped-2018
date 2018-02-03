#include "asf.h"
#include "stdio_serial.h"
#include "conf_uart_serial.h"

#ifdef __cplusplus
extern "C" {
#endif

static struct usart_module cdc_uart_module;

/**
 *  Configure UART console.
 */
static void configure_console(void)
{
	struct usart_config usart_conf;

	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = CONF_STDIO_MUX_SETTING;
	usart_conf.pinmux_pad0 = CONF_STDIO_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = CONF_STDIO_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = CONF_STDIO_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = CONF_STDIO_PINMUX_PAD3;
	usart_conf.baudrate    = CONF_STDIO_BAUDRATE;

	stdio_serial_init(&cdc_uart_module, CONF_STDIO_USART_MODULE, &usart_conf);
	usart_enable(&cdc_uart_module);
}


int main(void)
{
	struct port_config pin;
	system_init();
	/*Configure UART console.*/
	configure_console();
	/*Initialize the delay driver*/
	delay_init();
	/* Output example information */
	puts("Starting up\r\n");
	/*Configures PORT for LED0*/
	port_get_config_defaults(&pin);
	pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED0_PIN, &pin);
	port_pin_set_output_level(LED0_PIN, LED0_INACTIVE);

	/*main loop*/
	while(1) {
		puts("hello world! stay hyped\r");
		delay_s(1);
		port_pin_toggle_output_level(LED0_PIN);
	}
}

#ifdef __cplusplus
}
#endif
