
#include "usart.h"
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

	//usart_serial_init(&cdc_uart_module, CONF_STDIO_USART_MODULE, &usart_conf);
	usart_init(&cdc_uart_module, CONF_STDIO_USART_MODULE, &usart_conf);
	usart_enable(&cdc_uart_module);
}

void write(const char* data, uint8_t len);
void write(const char* data, uint8_t len) {
	for (int i=0; i<len; i++) {
		while(STATUS_OK !=usart_write_wait(&cdc_uart_module, data[i]));
	}
}

int main(void)
{
	system_init();
	/*Configure UART console.*/
	configure_console();
	/*Initialize the delay driver*/
	write("Hello world\r\n",13);
	/*main loop*/
	
	uint count = 0;
	while(1) {
		char c = '0' + (count%10);
		count++;
		write(&c, 1);
		write("\r\n",2);
		write("stay hyped\r\n", 12);
		
		for (int i=0; i<1000000; i++);
	}
}

#ifdef __cplusplus
}
#endif
