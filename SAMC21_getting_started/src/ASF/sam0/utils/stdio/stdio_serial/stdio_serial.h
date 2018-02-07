
#ifndef STDIO_SERIAL_H_INCLUDED
#define STDIO_SERIAL_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include <serial.h>

/** Pointer to the base of the USART module instance to use for stdio. */
extern volatile void *volatile stdio_base;

/** Pointer to the external low level write function. */
extern int (*ptr_put)(void volatile*, char);

/** Pointer to the external low level read function. */
extern void (*ptr_get)(void volatile*, char*);

/** \brief Initializes the stdio in Serial Mode.
 *
 * \param module       Software USART instance to associate with the hardware.
 * \param hw           Base address of the USART hardware instance.
 * \param config       USART configuration parameters for the STDIO stream.
 *
 */
static inline void stdio_serial_init(
		struct usart_module *const module,
		usart_inst_t const hw,
		const struct usart_config *const config)
{
	stdio_base = (void *)module;
	//ptr_put = (int (*)(void volatile*,char))&usart_serial_putchar;
	//ptr_get = (void (*)(void volatile*,char*))&usart_serial_getchar;

	usart_serial_init(module, hw, config);
# if defined(__GNUC__)
	// Specify that stdout and stdin should not be buffered.
	//setbuf(stdout, NULL);
	//setbuf(stdin, NULL);
	// Note: Already the case in IAR's Normal DLIB default configuration
	// and AVR GCC library:
	// - printf() emits one character at a time.
	// - getchar() requests only 1 byte to exit.
#  endif
}

#ifdef __cplusplus
}
#endif

#endif  // _STDIO_SERIAL_H_
