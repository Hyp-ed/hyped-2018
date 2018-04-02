/*
 * CanAPI.h
 *
 * Created: 15/03/2018 03:38:10
 *  Author: Piotr
 */ 


#ifndef CANAPI_H_
#define CANAPI_H_


void configure_can(void);
void can_register_callback(void (uint8_t* buffer, uint8_t size));
void can_send_standard_message(uint32_t id_value, uint8_t *data, uint32_t data_length);


#endif /* CANAPI_H_ */