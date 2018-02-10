/*
 * can_bus.h
 *
 * Created: 10.2.2018 12:02:53
 *  Author: E. Elgun 
 */ 

#include "Tx_Rx.hpp"

#ifndef CAN_BUS_H_
#define CAN_BUS_H_

namespace can_bus{
	
	//Dedicated file to set up hardware specific settings, such as the pointer to the message. 
	//Code in this part will vary depending on the hardware. 
	struct context {
		//Global setting to use remote messages in a given board. 
		bool remote_message;
	}

	class CAN {
		public:
		//Configuration of the CAN module for specified context.
		void configure(context *cx);
		void initialize();
		//This method will be called when the CAN module is ready and ready to transmit and recieve data.
		//This is the only method of the library which will be called over and over during run-time.
		void control();
		Rx rx_module1_recieve;
		Tx tx_module1_transmit;
		};

	void CAN::configure(context *tx) {
		
		}

	void CAN::initialize() {

		}

	void CAN::control() {

		}
	}
#endif /* CAN_BUS_H_ */