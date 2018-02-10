/*
 * Tx_Rx.h
 *
 * Created: 10.2.2018 12:15:11
 *  Author: E. Elgun
 */ 

#ifndef TX_RX_H_
#define TX_RX_H_

struct Tx {
	public:
	//Id of the channel which is assigned to a specific data stream.
	unsigned int channel_ID;
	int *data;
	unsigned int data_length;
	/*Frequency of the message transmission. This may have limitations
	  depending on the data, and used hardware */ 
	unsigned int frequency;
	//Toggle the channel. If set false, the specific can module will stop listening the stream.
	bool active;

	//Library specific code. 
	private:
	unsigned int error_counter;
	void error_handler(int error);
};

void Tx::error_handler(){
	
}

struct Rx{
	public:
	unsigned int channel_ID;
	int *data;
	unsigned int data_length;
	unsigned int frequency;
	bool active;
	
	//Getting the pointer for the specified received data. Will be handled by libraries. 
	int get_data();

	private:
	unsigned int error;
	void error_handler(int error);
};

int Rx::get_data() {
	int *data;
	return *data;
}

void Rx::error_handler(int error) {

}

#endif /* TX_RX_H_ */