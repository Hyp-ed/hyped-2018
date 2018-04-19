#include <linux/i2c-dev.h>
#include <sys/ioctl>
#include <iostream>
// #include <i2c/smbus.h>: This is a part of i2ctools. Identical to upper library.

class I2C {
    public:
        I2C();
        int i2c_wr();
        int i2c_rd();
        int i2c_transaction();
    private:
        int file;
        int adapter_id; //May change from boot to boot.
        int addr;
};

/**
 * Adapter is the lane that is going to be opened. device_addr is 
 * the address of the device on the I2C lane. This can be discovered
 * using I2C on BB's shell. If 10bit_addr is 1, 10 bit adressing will 
 * be enabled.
**/
I2C::I2C(int adapter, int device_addr, bool *10bit_addr)
{
    adapter_id = adapter;
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", adapter_id);
    file = open(filename, O_RDWR);
    if (file < 0) {
        std::cout << "Could not open I2C lane.";
        exit(1);
    }

    addr = device_addr;
    if(ioctl(file, I2C_SLAVE, addr) < 0) {
        std::cout << "Could not access/communicate to slave."
        exit(1);
    }

    if(10bit_addr)
    {
        int 10bit;
        10bit = ioctl(adapter, I2C_TENBIT, 1);
        if(10bit < 0) {return 10bit}
        return 0;
    }
}

I2C::~I2C()
{
    close(adapter);
}

int I2C::i2c_read(char *buff[], int n_bytes)
{
    int com;
    com = write(adapter_id, buff, n_bytes)
    if(com < 0){return com;}
    return 0
}

int I2C::i2c_wr(char *buff[], int n_bytes)
{
    int com;
    com = write(adapter_id, buff, n_bytes);
    if (com < 0) {return com;}
    return 0;
}

int I2C::i2c_transaction(uint8_t command, char buff[], int n_bytes)
{
    int com;
    com = write(adapter_id, &command, 1);
    if (com < 0) {return com}
    com = read(adapter_id, buff, n_bytes);
    if (com < 0) {return com};
    return 0;
}