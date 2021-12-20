#include "greenpak_wrapper.h"

// greenpak_wrapper::greenpak_wrapper(TwoWire* i2c, uint8_t addr){
//     _i2c = i2c;
//     _i2c_addr = addr;
// }

void greenpak_wrapper::greenpak_interface_init(TwoWire* i2c, uint8_t addr){
    _i2c = i2c;
    _i2c_addr = addr;
}

void greenpak_wrapper::write_register(uint8_t data){
    _i2c ->write(data);
}

// uint8_t greenpak_wrapper::read_register(){
//     uint8_t received_data = 0;
//     _i2c ->requestFrom(_i2c_addr, 1); // data length should be checked
//     while (_i2c->available())
//     {
//         received_data = _i2c->read();
//     }
    
//     return received_data;
// }