/**************************************************************************/
/*!
    @file       greenpak_wrapper.h
    @author     Vladimir Noha 
    @license    BSD (see LICENSE.txt)
    
    This is a simple wrapper for the I2C communication between GreenPAK SLG46** MSIM & ESP32 SoC
    The SLG46** provides a small, low power component for commonly used mixed-signal functions. 
    The user creates their circuit design by programming the one time Non-Volatile Memory (NVM) 
    to configure the interconnect logic, the I/O Pins and the macro cells of the SLG46**. 
    This highly versatile device allows a wide variety of mixed-signal functions to be designed 
    within a very small, low power single integrated circuit. 
 
    @section  HISTORY
 
    v1.0  - First version
*/
/**************************************************************************/

#include <Arduino.h>
#include <Wire.h>

class greenpak_wrapper
{
private:
    uint8_t     _i2c_addr;
    TwoWire*    _i2c;
public:
    void greenpak_interface_init(TwoWire* i2c, uint8_t addr);
    void write_register(uint8_t data);
    // uint8_t read_register();
};
