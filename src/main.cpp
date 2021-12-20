/***************************************************************************************************
 ______  _____ _____ ____ ___  _____       _           _   ______             _            
|  ____|/ ____|  __ \___ \__ \|  __ \     | |         | | |  ____|           (_)           
| |__  | (___ | |__) |__) | ) | |__) |___ | |__   ___ | |_| |__   _ __   __ _ _ _ __   ___ 
|  __|  \___ \|  ___/|__ < / /|  _  // _ \| '_ \ / _ \| __|  __| | '_ \ / _` | | '_ \ / _ \
| |____ ____) | |    ___) / /_| | \ \ (_) | |_) | (_) | |_| |____| | | | (_| | | | | |  __/
|______|_____/|_|   |____/____|_|  \_\___/|_.__/ \___/ \__|______|_| |_|\__, |_|_| |_|\___|
                                                                         __/ |             
                                                                        |___/              

          
***************************************************************************************************/

#include <Arduino.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include "Wire.h"
#include "greenpak_wrapper.h"
#include "command_wrapper.h"
#include "Pangodream_18650_CL.h"

#define SOC                 "ESP32"

#define SDA                 18
#define SCL                 19
#define I2C_FREQUENCY       400000 // I2C frequency in Hz

// I2C GreenPAK addrese`s
#define GP_ADDR             0x8

// OUTPUTS
#define G_LED               16
#define R_LED               17

// INPUTS
#define STDBY_CHARGER       22
#define CHRG_CHARGER        23
#define CELL_1_OUT          25
#define CELL_2_OUT          26
#define CELL_3_OUT          27
#define CELL_4_OUT          28

#define INPUT_BITMASK       ((1ULL<<STDBY_CHARGER)|(1ULL<<CHRG_CHARGER)|(1ULL<<CELL_1_OUT)|(1ULL<<CELL_2_OUT)|(1ULL<<CELL_3_OUT)|(1ULL<<CELL_4_OUT))
#define OUTPUT_BITMASK      ((1ULL<<G_LED)|(1ULL<<R_LED))

#ifdef __cplusplus
  extern "C" {
 #endif

  uint8_t temprature_sens_read();

#ifdef __cplusplus
}
#endif

uint8_t temprature_sens_read();

double cpu_temp_threshold = 65.0;
int reset_time = 5;
int battery_low_charg_level_threshold  = 25;

static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

TwoWire i2c = TwoWire(0);

greenpak_wrapper gp_motor_driver;
Pangodream_18650_CL bat;


void adc_check_efuse(){
    static esp_adc_cal_characteristics_t *adc_chars;
    adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, 1160, adc_chars);
    
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        Serial.println("eFuse Two Point: Supported");
    } else {
        Serial.println("eFuse Two Point: NOT supported");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        Serial.println("eFuse Vref: Supported");
    } else {
        Serial.println("eFuse Vref: NOT supported");
    }
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.println("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.println("Characterized using eFuse Vref");
    } else {
        Serial.println("Characterized using Default Vref");
    }
    switch (atten)
    {
        case 0:
        Serial.println("ADC_ATTEN_DB_0. No chages for the input voltage");
        break;
        case 1:
        Serial.println("ADC_ATTEN_DB_2_5. The input voltage will be reduce to about 1/1.34.");
        break;
        case 2:
        Serial.println("ADC_ATTEN_DB_6. The input voltage will be reduced to about 1/2");
        break;
        case 3:
        Serial.println("ADC_ATTEN_DB_11. The input voltage will be reduced to about 1/3.6");
        break;  
        default:
        Serial.println("Unknown attenuation.");
        break;
    }
    switch (adc_chars->bit_width)
    {
        case 0:
        Serial.println("ADC_WIDTH_BIT_9. ADC capture width is 9Bit");
        break;
        case 1:
        Serial.println("ADC_WIDTH_BIT_10. ADC capture width is 10Bit");
        break;
        case 2:
        Serial.println("ADC_WIDTH_BIT_11. ADC capture width is 11Bit");
        break;
        case 3:
        Serial.println("ADC_WIDTH_BIT_12. ADC capture width is 12Bit");  
        break;
        default:
        Serial.println("Unknown width.");
        break;
    }
}

// Check the ESP32 CPU internal temperature
void core_temperature_checker(){
    double temp_C = (temprature_sens_read() - 32) / 1.8;
    Serial.printf("ESP32 CPU Core temperature: %f C\n", temp_C);
    if(temp_C >= cpu_temp_threshold){
        digitalWrite(R_LED, 0x1);
        Serial.println("!!!WARNING!!!\n SYSTEM OVERHEATED");
    }
}


bool battery_voltage(){
    analogRead(ADC1_CHANNEL_6_GPIO_NUM);
    int battery_charge_level = bat.getBatteryChargeLevel();
    int battery_charge_volts = bat.getBatteryVolts();
    Serial.printf("Battery voltage: %i\n", battery_charge_volts);
    Serial.printf("Battery charge level: %i\n", battery_charge_level);
    if(battery_charge_level <= battery_low_charg_level_threshold){
        Serial.println("Battery requiers charging!");
        return false;
    }
    return true;
}

// Check the battery charge status
void battery_charge_checker(){
    uint8_t charger_stdby = gpio_get_level((gpio_num_t)STDBY_CHARGER) == 0x0;
    uint8_t charger_chrg = gpio_get_level((gpio_num_t)CHRG_CHARGER) == 0x0;

    if(charger_stdby == 0x0){
        Serial.println("Battery charged");
    }

    if (charger_chrg == 0x0){
        Serial.println("Battery charging");
    }
}


void movement_handler(){

const int NUM_SENSORS = 4;

enum button_enum {
  FORWARD,
  BACKWARD,
  LEFT_SIDE,
  RIGHT_SIDE
};

const int pins[NUM_SENSORS] = {
  CELL_1_OUT,           
  CELL_2_OUT,           
  CELL_3_OUT,           
  CELL_4_OUT,           
};

bool trigered[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++)
    {
      bool previous = trigered[i];
      trigered[i] = gpio_get_level((gpio_num_t)pins[i]) == 0x0;

      if (trigered[i] != previous)
        {
          if (trigered[i])
            {                                    
              switch (i)
                {
                case FORWARD:
                  Serial.println("MOVING BACKWARD");
                  gp_motor_driver.write_register(MOVE_BACKWARD);
                  break;

                case BACKWARD:
                  Serial.println("MOVING FORWARD");
                  gp_motor_driver.write_register(MOVE_FORWARD);
                  break;

                case LEFT_SIDE:
                  Serial.println("MOVING RIGHT");
                  gp_motor_driver.write_register(TURN_RIGHT);
                  break;
                
                case RIGHT_SIDE:
                  Serial.println("MOVING LEFT");
                  gp_motor_driver.write_register(TURN_LEFT);
                  break;

                default:
                  Serial.println("NO MOTION");
                  gp_motor_driver.write_register(STOP);
                  break;
                }
            }
        }
    }
}


void setup(){
    gpio_config_t io_conf = {};
        // OUTPUTS config
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = OUTPUT_BITMASK;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);
        // INPUT config
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = INPUT_BITMASK;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, atten);
    
    // pinMode(CELL_1_OUT, INPUT_PULLUP);
    // pinMode(CELL_2_OUT, INPUT_PULLUP);
    // pinMode(CELL_3_OUT, INPUT_PULLUP);
    // pinMode(R_LED, 0x02);
    // pinMode(G_LED, 0x02);
    // pinMode(STDBY_CHARGER, INPUT_PULLUP);
    // pinMode(CHRG_CHARGER, INPUT_PULLUP);

    Serial.begin(9600); // UART init

    i2c.begin(          // I2C Init
        SDA,
        SCL,
        I2C_FREQUENCY
    );

    adc_check_efuse();
    
    gp_motor_driver.greenpak_interface_init(&i2c, GP_ADDR);

    GPIO.out_w1ts = (1ULL << G_LED);
    GPIO.out_w1ts = (1ULL << R_LED);

    // digitalWrite(G_LED, 0x0);
    // digitalWrite(R_LED, 0x0);

    Serial.print("SoC: ");
    Serial.println(SOC);
    Serial.printf("Firmware size: %i\n",ESP.getSketchSize());
    Serial.printf("Free space for firmware: %i\n",ESP.getFreeSketchSpace());
    Serial.printf("Heap size: %i\n",ESP.getHeapSize());
    Serial.printf("Free heap: %i\n",ESP.getFreeHeap());
    Serial.print("SDK version: ");
    Serial.println(ESP.getSdkVersion());
    battery_charge_checker();
    // digitalWrite(G_LED, 0x1);
    GPIO.out_w1tc = (1ULL << G_LED);

}


void loop(){
    bool btr_state = battery_voltage();
    if(!btr_state){
        GPIO.out_w1tc = (1ULL << R_LED);
    }
    movement_handler();
}
