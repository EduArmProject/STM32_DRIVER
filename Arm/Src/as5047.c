#include "as5047.h"

// to init spi bus of encoder
//  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
//  HAL_SPI_Init(hspi);

void AS5047P_Init(AS5047P_t *encoder, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin){
    
    encoder->spi_interface.hspi = hspi;
    encoder->spi_interface.cs_port = cs_port;
    encoder->spi_interface.cs_pin = cs_pin;

    encoder->current_angle = 0.0f;
    encoder->error_flag = false;
    encoder->mag_too_high = false;
    encoder->mag_too_low = false;
    encoder->agc_value = 0;

    HAL_GPIO_WritePin(
        encoder->spi_interface.cs_port, 
        encoder->spi_interface.cs_pin, 
        GPIO_PIN_SET
    );
}
// Calculate 

// send the 16 bits with 14 of data MSB first
bool AS5047P_UpdateAngle(AS5047P_t *encoder) {
    // send the request
    SPI_Transmit_16b(&encoder->spi_interface, AS5047P_CMD_READ_ANGLE);
   
    uint16_t response = SPI_Transmit_16b(&encoder->spi_interface, AS5047P_CMD_READ_NOP);
    
    // 3. Check for AS5047P error flag (Bit 14)
    if (response & (1 << 14)) {
        encoder->error_flag = true;
        SPI_Transmit_16b(&encoder->spi_interface, AS5047P_CMD_CLEAR_ERR);
        return false;
    }
    
    // 4. Extract the 14-bit angle (Bits 13:0) and convert to float degrees
    uint16_t raw_angle = response & 0x3FFF;
    encoder->current_angle = ((float)raw_angle / 16384.0f) * 360.0f; // convert 2^14 value to a 360 degree angle
    encoder->error_flag = false;
    
    return true;
}

bool AS5047P_Diagnostic(AS5047P_t *encoder) {
    SPI_Transmit_16b(&encoder->spi_interface, AS5047P_CMD_READ_DIAG);
    
    uint16_t response = SPI_Transmit_16b(&encoder->spi_interface, AS5047P_CMD_READ_NOP);
    
     if (response & (1 << 14)) {
        encoder->error_flag = true;
        SPI_Transmit_16b(&encoder->spi_interface, AS5047P_CMD_CLEAR_ERR);
        return false;
    }
    
    
    encoder->error_flag = false;

    // Bit 11: MAGL (Magnetic field too low)
    encoder->mag_too_low = (response & (1 << 11)) ? true : false;
    
    // Bit 10: MAGH (Magnetic field too high)
    encoder->mag_too_high = (response & (1 << 10)) ? true : false;
    
    // Bits 7:0 : AGC value (Automatic Gain Control)
    encoder->agc_value = (uint8_t)(response & 0x00FF); // mask on the last 8 bits 000000011111111
    
    // Return false if any magnetic issue is detected
    if (encoder->mag_too_low || encoder->mag_too_high) {
        return false; 
    }
    return true; // Magnet position is healthy
}