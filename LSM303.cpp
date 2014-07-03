#include "LSM303.h"
/*

Relevant Pololu products:

#1250  LSM303DLH              SA0_A pulled to GND, accessible via.
#1264  LSM303DLH + L3G4200D   SA0_A pulled to GND, accessible thru-hole.
#1265  LSM303DLM + L3G4200D   SA0_A pulled to GND, accessible thru-hole.
#1268  LSM303DLHC + L3GD20
#1273  LSM303DLM              SA0_A pulled to GND, accessible via.
#2124  LSM303DLHC

LSM303DLHC has no SA0_A line

 */

#define MAG_ADDRESS            (0x3C >> 1)  // 0x1E
#define ACC_ADDRESS_SA0_A_LOW  (0x30 >> 1)  // 0x18
#define ACC_ADDRESS_SA0_A_HIGH (0x32 >> 1)  // 0x19


LSM303::LSM303(const char * i2cDeviceName) :
  i2c_mag(i2cDeviceName), i2c_acc(i2cDeviceName)
{
    // Detect the accelerometer address and device.
    bool device_determined = 0;

    // LSM303D with SA0 low pin
    if (device_determined != 1){
        i2c_mag.addressSet(LSM303D_SA0_LOW);
        i2c_acc.addressSet(LSM303D_SA0_LOW);
        if (i2c_acc.tryReadByte(LSM303_WHO_AM_I_M) == LSM303D_WHO_ID){
            device = Device::LSM303D;
            device_determined = 1;
        } 
    }
   
    // LSM303D with SA0 high pin
    if (device_determined != 1){ 
        i2c_mag.addressSet(LSM303D_SA0_HIGH);
        i2c_acc.addressSet(LSM303D_SA0_HIGH);
        if (i2c_acc.tryReadByte(LSM303_WHO_AM_I_M) == LSM303D_WHO_ID){
            device = Device::LSM303D;
            device_determined = 1;
        } 
    }

    // LSM303DLHC (high)
    if (device_determined != 1){
   
        i2c_mag.addressSet(MAG_ADDRESS);
        i2c_acc.addressSet(ACC_ADDRESS_SA0_A_LOW);
        bool sa0_a_high = i2c_acc.tryReadByte(LSM303_CTRL_REG1_A) == -1;
        if (sa0_a_high)
        {
            // Only the DLHC should be responding on the high address.
            i2c_acc.addressSet(ACC_ADDRESS_SA0_A_HIGH);
            device = Device::LSM303DLHC;
            device_determined = 1;
        }
        else
        {
            // Only the DLM has a LSM303_WHO_AM_I_M register.
            device = i2c_mag.tryReadByte(LSM303_WHO_AM_I_M) == 0x3C ? Device::LSM303DLM : Device::LSM303DLH;
            device_determined = 1;
        }
    
        // Make sure to throw an exception if we don't have the right address.
        readAccReg(LSM303_CTRL_REG1_A);

        if (readMagReg(LSM303_WHO_AM_I_M) != 0x3C)
        {
            throw std::runtime_error("Error getting \"Who Am I\" register.\n");
        }
    }

    // If device_determined hasn't been set to 1 yet, there is a problem somewhere
    if (device_determined != 1){
        throw std::runtime_error("Could not determine accelerometer type.\n");
    }


}

uint8_t LSM303::readMagReg(uint8_t reg)
{
    return i2c_mag.readByte(reg);
}

uint8_t LSM303::readAccReg(uint8_t reg)
{
    return i2c_acc.readByte(reg);
}

void LSM303::writeMagReg(uint8_t reg, uint8_t value)
{
    i2c_mag.writeByte(reg, value);
}

void LSM303::writeAccReg(uint8_t reg, uint8_t value)
{
    i2c_acc.writeByte(reg, value);
}

// Turns on the LSM303's accelerometer and magnetometers and places them in normal
// mode.
void LSM303::enable(void)
{
    // Enable accelerometer.
    
    if (device == Device::LSM303D)
    {
        writeAccReg(LSM303_CTRL_REG1_A, 0b01010111); // Normal power mode, all axes enabled, 50 Hz
        writeAccReg(LSM303_CTRL_REG2_A, 0b00011000); // 8 g full scale 
    }
    else if (device == Device::LSM303DLHC)
    {    
        writeAccReg(LSM303_CTRL_REG1_A, 0b01000111); // Normal power mode, all axes enabled, 50 Hz
        writeAccReg(LSM303_CTRL_REG4_A, 0b00101000); // 8 g full scale: FS = 10 on DLHC, high resolution output mode
    }
    else
    {
        writeAccReg(LSM303_CTRL_REG1_A, 0b00100111); // normal power mode, all axes enabled, 50 Hz
        writeAccReg(LSM303_CTRL_REG4_A, 0b00110000); // 8 g full scale: FS = 11 on DLH, DLM
    }

    // Enable magnetometer
    // Continuous conversion mode
    if (device == Device::LSM303D)
    {
        writeAccReg(LSM303_CTRL_REG5_A, 0b01100100); // high resolution output, 6.25 Hz
        writeAccReg(LSM303_CTRL_REG6_A, 0b01000000); // 8 g full scale
        writeMagReg(LSM303D_CTRL7, 0b00000000);
    }
    else
    { 
        writeMagReg(LSM303_MR_REG_M, 0b00110000);
    }
}

void LSM303::readAcc(void)
{
    uint8_t block[6];
    i2c_acc.readBlock(0x80 | LSM303_OUT_X_L_A, sizeof(block), block);

    a[0] = (int16_t)(block[0] | block[1] << 8) >> 4;
    a[1] = (int16_t)(block[2] | block[3] << 8) >> 4;
    a[2] = (int16_t)(block[4] | block[5] << 8) >> 4;
}

void LSM303::readMag(void)
{
    uint8_t block[6];

    if (device == Device::LSM303D){

        i2c_mag.readBlock(0x80 | LSM303D_OUT_X_L_M, sizeof(block), block);

        // register address order is X,Y,Z with low bytes first
        m[0] = (int16_t)(block[0] | block[1] << 8);
        m[1] = (int16_t)(block[2] | block[3] << 8);
        m[2] = (int16_t)(block[4] | block[5] << 8);

    } else {  
        i2c_mag.readBlock(0x80 | LSM303_OUT_X_H_M, sizeof(block), block);

        // DLM, DLHC: register address order is X,Z,Y with high bytes first
        m[0] = (int16_t)(block[1] | block[0] << 8);
        m[1] = (int16_t)(block[5] | block[4] << 8);
        m[2] = (int16_t)(block[3] | block[2] << 8);

        // TODO: handle DLH properly here (switch two components?)
    }



}

// Reads all 6 channels of the LSM303 and stores them in the object variables
void LSM303::read(void)
{
    readAcc();
    readMag();
}
