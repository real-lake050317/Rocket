#include "sensor_mpu6050.h"

void MPU6050_Init(void)
{
    uint8_t chipid = 0;
    uint8_t tmp = 0;

    //reset the chip
    tmp = 0x80;
    i2c_write(MPU6050_I2C_ADDRESS ,MPU6050_PWR_MGMT_1, &tmp, 0x01);

    for(int i=0; i<30000; i++){__ASM("nop");}

    //wakeup the chip
    tmp = 0x00;
    i2c_write(MPU6050_I2C_ADDRESS ,MPU6050_PWR_MGMT_1, &tmp, 0x01);

    //read chip id
    i2c_read(MPU6050_I2C_ADDRESS ,MPU6050_WHO_AM_I, &chipid, 0x01);

    for(int i=0; i<30000; i++){__ASM("nop");}
        
    //internal 20MHz oscillator
    tmp = 0x00;
    i2c_write(MPU6050_I2C_ADDRESS ,MPU6050_PWR_MGMT_1, &tmp, 0x01);

    //read MPU6050_PWR_MGMT_2 register
    i2c_read(MPU6050_I2C_ADDRESS ,MPU6050_PWR_MGMT_2, &tmp, 0x01);

    //enable acc/gyro xyz
    tmp &= ~(0x38 | 0x07);
    i2c_write(MPU6050_I2C_ADDRESS ,MPU6050_PWR_MGMT_2, &tmp, 0x01);

    //acc +-2g
    tmp = ( 0x00 << 3 ); 
    i2c_write(MPU6050_I2C_ADDRESS ,MPU6050_ACCEL_CONFIG, &tmp, 0x01);
    
    //gyro 2000dps
    tmp = ( 0x03 << 3 );
    i2c_write(MPU6050_I2C_ADDRESS ,MPU6050_GYRO_CONFIG, &tmp, 0x01);
    
}

void MPU6050_GetAccData(float *acc)
{
    uint8_t tmp[6] = {0};
    int16_t tmp_int[3] = {0};

    i2c_read(MPU6050_I2C_ADDRESS ,MPU6050_ACCEL_XOUT_H, tmp, 0x06);
    tmp_int[0] = (int16_t)((tmp[0]<< 8) | tmp[1]);
    tmp_int[1] = (int16_t)((tmp[2]<< 8) | tmp[3]);
    tmp_int[2] = (int16_t)((tmp[4]<< 8) | tmp[5]);

    acc[0] = (float)(tmp_int[0]) * 0.00059854736328125f;
    acc[1] = (float)(tmp_int[1]) * 0.00059854736328125f;
    acc[2] = (float)(tmp_int[2]) * 0.00059854736328125f;
}

void MPU6050_GetGyroData(float *gyro)
{
    uint8_t tmp[6] = {0};
    int16_t tmp_int[3] = {0};

    i2c_read(MPU6050_I2C_ADDRESS ,MPU6050_GYRO_XOUT_H, tmp, 0x06);
    tmp_int[0] = (int16_t)((tmp[0]<< 8) | tmp[1]);
    tmp_int[1] = (int16_t)((tmp[2]<< 8) | tmp[3]);
    tmp_int[2] = (int16_t)((tmp[4]<< 8) | tmp[5]);

    gyro[0] = (float)(tmp_int[0]) * 0.001064225153455f;
    gyro[1] = (float)(tmp_int[1]) * 0.001064225153455f;
    gyro[2] = (float)(tmp_int[2]) * 0.001064225153455f;
}