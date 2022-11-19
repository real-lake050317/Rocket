/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "sensor_mpu6050.h"
#include "nrf_pwr_mgmt.h"
#include "app_uart.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common addresses definition for temperature sensor. */
#define LM75B_ADDR          (0x90U >> 1)  // 1001 0000 >> 1 =  0100 1000

#define LM75B_REG_TEMP      0x00U
#define LM75B_REG_CONF      0x01U
#define LM75B_REG_THYST     0x02U
#define LM75B_REG_TOS       0x03U

/* Mode for LM75B. */
#define NORMAL_MODE 0U

/* Common addresses definition for MPU6050. */
#define MPU6050_ADDR       	0x


float MPU6050_Acc[3] = {0};
float MPU6050_Gyro[3] = {0};

bool i2c_write( uint8_t device_address, uint8_t register_address, uint8_t *value, uint8_t number_of_bytes );
bool i2c_read( uint8_t device_address, uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes );


#define C_Output_Acc                     0x61
#define C_Output_Gyro		                 0x67 

#define Output_Select_Gyro        			 0x01
#define Output_Select_Acc         			 0x00

#define Default_Zero										 0x00

uint8_t StringSelect;
char printf_buffer[256];
uint16_t length;

static volatile bool m_xfer_done = false;

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

static uint8_t m_sample;

void LM75B_set_mode(void) {
    ret_code_t err_code;

    uint8_t reg[2] = {LM75B_REG_CONF, NORMAL_MODE};
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    reg[0] = LM75B_REG_TEMP;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, 1, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

__STATIC_INLINE void data_handler(uint8_t temp) {
    NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
}

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context) {
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

void twi_init (void) {
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_MPU6050_config = {
       .scl                = MPU6050_SCL_PIN, // PIN 0.27
       .sda                = MPU6050_SDA_PIN, // PIN 0.26
       .frequency          = NRF_DRV_TWI_FREQ_100K, 
       .interrupt_priority = APP_IRQ_PRIORITY_LOW,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_MPU6050_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

static void read_sensor_data() {
    m_xfer_done = false;
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, LM75B_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}

bool i2c_write(uint8_t device_address, uint8_t register_address, uint8_t *value, uint8_t number_of_bytes ) {  
     uint8_t w_data[number_of_bytes+1], i;
   
     w_data[0] = register_address;
     for ( i = 0 ; i < number_of_bytes ; i++ )
     {
        w_data[i+1] = value[i];  
     }

     nrf_drv_twi_tx(&m_twi, (device_address)>>1, w_data, number_of_bytes+1, false);
     
     return true;
}

bool i2c_read(uint8_t device_address, uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes) {    
     nrf_drv_twi_tx(&m_twi, (device_address)>>1, &register_address, 1, true);
     nrf_drv_twi_rx(&m_twi, (device_address)>>1, destination, number_of_bytes);
       
     return true;  
}

static void idle_state_handle(void) {
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}

int main(void) {
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI example started");
    NRF_LOG_FLUSH();
	
	twi_init();	
	nrf_delay_ms(50);
    MPU6050_Init();	
    StringSelect = 1;
    length = Default_Zero;
    // Enter main loop.
    for (;;)
    {
        memset(printf_buffer, 0x00, sizeof(printf_buffer));
        if(StringSelect == 1)
		{					//if(StringSelect == Output_Select_Gyro)
		snprintf(printf_buffer, 100, "G: %.1f; %.1f; %.1f", MPU6050_Gyro[0], MPU6050_Gyro[1], MPU6050_Gyro[2]);
		}
        else{
            snprintf(printf_buffer,100, "A: %.1f; %.1f; %.1f", MPU6050_Acc[0], MPU6050_Acc[1], MPU6050_Acc[2]);
			}
        length = strlen(printf_buffer);
        //uint32_t err_code = ble_nus_data_send(&m_nus, (uint8_t *)printf_buffer, &length, m_conn_handle);
        nrf_delay_ms(500);
        MPU6050_GetAccData(MPU6050_Acc);
        nrf_delay_ms(50);
        MPU6050_GetGyroData(MPU6050_Gyro);
        //idle_state_handle();
		NRF_LOG_FLUSH();
    }
		
}