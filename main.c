/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "MFRC522.c"

/* Defined for SPI communication */

void app_main()
{
    //printf("%d\n", CommandReg);

    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=5000000,               //Clock out at 5 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        //.pre_cb=ili_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    //Attach the RFID to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);
    /*printf("%x\n",PCD_ReadRegister(spi,ModWidthReg));
    printf("%x\n",PCD_ReadRegister(spi,TxControlReg));
    printf("%x\n",PCD_ReadRegister(spi,TxASKReg));
    
    printf("%x\n",PCD_ReadRegister(spi,ModWidthReg));
    printf("%x\n",PCD_ReadRegister(spi,TxControlReg));
    printf("%x\n",PCD_ReadRegister(spi,TxASKReg));*/
    PCD_Init(spi);
    while(1){
        if(PICC_IsNewCardPresent(spi)){
        vTaskDelay(100 / portTICK_PERIOD_MS);
        printf("Card present\n");}
        
    }
}
