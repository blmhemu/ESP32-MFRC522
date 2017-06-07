#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "MFRC522.h"

void PCD_Version(spi_device_handle_t spi){

    uint8_t ver = PCD_ReadRegister(spi,VersionReg);
    if (ver == 0x92)
    {
        printf("MFRC522 Version 2 detected.\n");
    }
    else if (ver == 0x91)
    {
        printf("MFRC522 Version 1 detected.\n");
    }
    else{
        printf("Is connected device MFRC522 ? If yes, check the wiring again.\n");

        for (int i = 5; i >= 0; i--) {
            printf("Restarting in %d seconds...\n", i);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        printf("Restarting Now.\n");
        fflush(stdout);
        esp_restart();
    }
}
void PCD_WriteRegister(spi_device_handle_t spi , uint8_t Register , uint8_t value){

    esp_err_t ret;
    uint8_t reg = Register;
    uint8_t val = value;
    static spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.flags=SPI_TRANS_USE_TXDATA;
    t.length = 16;
    t.tx_data[0] = reg;
    t.tx_data[1] = val;
    ret = spi_device_queue_trans(spi,&t,10);
    assert(ret==ESP_OK);
    spi_transaction_t *rtrans;
    ret = spi_device_get_trans_result(spi,&rtrans,10);
    assert(ret==ESP_OK);
}

void PCD_WriteRegisterMany(spi_device_handle_t spi , uint8_t Register, uint8_t count, uint8_t *values){

    esp_err_t ret;
    uint8_t total[count+1];
    total[0] = Register;

    for (int i = 1; i <= count; ++i)
    {
        total[i] = values[i-1];
    }

    static spi_transaction_t t1;
    memset(&t1, 0, sizeof(t1));       //Zero out the transaction
    t1.length = 8*(count+1);
    t1.tx_buffer = total;

    ret = spi_device_transmit(spi,&t1);
    assert(ret==ESP_OK);
}

uint8_t PCD_ReadRegister(spi_device_handle_t spi , uint8_t Register){

    esp_err_t ret;
    uint8_t reg = Register | 0x80;
    uint8_t val;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 8;
    t.tx_buffer = &reg;
    t.rx_buffer = &val;
    ret = spi_device_transmit(spi,&t);
    assert(ret==ESP_OK);
    t.tx_buffer=(uint8_t*)0;
    ret = spi_device_transmit(spi,&t);
    assert(ret==ESP_OK);
    return val;    
}

void PCD_ReadRegisterMany(spi_device_handle_t spi,
                                uint8_t Register,   ///< The register to read from. One of the PCD_Register enums.
                                uint8_t count,         ///< The number of bytes to read
                                uint8_t *values,       ///< Byte array to store the values in.
                                uint8_t rxAlign        ///< Only bit positions rxAlign..7 in values[0] are updated.
                                ) {
    if (count == 0) {
        return;
    }

    esp_err_t ret;
    uint8_t reg = 0x80 | Register;              // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));                  //Zero out the transaction
    t.length = 8;
    t.rxlength = 8*count;
    t.tx_buffer = &reg;
    t.rx_buffer = values;
    ret = spi_device_transmit(spi,&t);
    assert(ret==ESP_OK);
    t.tx_buffer=(uint8_t*)0;
    ret = spi_device_transmit(spi,&t);
    assert(ret==ESP_OK);
}

void PCD_ClearRegisterBitMask(spi_device_handle_t spi,uint8_t reg,   ///< The register to update. One of the PCD_Register enums.
                                        uint8_t mask           ///< The bits to clear.
                                      ) {
    uint8_t tmp;
    tmp = PCD_ReadRegister(spi,reg);
    PCD_WriteRegister(spi,reg, tmp & (~mask));      // clear bit mask
}

void PCD_SetRegisterBitMask(spi_device_handle_t spi , uint8_t reg, uint8_t mask){
    uint8_t tmp;
    tmp = PCD_ReadRegister(spi,reg);
    PCD_WriteRegister(spi,reg, tmp | mask);         // set bit mask
}

void PCD_Init(spi_device_handle_t spi){
    
    PCD_Version(spi);
    gpio_pad_select_gpio(PIN_NUM_RST);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);

    // Hard Reset RFID
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // Reset baud rates
    PCD_WriteRegister(spi,TxModeReg, 0x00);
    PCD_WriteRegister(spi,RxModeReg, 0x00);
    // Reset ModWidthReg
    PCD_WriteRegister(spi,ModWidthReg, 0x26);

    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.

    PCD_WriteRegister(spi,TModeReg, 0x80);          // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    PCD_WriteRegister(spi,TPrescalerReg, 0xA9);     // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
    PCD_WriteRegister(spi,TReloadRegH, 0x03);       // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    PCD_WriteRegister(spi,TReloadRegL, 0xE8);
    PCD_WriteRegister(spi,TxASKReg, 0x40);      // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    PCD_WriteRegister(spi,ModeReg, 0x3D);       // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    PCD_AntennaOn(spi);                        // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
    printf("Initialization successful.\n");

}

void PCD_AntennaOn(spi_device_handle_t spi){
    uint8_t value = PCD_ReadRegister(spi,TxControlReg);
    if ((value & 0x03) != 0x03) {
        PCD_WriteRegister(spi,TxControlReg, value | 0x03);
    }
    printf("Antenna turned on.\n");
}

bool PICC_IsNewCardPresent(spi_device_handle_t spi){
    static uint8_t bufferATQA[3]={0,0,0};
    uint8_t bufferSize = sizeof(bufferATQA);
    // Reset baud rates
    PCD_WriteRegister(spi,TxModeReg, 0x00);
    PCD_WriteRegister(spi,RxModeReg, 0x00);
    // Reset ModWidthReg
    PCD_WriteRegister(spi,ModWidthReg, 0x26);
    uint8_t result = PICC_RequestA(spi,bufferATQA, &bufferSize);
    return (result == STATUS_OK || result == STATUS_COLLISION);
}

uint8_t PICC_RequestA(spi_device_handle_t spi,uint8_t *bufferATQA,uint8_t *bufferSize){
    return PICC_REQA_or_WUPA(spi,PICC_CMD_REQA, bufferATQA, bufferSize);
}

uint8_t PICC_REQA_or_WUPA(spi_device_handle_t spi,uint8_t command,uint8_t *bufferATQA,uint8_t *bufferSize){
    uint8_t validBits;
    uint8_t status;
    if(bufferATQA == NULL || *bufferSize < 2){
        return STATUS_NO_ROOM;
    }
    PCD_ClearRegisterBitMask(spi,CollReg, 0x80);        // ValuesAfterColl=1 => Bits received after collision are cleared.
    validBits = 7;                                      // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) uint8_t. TxLastBits = BitFramingReg[2..0]
    status = PCD_TransceiveData(spi,&command, 1, bufferATQA, bufferSize, &validBits,0,false);
    if (status != STATUS_OK) {
        return status;
    }
    if (*bufferSize != 2 || validBits != 0) {           // ATQA must be exactly 16 bits.
        return STATUS_ERROR;                        
    }
    return STATUS_OK;
}

uint8_t PCD_TransceiveData( spi_device_handle_t spi,
                                                    uint8_t *sendData,     ///< Pointer to the data to transfer to the FIFO.
                                                    uint8_t sendLen,       ///< Number of uint8_ts to transfer to the FIFO.
                                                    uint8_t *backData,     ///< nullptr or pointer to buffer if data should be read back after executing the command.
                                                    uint8_t *backLen,      ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                                    uint8_t *validBits,    ///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits. Default nullptr.
                                                    uint8_t rxAlign,       ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                                    bool checkCRC       ///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
                                 ) {
    uint8_t waitIRq = 0x30;        // RxIRq and IdleIRq
    return PCD_CommunicateWithPICC(spi,PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}

// TO BE COMPLETED .....
/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t PCD_CommunicateWithPICC(spi_device_handle_t spi,
                                                        uint8_t command,       ///< The command to execute. One of the PCD_Command enums.
                                                        uint8_t waitIRq,       ///< The bits in the ComIrqReg register that signals successful completion of the command.
                                                        uint8_t *sendData,     ///< Pointer to the data to transfer to the FIFO.
                                                        uint8_t sendLen,       ///< Number of bytes to transfer to the FIFO.
                                                        uint8_t *backData,     ///< nullptr or pointer to buffer if data should be read back after executing the command.
                                                        uint8_t *backLen,      ///< In: Max number of bytess to write to *backData. Out: The number of uint8_ts returned.
                                                        uint8_t *validBits,    ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
                                                        uint8_t rxAlign,       ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                                        bool checkCRC          ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
                                     ) {

    // Prepare values for BitFramingReg
    uint8_t txLastBits = validBits ? *validBits : 0;
    uint8_t bitFraming = (rxAlign << 4) + txLastBits;      // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
    
    PCD_WriteRegister(spi,CommandReg, PCD_Idle);            // Stop any active command.
    PCD_WriteRegister(spi,ComIrqReg, 0x7F);                 // Clear all seven interrupt request bits
    PCD_WriteRegister(spi,FIFOLevelReg, 0x80);              // FlushBuffer = 1, FIFO initialization
    PCD_WriteRegisterMany(spi,FIFODataReg,sendLen,sendData);  // Write sendData to the FIFO
    PCD_WriteRegister(spi,BitFramingReg, bitFraming);       // Bit adjustments
    PCD_WriteRegister(spi,CommandReg, command);             // Execute the command
    if (command == PCD_Transceive) {
        PCD_SetRegisterBitMask(spi,BitFramingReg, 0x80);    // StartSend=1, transmission of data starts
    }
    
    // Wait for the command to complete.
    // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
    // Each iteration of the do-while-loop takes 17.86μs.
    // TODO check/modify for other architectures than Arduino Uno 16bit
    uint16_t i;
    for (i = 20000; i > 0; i--) {
        uint8_t n = PCD_ReadRegister(spi,ComIrqReg);   // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
        if (n & waitIRq) {                  // One of the interrupts that signal success has been set.
            break;
        }
        if (n & 0x01) {                     // Timer interrupt - nothing received in 25ms
            return STATUS_TIMEOUT;
        }
    }
    // 35.7ms and nothing happend. Communication with the MFRC522 might be down.
    if (i == 0) {
        return STATUS_TIMEOUT;
    }
    
    // Stop now if any errors except collisions were detected.
    uint8_t errorRegValue = PCD_ReadRegister(spi,ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
    if (errorRegValue & 0x13) {  // BufferOvfl ParityErr ProtocolErr
        return STATUS_ERROR;
    }
  
    uint8_t _validBits = 0;
    
    // If the caller wants data back, get it from the MFRC522.
    if (backData && backLen) {
        uint8_t n = PCD_ReadRegister(spi,FIFOLevelReg);    // Number of bytes in the FIFO
        if (n > *backLen) {
        
            return STATUS_NO_ROOM;
        }
        *backLen = n;                                           // Number of bytes returned
        PCD_ReadRegisterMany(spi,FIFODataReg, n, backData, rxAlign);    // Get received data from FIFO
        _validBits = PCD_ReadRegister(spi,ControlReg) & 0x07;       // RxLastBits[2:0] indicates the number of valid bits in the last received uint8_t. If this value is 000b, the whole uint8_t is valid.
        if (validBits) {
            *validBits = _validBits;
        }

    }
    
    // Tell about collisions
    if (errorRegValue & 0x08) {     // CollErr
        return STATUS_COLLISION;
    }
    
    // Perform CRC_A validation if requested.
    /*if (backData && backLen && checkCRC) {
        // In this case a MIFARE Classic NAK is not OK.
        if (*backLen == 1 && _validBits == 4) {
            return STATUS_MIFARE_NACK;
        }
        // We need at least the CRC_A value and all 8 bits of the last uint8_t must be received.
        if (*backLen < 2 || _validBits != 0) {
            return STATUS_CRC_WRONG;
        }
        // Verify CRC_A - do our own calculation and store the control in controlBuffer.
        uint8_t controlBuffer[2];
        uint8_t status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
        if (status != STATUS_OK) {
            return status;
        }
        if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
            return STATUS_CRC_WRONG;
        }
    }*/
    return STATUS_OK;
} // End PCD_CommunicateWithPICC()