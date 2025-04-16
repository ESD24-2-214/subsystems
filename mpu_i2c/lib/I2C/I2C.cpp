extern "C" {
    #include <stdlib.h>
    #include <string.h>
    #include <inttypes.h>
    }
    
    #include "esp32-hal-i2c.h"
    #include "esp32-hal-i2c-slave.h"
    #include "I2C.hpp"
    #include "Arduino.h"


MasterI2C::MasterI2C(uint8_t bus_num)
    :esp_i2c_num(bus_num & 1)
    ,sda(-1)
    ,scl(-1)
    ,bufferSize(I2C_BUFFER_LENGTH) // default Buffer Size
    ,rxBuffer(NULL)
    ,rxIndex(0)
    ,rxLength(0)
    ,txBuffer(NULL)
    ,txLength(0)
    ,txAddress(0)
    ,_timeOutMillis(50)
    ,transmitting(false)
#if !CONFIG_DISABLE_HAL_LOCKS
    ,transmittingTask(NULL)
    ,I2C_lock(NULL)
#endif
{}

MasterI2C::~MasterI2C(void)
{
    end();
#if !CONFIG_DISABLE_HAL_LOCKS
    if(I2C_lock != NULL){
        vSemaphoreDelete(I2C_lock);
    }
#endif
}

bool MasterI2C::initPins(int sdaPin, int sclPin)
{
    if(sdaPin < 0) { // default param passed
        if(sda==-1) {
                sdaPin = SDA;    //use Default Pin
            } else {
                sdaPin = sda;    // reuse prior pin
            }
    if(sclPin < 0) { // default param passed
        if(scl==-1) {
                sclPin = SCL;    //use Default Pin
            } else {
                sclPin = scl;    // reuse prior pin
            }
        }
    }
    sda = sdaPin;
    scl = sclPin;
    return true;
}

bool MasterI2C::setPins(int sdaPin, int sclPin)
{
#if !CONFIG_DISABLE_HAL_LOCKS
    if(!create_acquire_lock()){return false;}
#endif

    if(!i2cIsInit(esp_i2c_num)){
        initPins(sdaPin, sclPin);
    } else {
        log_e("bus already initialized. change pins only when not.");
    }

#if !CONFIG_DISABLE_HAL_LOCKS 
    release_lock(I2C_lock); 
#endif
    return i2cIsInit(esp_i2c_num);
}    

bool MasterI2C::allocateI2CBuffer(void){
    // both buffer can be allocated or none will be
    if (rxBuffer == NULL) {
        rxBuffer = (uint8_t *)malloc(bufferSize);
        if (rxBuffer == NULL) {
            log_e("Can't allocate memory for I2C_%d rxBuffer", esp_i2c_num);
            return false;
        }
    }
    if (txBuffer == NULL) {
        txBuffer = (uint8_t *)malloc(bufferSize);
        if (txBuffer == NULL) {
            log_e("Can't allocate memory for I2C_%d txBuffer", esp_i2c_num);
            freeI2CBuffer();  // free rxBuffer for safety!
            return false;
        }
    }
    // in case both were allocated before, they must have the same size. All good.
    return true;
}

void MasterI2C::freeI2CBuffer(void){
    if (rxBuffer != NULL) {
        free(rxBuffer);
        rxBuffer = NULL;
    }
    if (txBuffer != NULL) {
        free(txBuffer);
        txBuffer = NULL;
    }
}

size_t MasterI2C::setBufferSize(size_t bSize){
    // Maximum size .... HEAP limited ;-)
    if (bSize < 32) {    // 32 bytes is the I2C FIFO Len for ESP32/S2/S3/C3
        log_e("Minimum Wire Buffer size is 32 bytes");
        return 0;
    }
#if !CONFIG_DISABLE_HAL_LOCKS
    if(!create_acquire_lock()){return false;}
#endif
    // allocateWireBuffer allocates memory for both pointers or just free them
    if (rxBuffer != NULL || txBuffer != NULL) {
        // if begin() has been already executed, memory size changes... data may be lost. We don't care! :^)
        if (bSize != bufferSize) {
            // we want a new buffer size ... just reset buffer pointers and allocate new ones
            freeI2CBuffer();
            bufferSize = bSize;
            if (!allocateI2CBuffer()) {
                // failed! Error message already issued
                bSize = 0; // returns error
                log_e("Buffer allocation failed");
            }
        } // else nothing changes, all set!
    } else {
        // no memory allocated yet, just change the size value - allocation in begin()
        bufferSize = bSize;
    }
#if !CONFIG_DISABLE_HAL_LOCKS
    release_lock(I2C_lock);
#endif
    return bSize;
}

bool MasterI2C::begin(int sdaPin, int sclPin, uint32_t frequency=0){
    bool started = false;
    esp_err_t err = ESP_OK;
#if !CONFIG_DISABLE_HAL_LOCKS
    if(!create_acquire_lock()){return false;}
#endif
    if(i2cIsInit(esp_i2c_num)){
        log_w("Bus already started.");
        started = true;
        goto end;
    }
    if (!allocateI2CBuffer()) {
        // failed! Error Message already issued
        goto end;
    }
    if(!initPins(sdaPin, sclPin)){
        goto end;
    }
    err = i2cInit(esp_i2c_num, sda, scl, frequency);
    started = (err == ESP_OK);

end:
    if (!started) {freeI2CBuffer();}
#if !CONFIG_DISABLE_HAL_LOCKS
    release_lock(I2C_lock);
#endif
    return started;
}

bool MasterI2C::end(void){
    esp_err_t err = ESP_OK;
#if !CONFIG_DISABLE_HAL_LOCKS
    if(!acquire_lock(I2C_lock)){return false;}
#endif
    if(i2cIsInit(esp_i2c_num)){
        err = i2cDeinit(esp_i2c_num);
    }
    freeI2CBuffer();
#if !CONFIG_DISABLE_HAL_LOCKS
    release_lock(I2C_lock);
#endif
    return (err == ESP_OK);
}

uint32_t MasterI2C::getClock(void){
    uint32_t frequency = 0;
#if !CONFIG_DISABLE_HAL_LOCKS
    if(!acquire_lock(I2C_lock)){return false;}
#endif
    i2cGetClock(esp_i2c_num, &frequency);
#if !CONFIG_DISABLE_HAL_LOCKS
    release_lock(I2C_lock);
#endif
    return frequency;
}

bool MasterI2C::setClock(uint32_t frequency){
    esp_err_t err = ESP_OK;
#if !CONFIG_DISABLE_HAL_LOCKS
    if(!acquire_lock(I2C_lock)){return false;}
#endif
    err = i2cSetClock(esp_i2c_num, frequency);
#if !CONFIG_DISABLE_HAL_LOCKS
    release_lock(I2C_lock);
#endif
    return (err == ESP_OK);
}

void MasterI2C::setTimeOut(uint16_t timeOutMillis)
{
    _timeOutMillis = timeOutMillis;
}

void MasterI2C::beginTransmission(uint16_t address){

#if !CONFIG_DISABLE_HAL_LOCKS
    if(transmitting && transmittingTask == xTaskGetCurrentTaskHandle()){
    log_e("Expected endTransmission not beginTransmission! Clearing...");
    //release lock
    release_lock(I2C_lock);
    }
    if(I2C_lock == NULL || acquire_lock(I2C_lock) != pdTRUE){
        log_e("could not acquire lock");
        return;
    }
#endif
    transmitting = true;
    transmittingTask = xTaskGetCurrentTaskHandle();
    txAddress = address;
    txLength = 0;
}

uint8_t MasterI2C::transmit(){
    if(!transmitting 
#if !CONFIG_DISABLE_HAL_LOCKS       
        || transmittingTask == xTaskGetCurrentTaskHandle()
#endif    
    ){
        log_e("Transmission not startet. Expected beginTransmission");
        return 4;
    }
    if (txBuffer == NULL){
        log_e("NULL TX buffer pointer");
        return 4;
    }
    esp_err_t err = ESP_OK;
    err = i2cWrite(esp_i2c_num, txAddress, txBuffer, txLength, _timeOutMillis);
    switch(err){
        case ESP_OK: return 0;
        case ESP_FAIL: return 2;
        case ESP_ERR_TIMEOUT: return 5;
        default: break;
    }
    return 4;
}

uint8_t MasterI2C::endTransmission(void){
    if(transmitting == false){
        log_e("Transmission not startet, expected beginTransmission");
        return ESP_FAIL;
    }
#if !CONFIG_DISABLE_HAL_LOCKS
    if(I2C_lock == NULL || !release_lock(I2C_lock)){
    log_e("Lock not found, expected begin");
    return ESP_FAIL;
    }   
#endif 
    transmitting = false;       
}

uint8_t MasterI2C::requestData(uint16_t address, size_t rsize, bool writeRead){
    if (rxBuffer == NULL || txBuffer == NULL){
        log_e("NULL buffer pointer");
        return 0;
    }
    if(!transmitting 
#if !CONFIG_DISABLE_HAL_LOCKS
    || transmittingTask == xTaskGetCurrentTaskHandle()
#endif
    ){
        log_e("Not transmitting");
        return -1;
    }
    if(writeRead){
        rxIndex = 0;
        rxLength = 0;
        esp_err_t err = ESP_OK;
        const size_t wsize = 1;
        uint8_t reg_address[wsize] = {(uint8_t)address};
        err = i2cWriteReadNonStop(esp_i2c_num, txAddress, reg_address, wsize, rxBuffer, rsize, _timeOutMillis, &rxLength);
        if(err){
            log_e("i2cWriteReadNonStop returned Error %d", err);
        }
    } else{
        rxIndex = 0;
        rxLength = 0;
        esp_err_t err = ESP_OK;
        err = i2cRead(esp_i2c_num, address, rxBuffer, rsize, _timeOutMillis, &rxLength);
        if(err){
            log_e("i2cRead returned Error %d", err);
        }
    }
    return rxLength;
}

size_t MasterI2C::write(uint8_t data){
    if (txBuffer == NULL){
        log_e("NULL TX buffer pointer");
        return 0;
    }
    if(txLength >= bufferSize) {
        return 0;
    }
    txBuffer[txLength++] = data;
    return 1;
}

size_t MasterI2C::write(const uint8_t *data, size_t quantity)
{
    for(size_t i = 0; i < quantity; ++i) {
        if(!write(data[i])) {
            return i;
        }
    }
    return quantity;

}

int MasterI2C::available(void)
{
    int result = rxLength - rxIndex;
    return result;
}

int MasterI2C::read(void)
{
    int value = -1;
    if (rxBuffer == NULL){
        log_e("NULL RX buffer pointer");
        return value;
    }
    if(rxIndex < rxLength) {
        value = rxBuffer[rxIndex++];
    }
    return value;
}

int MasterI2C::peek(void)
{
    int value = -1;
    if (rxBuffer == NULL){
        log_e("NULL RX buffer pointer");
        return value;
    }
    if(rxIndex < rxLength) {
        value = rxBuffer[rxIndex];
    }
    return value;
}

void MasterI2C::flush(void)
{
    rxIndex = 0;
    rxLength = 0;
    txLength = 0;
    //i2cFlush(num); // cleanup
}

#if !CONFIG_DISABLE_HAL_LOCKS
bool MasterI2C::create_lock(void){
    if(I2C_lock == NULL){
        I2C_lock = xSemaphoreCreateMutex();
        if(I2C_lock == NULL){
            log_e("xSemaphoreCreateMutex failed");
            return false;
        }
        return true;
    }
    return false;
}

bool MasterI2C::acquire_lock(SemaphoreHandle_t lock){
    if(lock == NULL || xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE){
        log_e("could not acquire lock");
        return false;
    }
    return true;
}

bool MasterI2C::release_lock(SemaphoreHandle_t lock){
    if(lock == NULL){
        log_e("could not release lock");
        return false;
    }   
    if(xSemaphoreGive(lock) == pdTRUE){
        log_e("could not release lock or lock was not acquired");
        return false;
    }
    return true;
}

bool MasterI2C::create_acquire_lock(void){
    if(I2C_lock != NULL){return false;}
    if(!create_lock()){return false;}
    if(!acquire_lock(I2C_lock)){return false;}
    return true;
}
#endif