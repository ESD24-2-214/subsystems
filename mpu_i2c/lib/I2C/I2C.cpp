extern "C" {
    #include <stdlib.h>
    #include <string.h>
    #include <inttypes.h>
    }
    //#include <esp32-hal-log.h>
    #include <I2C.hpp>
    #include <esp32-hal-i2c.h>
    #include <esp32-hal-i2c-slave.h>
    #include <semaph.hpp>
    #include <Arduino.h>
    
    

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
    ,_timeOutMillis(500)
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
    if(!create_acquire_lock(&(this->I2C_lock))){return false;}
#endif

    if(!i2cIsInit(esp_i2c_num)){
        initPins(sdaPin, sclPin);
    } else {
        DEBUG("bus already initialized. change pins only when not."," ");
    }

#if !CONFIG_DISABLE_HAL_LOCKS 
    release_lock(this->I2C_lock); 
#endif
    return i2cIsInit(esp_i2c_num);
}    

bool MasterI2C::allocateI2CBuffer(void){
    // both buffer can be allocated or none will be
    if (rxBuffer == NULL) {
        rxBuffer = (uint8_t *)malloc(bufferSize);
        if (rxBuffer == NULL) {
            DEBUG("Can't allocate memory for rxBuffer I2C_", esp_i2c_num);
            return false;
        }
    }
    if (txBuffer == NULL) {
        txBuffer = (uint8_t *)malloc(bufferSize);
        if (txBuffer == NULL) {
            DEBUG("Can't allocate memory for rxBuffer I2C_", esp_i2c_num);
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
        DEBUG("Minimum Wire Buffer size is 32 bytes"," ");
        return 0;
    }
#if !CONFIG_DISABLE_HAL_LOCKS
    if(!create_acquire_lock(&(this->I2C_lock))){return false;}
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
                DEBUG("Buffer allocation failed"," ");
            }
        } // else nothing changes, all set!
    } else {
        // no memory allocated yet, just change the size value - allocation in begin()
        bufferSize = bSize;
    }
#if !CONFIG_DISABLE_HAL_LOCKS
    release_lock(this->I2C_lock);
#endif
    return bSize;
}

bool MasterI2C::begin(int sdaPin, int sclPin, uint32_t frequency){
    bool started = false;
    esp_err_t err = ESP_OK;
#if !CONFIG_DISABLE_HAL_LOCKS
    if(!create_acquire_lock( &(this->I2C_lock) ) ){return false;}
#endif
    if(i2cIsInit(esp_i2c_num)){
        DEBUG("Bus already started."," ");
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
    release_lock(this->I2C_lock);
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
    release_lock(this->I2C_lock);
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
    release_lock(this->I2C_lock);
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
    release_lock(this->I2C_lock);
#endif
    return (err == ESP_OK);
}

void MasterI2C::setTimeOut(uint16_t timeOutMillis)
{
    _timeOutMillis = timeOutMillis;
}

void MasterI2C::beginTransmission(uint16_t address){

#if !CONFIG_DISABLE_HAL_LOCKS
    if(transmitting || transmittingTask == xTaskGetCurrentTaskHandle() ){
    DEBUG("Expected endTransmission not beginTransmission! Clearing..."," ");
    //release lock
    release_lock(this->I2C_lock);
    }
    if(I2C_lock == NULL || acquire_lock(I2C_lock) != pdTRUE){
        DEBUG("could not acquire lock"," ");
        return;
    }
#endif
    transmitting = true;
    transmittingTask = xTaskGetCurrentTaskHandle();
    txAddress = address;
    txLength = 0;
}

uint8_t MasterI2C::transmitWrite(){
    if(!transmitting 
#if !CONFIG_DISABLE_HAL_LOCKS       
        || !(transmittingTask == xTaskGetCurrentTaskHandle())
#endif    
    ){
        DEBUG("Transmission not startet. Expected beginTransmission"," ");
        return 4;
    }
    if (txBuffer == NULL){
        DEBUG("NULL TX buffer pointer"," ");
        return 4;
    }
    esp_err_t err = ESP_OK;
    err = i2cWrite(esp_i2c_num, txAddress, txBuffer, txLength, _timeOutMillis);
    txLength = 0;
    switch(err){
        case ESP_OK: return 0;
        case ESP_FAIL: return 2;
        case ESP_ERR_TIMEOUT: return 5;
        default: break;
    }
    return 4;
}

bool MasterI2C::endTransmission(void){
    if(transmitting == false){
        DEBUG("Transmission not startet, expected beginTransmission"," ");
        return false;
    }
#if !CONFIG_DISABLE_HAL_LOCKS
    if(this->I2C_lock == NULL || !release_lock(this->I2C_lock)){
    DEBUG("Lock not found, expected begin"," ");
    return false;
    }   
#endif 
    transmitting = false;
    transmittingTask = NULL;
    return true;
}

uint8_t MasterI2C::requestData(size_t rsize, bool writeRead, uint16_t address){
    if (rxBuffer == NULL || txBuffer == NULL){
        DEBUG("NULL buffer pointer"," ");
        return 0;
    }
    if(!transmitting 
#if !CONFIG_DISABLE_HAL_LOCKS
    || this->transmittingTask != xTaskGetCurrentTaskHandle()
#endif
    ){
        DEBUG("Not transmitting"," ");
        return -1;
    }
    if(writeRead){
        rxIndex = 0;
        rxLength = 0;
        esp_err_t err = ESP_OK;
        err = i2cWriteReadNonStop(esp_i2c_num, txAddress, txBuffer, txLength, rxBuffer, rsize, _timeOutMillis, &rxLength);
        if(err){;
        DEBUG("i2cWriteReadNonStop returned Error: ", err);
        }
        txLength = 0;
    } else{
        rxIndex = 0;
        rxLength = 0;
        esp_err_t err = ESP_OK;
        err = i2cRead(esp_i2c_num, address, rxBuffer, rsize, _timeOutMillis, &rxLength);
        if(err){
            DEBUG("i2cRead returned Error: ", err);
        }
    }
    return rxLength;
}

size_t MasterI2C::write(uint8_t data){
    if (txBuffer == NULL){
        DEBUG("NULL TX buffer pointer", " ");
        return 0;
    }
    if(txLength >= bufferSize) {
        DEBUG("TX buffer full", " ");
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
        DEBUG("NULL RX buffer pointer"," ");
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
        DEBUG("NULL RX buffer pointer"," ");
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
}



