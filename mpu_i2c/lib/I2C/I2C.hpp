
#ifndef I2C_Hpp
#define I2C_Hpp

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <esp32-hal.h>
#include <esp32-hal-i2c.h>
#if !CONFIG_DISABLE_HAL_LOCKS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#endif
#include "Stream.h"

#define size_t unsigned int

#ifndef I2C_BUFFER_LENGTH
    #define I2C_BUFFER_LENGTH 128  // Default size, if none is set using Wire::setBuffersize(size_t)
#endif

// #define log_e should be defined if not using aruino IDE error logging

static const uint8_t SDA = 18; // default SDA pin for lilygo-ESP32-s3
static const uint8_t SCL = 17; // default SCL pin for lilygo-ESP32-s3

class MasterI2C : public Stream
{
    protected:
    uint8_t esp_i2c_num;
    int8_t sda;
    int8_t scl;

    size_t bufferSize;
    uint8_t *rxBuffer;
    size_t rxIndex;
    size_t rxLength;

    uint8_t *txBuffer;
    size_t txLength;
    uint16_t txAddress;

    uint32_t _timeOutMillis;
    bool transmitting;
#if !CONFIG_DISABLE_HAL_LOCKS
    TaskHandle_t transmittingTask;
    SemaphoreHandle_t I2C_lock;
#endif

private:
    bool initPins(int sdaPin, int sclPin);
    bool allocateI2CBuffer(void);
    void freeI2CBuffer(void);
#if !CONFIG_DISABLE_HAL_LOCKS
    bool create_lock(void);
    bool acquire_lock(SemaphoreHandle_t lock);
    bool release_lock(SemaphoreHandle_t lock);
    bool create_acquire_lock(void);
#endif
public:
    
    MasterI2C(uint8_t bus_num);
    ~MasterI2C(void);

    bool setPins(int sdaPin, int sclPin);

    // returns true, if successful init of i2c bus
    bool begin(int sdaPin, int sclPin, uint32_t frequency=0); 

    bool hasStarted(void); // returns true, if successful init of i2c bus

    bool end(void);

    size_t setBufferSize(size_t bSize);

    void setTimeOut(uint16_t timeOutMillis); // default timeout of i2c transactions is 50ms
    uint16_t getTimeOut();

    bool setClock(uint32_t);
    uint32_t getClock();

    void beginTransmission(uint16_t address);

    uint8_t transmit();

    uint8_t endTransmission(void);

    size_t write(uint8_t data);
    size_t write(const uint8_t *data, size_t quantity);

    uint8_t requestData(uint16_t address, size_t size, bool sendStop);

    int available(void);
    int read(void);
    int peek(void);
    void flush(void);


};



#endif // I2C_Hpp