


#define size_t unsigned int

class MasterI2C
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
    bool continuous;
#if !CONFIG_DISABLE_HAL_LOCKS
    TaskHandle_t continuousTask;
    SemaphoreHandle_t lock;
#endif

private:
    bool initPins(int sdaPin, int sclPin);
    bool allocateWireBuffer(void);
    void freeWireBuffer(void);

public:
    
    MasterI2C(uint8_t bus_num);
    ~MasterI2C(void);

    bool setPins(int sdaPin, int sclPin);

    // returns true, if successful init of i2c bus
    bool begin(int sdaPin, int sclPin, uint32_t frequency=0); 


};