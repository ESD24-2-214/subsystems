#ifndef CONFIG
#define CONFIG

#define gyro_scale_factor250 (1/131.0) // 250 degrees/sec LSB/(degrees/sec)
#define gyro_scale_factor500 (1/65.5) // 500 degrees/sec LSB/(degrees/sec)
#define gyro_scale_factor1000 (1/32.8) // 1000 degrees/sec LSB/(degrees/sec)
#define gyro_scale_factor2000 (1/16.4) // 2000 degrees/sec LSB/(degrees/sec)

#define acc_scale_factor2g (1/16384.) // 2g LSB/(g)
#define acc_scale_factor4g (1/8192.) // 4g LSB/(g)
#define acc_scale_factor8g (1/4096.) // 8g LSB/(g)
#define acc_scale_factor16g (1/2048.) // 16g LSB/(g)

#define mag_scale_factor1 0.6 // 0.6uT/LSB
#define mag_scale_factor2 0.15 // 0.15uT/LSB

#define LDR_MAX 1023 // LDR max value
#define LDR_MIN 0 // LDR min value
#define LDR_PIN_F 4 // LDR GPIO pin number
#define LDR_PIN_L 1 // LDR GPIO pin number
#define LDR_PIN_R 2 // LDR GPIO pin number
#define LDR_PIN_B 0 // LDR GPIO pin number
#define LDR_SAMPLES 10 // Number of samples to average for LDR reading

#endif