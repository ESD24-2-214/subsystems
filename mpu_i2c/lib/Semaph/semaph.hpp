#include <Arduino.h>
#include <Config.hpp>
#if !CONFIG_DISABLE_HAL_LOCKS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#endif


#ifndef SEMAPH
#define SEMAPH

#if !CONFIG_DISABLE_HAL_LOCKS
    bool create_lock(SemaphoreHandle_t *lock);
    bool acquire_lock(SemaphoreHandle_t lock);
    bool release_lock(SemaphoreHandle_t lock);
    bool create_acquire_lock(SemaphoreHandle_t *lock);
#endif

#endif