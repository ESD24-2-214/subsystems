#ifndef SEMAPH
#define SEMAPH
#include <Arduino.h>
#include <Config.hpp>
#include <DEBUG.hpp>
#if !CONFIG_DISABLE_HAL_LOCKS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#endif


#if !CONFIG_DISABLE_HAL_LOCKS
    bool create_lock(SemaphoreHandle_t *lock);
    bool acquire_lock(SemaphoreHandle_t lock);
    bool release_lock(SemaphoreHandle_t lock);
    bool create_acquire_lock(SemaphoreHandle_t *lock);
#endif

#endif