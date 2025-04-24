#include <semaph.hpp>

#if !CONFIG_DISABLE_HAL_LOCKS
bool create_lock(SemaphoreHandle_t *lock){
    if(*lock == NULL){
        DEBUG2("Creating....")
        *lock = xSemaphoreCreateMutex();
        if(*lock == NULL){
            DEBUG("xSemaphoreCreateMutex failed"," ");
            return false;
        }
        DEBUG2("Created....")
        return true;
    }
    return false;
    DEBUG("Semaphore exist, lock != NULL"," ");
}

bool acquire_lock(SemaphoreHandle_t lock){
    if(lock == NULL || (xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE) ){
        DEBUG("could not acquire lock"," ");
        return false;
    }
    return true;
}

bool release_lock(SemaphoreHandle_t lock){
    if(lock == NULL){
        DEBUG("could not release lock"," ");
        return false;
    }   
    if(xSemaphoreGive(lock) != pdTRUE){
        DEBUG("could not release lock or lock was not acquired"," ");
        return false;
    }
    return true;
}

bool create_acquire_lock(SemaphoreHandle_t *lock){
    if(*lock != NULL){return false;}
    if(!create_lock(lock)){return false;}
    if(!acquire_lock(*lock)){return false;}
    return true;
}
#endif