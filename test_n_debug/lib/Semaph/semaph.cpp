#include <semaph.hpp>

#if !CONFIG_DISABLE_HAL_LOCKS
/* @brief This function is used to create a semaphore Mutex lock
** @param lock: the pointer to the semaphore handle
** @note This function will return false if the lock is not NULL or if the lock could not be created
*/
bool create_lock(SemaphoreHandle_t *lock){
    if(*lock == NULL){
        PRINT_DEBUG("Creating....\n")
        *lock = xSemaphoreCreateMutex();
        if(*lock == NULL){
            ERROR_LOG("xSemaphoreCreateMutex failed"," ");
            return false;
        }
        PRINT_DEBUG("Created....\n")
        return true;
    }
    return false;
    ERROR_LOG("Semaphore exist, lock != NULL"," ");
}

/* @brief This function is used to acquire a semaphore lock
** @param lock: the semaphore handle
** @note This function will block until the lock is acquired.
** @note This function will return false if the lock is NULL or if the lock could not be acquired.
** @note The block time/wait time for acquireing the lock is set to max (portMAX_DELAY).
*/
bool acquire_lock(SemaphoreHandle_t lock){
    if(lock == NULL || (xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE) ){
        ERROR_LOG("could not acquire lock"," ");
        return false;
    }
    return true;
}

/* @brief This function is used to release a semaphore lock
** @param lock: the semaphore handle
** @note This function will return false if the lock is NULL or if the lock could not be released
*/
bool release_lock(SemaphoreHandle_t lock){
    if(lock == NULL){
        ERROR_LOG("could not release lock"," ");
        return false;
    }   
    if(xSemaphoreGive(lock) != pdTRUE){
        ERROR_LOG("could not release lock or lock was not acquired"," ");
        return false;
    }
    return true;
}

/* @brief This function is used to create and acquire a semaphore lock
** @param lock: the pointer to the semaphore handle
** @note This function will return false if somthing goes wrong or if the lock is not NULL else true
** @note This function will create the lock if it does not exist and acquire it
** @note See create_lock() and acquire_lock() for more details
*/
bool create_acquire_lock(SemaphoreHandle_t *lock){
    if(*lock != NULL){return false;}
    if(!create_lock(lock)){return false;}
    if(!acquire_lock(*lock)){return false;}
    return true;
}
#endif