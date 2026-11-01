#ifndef TEST_FLASH_HAL_H
#define TEST_FLASH_HAL_H
#include <stdint.h>
typedef enum{HAL_OK,HAL_ERROR} HAL_StatusTypeDef;
#define FLASH_TYPEPROGRAM_WORD 0U
#define FLASH_FLAG_EOP 1U
#define FLASH_FLAG_OPERR 2U
#define FLASH_FLAG_WRPERR 4U
#define FLASH_FLAG_PGAERR 8U
#define FLASH_FLAG_PGPERR 16U
#define FLASH_FLAG_PGSERR 32U
#define __HAL_FLASH_CLEAR_FLAG(x) ((void)(x))
#define __HAL_FLASH_DATA_CACHE_DISABLE() ((void)0)
#define __HAL_FLASH_DATA_CACHE_RESET() ((void)0)
#define __HAL_FLASH_DATA_CACHE_ENABLE() ((void)0)
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type,uintptr_t address,uint64_t data);
#endif
