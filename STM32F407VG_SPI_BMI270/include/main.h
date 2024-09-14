/**
 ******************************************************************************
 * @file    UART/UART_TwoBoards_ComPolling/Inc/main.h
 * @author  MCD Application Team
 * @brief   Header for main.c module
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes -----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4_discovery.h"
#include "bmp388.h"
#include <stdio.h>

/* Exported types -----------------------------------------------------------*/
/* Exported constants -------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__) (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */