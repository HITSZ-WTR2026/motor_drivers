/**
 * @file    gpio_driver.h
 * @author  syhanjin
 * @date    2025-09-10
 * @brief   gpio driver
 */
#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H
#include "main.h"

typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
} GPIO_t;

static inline void GPIO_WritePin(GPIO_t* hgpio, const GPIO_PinState PinState)
{
    HAL_GPIO_WritePin(hgpio->port, hgpio->pin, PinState);
}

static inline void GPIO_SetPin(GPIO_t* hgpio) { GPIO_WritePin(hgpio, GPIO_PIN_SET); }

static inline void GPIO_ResetPin(GPIO_t* hgpio) { GPIO_WritePin(hgpio, GPIO_PIN_RESET); }

static inline void GPIO_TogglePin(GPIO_t* hgpio) { HAL_GPIO_TogglePin(hgpio->port, hgpio->pin); }

#endif // GPIO_DRIVER_H
