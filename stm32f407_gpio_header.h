
#include "stm32f407xx.h"

/* Configure GPIOx port mode register */

#define GPIO_PIN_INPUT_MODE			((uint32_t)0x00 )
#define GPIO_PIN_OUTPUT_MODE		((uint32_t)0x01 )
#define GPIO_PIN_ATL_MODE				((uint32_t)0x02 )
#define GPIO_PIN_ANA_MODE				((uint32_t)0x03 )


/* Configure GPIOx port output type register */

#define GPIO_PIN_OPTYPE_PUSHPULL	((uint32_t)0x00)
#define GPIO_PIN_OPTYPE_OPENDRAIN	((uint32_t)0x01)

/* Configure GPIOx port output speed register */

#define GPIO_PIN_LOW_SPEED				((uint32_t)0x00)
#define GPIO_PIN_MEDIUM_SPEED			((uint32_t)0x01)
#define GPIO_PIN_HIGH_SPEED				((uint32_t)0x02)
#define GPIO_PIN_VERY_HIGH_SPEED	((uint32_t)0x03)

/* Configure GPIO port pull-up/pull-down register */

#define GPIO_PIN_NO_PULL_UP_DOWN	((uint32_t)0x00)
#define GPIO_PIN_PULLUP						((uint32)0x01)
#define GPIO_PIN_PULLDOWN					((uint32)0x02)

/* GPIO port Address */

#define GPIO_PORT_A	GPIOA
#define GPIO_PORT_B	GPIOB
#define GPIO_PORT_C	GPIOC
#define GPIO_PORT_D	GPIOD
#define GPIO_PORT_E	GPIOE
#define GPIO_PORT_F	GPIOF
#define GPIO_PORT_G	GPIOG
#define GPIO_PORT_H	GPIOH
#define GPIO_PORT_I	GPIOI

/* Enable the clock for the GPIOx using RCC AHB1 Register */
#define GPIO_PORT_A_CLOCK_ENABLE()	RCC->AHB1ENR |= (1 << 0)
#define GPIO_PORT_B_CLOCK_ENABLE()	RCC->AHB1ENR |= (1 << 1)
#define GPIO_PORT_C_CLOCK_ENABLE()	RCC->AHB1ENR |= (1 << 2)
#define GPIO_PORT_D_CLOCK_ENABLE()	RCC->AHB1ENR |= (1 << 3)
#define GPIO_PORT_E_CLOCK_ENABLE()	RCC->AHB1ENR |= (1 << 4)
#define GPIO_PORT_F_CLOCK_ENABLE()	RCC->AHB1ENR |= (1 << 5)
#define GPIO_PORT_G_CLOCK_ENABLE()	RCC->AHB1ENR |= (1 << 6)
#define GPIO_PORT_H_CLOCK_ENABLE()	RCC->AHB1ENR |= (1 << 7)
#define GPIO_PORT_I_CLOCK_ENABLE()	RCC->AHB1ENR |= (1 << 8)

typedef struct 
{
	uint32_t pin;
	uint32_t mode;
	uint32_t optype;
	uint32_t speed;
	uint32_t pull;
	uint32_t alt;
} gpio_pin_configuration;

/*  Interrupt Edgle selection mode */
typedef enum 
{
	RISING_EDGE,
	FALLING_EDGE,
	RISING_FALLING_EDGE
}	edge_select;

/* Exposed APIs for application */

/* Initialize the GPIO */

void stm32f407_gpio_init(GPIO_TypeDef *GPIOx, gpio_pin_configuration *gpio_pin_config);

/* Write date to GPIO Pin */

void stm32f407_gpio_write(GPIO_TypeDef *GPIOx, uint16_t pin, uint8_t val); 


/* Read date from GPIO Pin */

uint8_t stm32f407_gpio_read(GPIO_TypeDef *GPIOx, uint16_t pin); 

void stm32f407_gpio_alt_function(GPIO_TypeDef *GPIOx, uint16_t pin, uint16_t alt_fun_value);

/* GPIO Interrupt exposed APIs */

void gpio_configure_interrupt(uint16_t pin, edge_select edge);

void gpio_enable_interrupt(uint16_t pin, IRQn_Type irq_no );

void gpio_clear_pendingbit(uint16_t pin );
