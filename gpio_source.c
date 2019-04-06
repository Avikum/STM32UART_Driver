
#include "stm32f407_gpio_header.h"
#include "stm32f407xx.h"



/* Initialize the GPIO */

void stm32f407_gpio_init(GPIO_TypeDef *GPIOx, gpio_pin_configuration *gpio_pin_config)
{

	GPIOx->MODER   |= (gpio_pin_config->mode << (2 * gpio_pin_config->pin));
	GPIOx->OTYPER  |= (gpio_pin_config->optype << gpio_pin_config->pin );
	GPIOx->OSPEEDR |= (gpio_pin_config->speed << (2 * gpio_pin_config->pin));
	GPIOx->PUPDR   |= (gpio_pin_config->pull << (2 * gpio_pin_config->pin));


}

uint8_t stm32f407_gpio_read(GPIO_TypeDef *GPIOx, uint16_t pin)
{
	uint8_t val;
	val = ( GPIOx->IDR >> pin ) & 0x00000001 ;
	return val;
}

void stm32f407_gpio_write(GPIO_TypeDef *GPIOx, uint16_t pin, uint8_t val)
{
	if(val)
	GPIOx->ODR |= (1 << pin );
	else
	GPIOx->ODR &= ~(1 << pin );

}
/* GPIO interrupt APIs */
void gpio_configure_interrupt(uint16_t pin, edge_select edge)
{
	if( edge == RISING_EDGE )
		EXTI->RTSR |= (1 << pin);
	else if ( edge == FALLING_EDGE )
		EXTI->FTSR |= (1 << pin);
	else if (edge == RISING_FALLING_EDGE )
	{
		EXTI->RTSR |= (1 << pin);
		EXTI->FTSR |= (1 << pin);
	}
	else 
	{
	
	}
}

void gpio_enable_interrupt(uint16_t pin, IRQn_Type irq_no )
{
	EXTI->IMR |= (1 << pin);
	NVIC_EnableIRQ(irq_no);
	
}

void gpio_clear_pendingbit(uint16_t pin )
{
	if(EXTI->PR & (1 << pin ))
	{
		EXTI->PR |= (1 << pin);
	}
}

/* Configure teh alternate functionality of GPIO */
void stm32f407_gpio_alt_function(GPIO_TypeDef *GPIOx, uint16_t pin, uint16_t alt_fun_value)
{
	if( pin <= 7)
		GPIOx->AFR[0] |= (alt_fun_value << ( 4* pin)); // lower regiser for pin 0 to pin 7 
	else
		GPIOx->AFR[1] |= (alt_fun_value << ( 4* (pin % 8))); // higher register for pin 8 to pin 16=5
}

