
#include "stm32f407_gpio_header.h"
#include "stm32f407xx.h"
#include "application.h"
#include "uart_header.h"

#define GREEN_LED		12
#define ORANGE_LED	13
#define RED					14
#define BLUE				15

#define USER_PUSH		0

uint8_t message1[] = "Avinash\nKumar\nSingh\nSTM32F407 Microcontroller \n ";
uint8_t message2[] = "Invalid Command !!! \n";
uint8_t message3[] = "Success !! \n";
uint8_t rx_buffer[4];

uart_handle_t uart_handle;



void led_init()
{
	gpio_pin_configuration gpio_pin_config;
	/* Enable the clock for GPIOD */
	GPIO_PORT_D_CLOCK_ENABLE();
	gpio_pin_config.pin    = GREEN_LED;
	gpio_pin_config.mode   = GPIO_PIN_OUTPUT_MODE;
	gpio_pin_config.optype = GPIO_PIN_OPTYPE_PUSHPULL;
	gpio_pin_config.speed  = GPIO_PIN_MEDIUM_SPEED;
	gpio_pin_config.pull   = GPIO_PIN_NO_PULL_UP_DOWN;
	
	stm32f407_gpio_init(GPIO_PORT_D, &gpio_pin_config);

	gpio_pin_config.pin    = ORANGE_LED;
	stm32f407_gpio_init(GPIO_PORT_D, &gpio_pin_config);

	gpio_pin_config.pin    = RED;
	stm32f407_gpio_init(GPIO_PORT_D, &gpio_pin_config);

	gpio_pin_config.pin    = BLUE;
	stm32f407_gpio_init(GPIO_PORT_D, &gpio_pin_config);
	

	
}

void led_on( uint16_t pin)
{
	stm32f407_gpio_write(GPIO_PORT_D, pin, 1);
}

void led_off(uint16_t pin)
{
	stm32f407_gpio_write(GPIO_PORT_D, pin, 0);
}

void led_toggle(uint16_t pin)
{
	if(stm32f407_gpio_read(GPIO_PORT_D,pin))
		stm32f407_gpio_write(GPIO_PORT_D, pin, 1);
	else
		stm32f407_gpio_write(GPIO_PORT_D, pin, 0);
}

void uart_gpio_init()
{
	gpio_pin_configuration uart_pin_config;
	/* enable the clock */
	GPIO_PORT_A_CLOCK_ENABLE();
	
	
	/* Configure the GPIO_A pin 2 as Tx */
	uart_pin_config.pin 		= USARTx_TX_PIN;
	uart_pin_config.mode 		= GPIO_PIN_ATL_MODE;
	uart_pin_config.optype 	= GPIO_PIN_OPTYPE_PUSHPULL;
	uart_pin_config.speed  	= GPIO_PIN_HIGH_SPEED;
	uart_pin_config.pull   	= GPIO_PIN_NO_PULL_UP_DOWN;
	stm32f407_gpio_alt_function(GPIOA,uart_pin_config.pin, 0x07);
	
	stm32f407_gpio_init(GPIOA, &uart_pin_config);
	
	/* Configure the GPIO_A pin 3 as Rx */
	uart_pin_config.pin = USARTx_RX_PIN;
	stm32f407_gpio_alt_function(GPIOA,uart_pin_config.pin, 0x07);
	
	stm32f407_gpio_init(GPIOA, &uart_pin_config);
	
	
}

void handle_cmd(uint8_t cmd)
{
	led_on(BLUE);
	if (cmd == 'H')
	{
		led_on(ORANGE_LED);
		//led_on(GREEN_LED);
		led_on(RED);
		
		stm32f407_uart_tx(&uart_handle,message3,sizeof(message3)-1);
	}
	else
	{
		led_on(ORANGE_LED);
		led_off(ORANGE_LED);
		//led_off(GREEN_LED);
		led_off(RED);
		led_off(BLUE);
		stm32f407_uart_tx(&uart_handle,message3,sizeof(message3)-1);
	}
	}

void parse_cmd( uint8_t *cmd)
{
	if(cmd[0] == 'L' && cmd[1] == 'E' && cmd[2] == 'D')
	{
		if(cmd[3] == 'H' )
			handle_cmd(cmd[3]);
		else if (cmd[3] == 'L' )
			handle_cmd(cmd[3]);
		else
			stm32f407_uart_tx(&uart_handle,message2,sizeof(message2)-1);	
		
	}
}

/*This callback will be called by the driver when the application receives the command */
void app_rx_cmp_callback(void *size)
{
	led_on(BLUE);
	//we got a command,  parse it 
	parse_cmd(rx_buffer);
	
}

int  main()
{
	
	/* Led init */
	led_init();
	
	/* configure the GPIO for Uart functionality */
	uart_gpio_init();
	
	_UART2_CLOCK_ENABLE();
	
	uart_handle.Instance = USART2;
	
	
	uart_handle.init.BaudRate 		= USART_BAUD_9600;
	uart_handle.init.Wordlenth 		= CONFIG_8DATA_BIT;
	uart_handle.init.Oversampling = OVERSAMPLING_BY_16;
	uart_handle.init.Parity				= PARITY_NONE;
	uart_handle.init.Stopbits			= USART_STOP_BITS_1 ;
	
	/*fill out the application callbacks */
	uart_handle.rx_cmp_cb = app_rx_cmp_callback;
	stm32f407_uart_init(&uart_handle);
	
	/*enable the IRQ of USART2 peripheral */
	NVIC_EnableIRQ(USART2_IRQn);
	
	while(uart_handle.tx_state != UART_STATE_READY);
	/*send the Debug message */
	stm32f407_uart_tx(&uart_handle,message1, sizeof(message1)-1);
	
	
	while(1)
		if(uart_handle.tx_state != UART_STATE_BUSY_TX)
			break;
	
	//led_on(GREEN _LED);
	/* Reacive the data */
	while (1)
	{
		while(uart_handle.rx_state != UART_STATE_READY);
		stm32f407_uart_rx(&uart_handle,rx_buffer,5);
	}
	return 0;
}

void USART2_IRQHandler(void)
{
	stm32f407_uart_interrupt_handle(&uart_handle);
}
