#include <stdint.h>
#include "stm32f407xx.h"


/****************************** USART Control Registe 1 *************************/

#define USART_CR1_REG_RE_ENABLE								((uint32_t) 1<<2)
#define USART_CR1_REG_TE_ENABLE								((uint32_t) 1<<3)

#define USART_CR1_REG_IDLE_INTERRUPT_ENABLE		((uint32_t) 1<<4)
#define USART_CR1_REG_RXNE_INTERRUPT_ENABLE		((uint32_t) 1<<5)
#define USART_CR1_REG_TCIE_INTERRUPT_ENABLE		((uint32_t) 1<<6)
#define	USART_CR1_REG_TXE_INTERRUPT_ENABLE    ((uint32_t) 1<<7)
#define USART_CR1_REG_PE_INTERRUPT_ENABLE 		((uint32_t) 1<<8) 	// PE interrupt enable
 
#define USART_CR1_REG_PARITY									((uint32_t) 1 <<9) 	//	Parity selection ( Even = 0, Odd = 1)
#define EVEN_PARITY														0
#define ODD_PARITY														1

#define USART_CR1_REG_PARITY_CTRL_ENABLE			((uint32_t) 1<<10)
#define PARITY_NONE														0
#define PARITY_ENABLE													1

#define USART_CR1_REG_CONFIG_WORD							((uint32_t) 1<<12)
#define CONFIG_8DATA_BIT											0
#define CONFIG_9DATA_BIT											1

#define USART_CR1_REG_USART_ENABLE						((uint32_t) 1<<13)

#define USART_CR1_REG_CONFIG_OVERSAMLPING_MODE	((uint32_t) 1<<15)
#define OVERSAMPLING_BY_16										0
#define OVERSAMPLING_BY_8											1

/*******************  Bit definition for USART_CR2 register  ********************/
#define USART_REG_CR2_STOP_BITS                 	 12   
#define USART_STOP_BITS_1                          (uint32_t)0
#define USART_STOP_BITS_HALF                       (uint32_t)1
#define USART_STOP_BITS_2                          (uint32_t)2
#define USART_STOP_BITS_1NHALF                     (uint32_t)3

/****************************** USART BRR register ******************************/



/***************************** Bit definition for USART_CR3 register **************/
#define USART_CR3_REG_ERR_INT_EN				((uint32_t) 1<<0)

#define UART_STOPBITS_1        										( (uint32_t)0x00 )
#define UART_STOPBITS_HALF     										( (uint32_t)0x01 )
#define UART_STOPBITS_2        										( (uint32_t)0x02 )
#define UART_STOPBITS_ONENHALF 										( (uint32_t)0x03 )

#define USART_BAUD_9600                   				(uint32_t)9600
#define USART_BAUD_115200                 				(uint32_t)115200
#define USART_BAUD_2000000                				(uint32_t)2000000


/******************************** USART Status Register **********************/

#define USART_SR_REG_PARITY_ERROR					((uint32_t) 1<<0)
#define USART_SR_REG_FRAMEING_ERROR				((uint32_t) 1<<1)
#define USART_SR_REG_OVERRUN_ERROR				((uint32_t) 1<<3)
#define USART_SR_REG_IDLE_LINE						((uint32_t) 1<<4)
#define USART_SR_REG_RXNE									((uint32_t) 1<<5)
#define USART_SR_REG_TC_COMPLETE					((uint32_t) 1<<6) // Transmission complete 
#define USART_SR_REG_TXE									((uint32_t) 1<<7)

/** 
  * @brief HAL UART State structures definition  
  */ 
typedef enum
{
  UART_STATE_RESET             = 0x00,    /*!< Peripheral is not yet Initialized                  */
  UART_STATE_READY             = 0x01,    /*!< Peripheral Initialized and ready for use           */
  UART_STATE_BUSY              = 0x02,    /*!< an internal process is ongoing                     */   
  UART_STATE_BUSY_TX           = 0x12,    /*!< Data Transmission process is ongoing               */ 
  UART_STATE_BUSY_RX           = 0x22,    /*!< Data Reception process is ongoing                  */
  UART_STATE_BUSY_TX_RX        = 0x32,    /*!< Data Transmission and Reception process is ongoing */        
}uart_state_t;

/****************************************** UART possible erros **********************************/
#define UART_ERROR_NONE							((uint32_t) 0x00000000)
#define UART_ERROR_PE								((uint32_t) 0x00000001)
#define UART_ERROR_NE								((uint32_t) 0x00000002)
#define UART_ERROR_FE								((uint32_t) 0x00000004) 		// frame error
#define UART_ERROR_ORE							((uint32_t) 0x00000008)  		// Overrun error
#define	UART_ERROR_DMA							((uint32_t) 0x00000010)   	// DMA transfer error


/* Macros to enable Clock for USARTs */

#define	_UART1_CLOCK_ENABLE()					RCC->APB2ENR |= (1<<4);
#define	_UART6_CLOCK_ENABLE()					RCC->APB2ENR |= (1<<5);
#define	_UART2_CLOCK_ENABLE()					RCC->APB1ENR |= (1<<17);
#define	_UART3_CLOCK_ENABLE()					RCC->APB1ENR |= (1<<18);
#define	_UART4_CLOCK_ENABLE()					RCC->APB1ENR |= (1<<19);
#define	_UART5_CLOCK_ENABLE()					RCC->APB1ENR |= (1<<20);


/**
 * UART Init structure definition
*/

typedef struct 
{
	uint32_t BaudRate;				// Config UART communication baud rate 
	uint32_t Wordlenth;				// Config number of data byte transmitted or received
	uint32_t Stopbits;				// Specifies the number of stop bits
	uint32_t Parity;					// Specifies the parity mode ( enable or Disable)
	uint32_t Mode;						// Specifies whether the Receiver or transmitter mode
	uint32_t Oversampling;		// Specifies the oversampling 8 or 16 

} uart_init_t;




/*Application callbacks typedef */
typedef void (	TX_COMP_CB_t) (void *ptr);
typedef void (	RX_COMP_CB_t) (void *ptr);

/**
 * UART handle structure definition
*/
typedef struct
{
	USART_TypeDef 				*Instance;  // uart register base address
	uart_init_t    				init;	  // Uart communication parameter
	uint8_t		   					*pTxbuffer;
	uint16_t       				TxXfersize;
	uint16_t       				TxXfercount;
	uint8_t		  			 		*pRbuffer;
	uint16_t      				Rxfersize;
	uint16_t      			 	RxFercount;
	uart_state_t          rx_state;         /* UART communication state           */
	uart_state_t         	tx_state;         /* UART communication state           */ 
	uint32_t	  					ErrorCode;
	TX_COMP_CB_t          *tx_cmp_cb ;      /* Application call back when tx completed */	
	RX_COMP_CB_t          *rx_cmp_cb ;      /* Application callback when RX Completed */
	
}uart_handle_t;

/******************************************************************************/
/*                                                                            */
/*                      Driver exposed APIs                                   */
/*                                                                            */
/******************************************************************************/


void stm32f407_uart_init(uart_handle_t *handle);

void stm32f407_uart_tx(uart_handle_t *handle, uint8_t *buffer, uint32_t len);

void stm32f407_uart_rx( uart_handle_t *handle,uint8_t *buffer, uint32_t len);


/**
  * @brief  This API handles UART interrupt request.
  * @param  huart: pointer to a uart_handle_t structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void stm32f407_uart_interrupt_handle(uart_handle_t *uart);