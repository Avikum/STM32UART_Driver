
#include "uart_header.h"

static void config_wordlength( USART_TypeDef *uartx, uint32_t wordlength)
{
	if(wordlength)
	{
		uartx->CR1 |= USART_CR1_REG_CONFIG_WORD;
	}
	else 
	{
		uartx->CR1 &= ~USART_CR1_REG_CONFIG_WORD;
	}
}

static void config_oversampling(USART_TypeDef *uartx, uint32_t oversampling)
{
	if(oversampling)
	{
		uartx->CR1 |= USART_CR1_REG_CONFIG_OVERSAMLPING_MODE; // Oversampling By 8
	}
	else
	{
		uartx->CR1 &= ~USART_CR1_REG_CONFIG_OVERSAMLPING_MODE;  // Oversampling By 16
	}
	
}

static void config_parity( USART_TypeDef *uartx, uint32_t parity)
{
	if(parity)
	{
		uartx->CR1 |= USART_CR1_REG_PARITY;  // Odd Parity
	}
	else
	{
		uartx->CR1 &= ~USART_CR1_REG_PARITY; // Even parity
	}
}

static void config_stopbits(USART_TypeDef *uartx, uint32_t stopbits)
{
	//clear the 12th and 13 th bits 
	uartx->CR2 &= ~( 0x3 << USART_REG_CR2_STOP_BITS);
	
	if( stopbits == USART_STOP_BITS_1)
		uartx->CR2 |= (0x00 <<USART_REG_CR2_STOP_BITS); // 1 stop bit
	else if ( stopbits == USART_STOP_BITS_HALF)
		uartx->CR2 |= (0x01 << USART_REG_CR2_STOP_BITS); //0.5 stop bit
	else if ( stopbits == UART_STOPBITS_2  )
		uartx->CR2 |= (0x02 << USART_REG_CR2_STOP_BITS); //2 stop bit
	else 
		uartx->CR2 |= (0x03 << USART_REG_CR2_STOP_BITS);  // 1.5 stop bit 
}

static void config_baudrate(USART_TypeDef *uartx, uint32_t baudrate)
{
	
	if( baudrate == USART_BAUD_9600)
		uartx->BRR |= 0x683;
	else if (baudrate == USART_BAUD_115200)
		uartx->BRR |= 0x8A;
	else
		uartx->BRR |= 0x8A;
}

static void enable_uart_peripheral(USART_TypeDef *uartx)
{
	uartx->CR1 |= USART_CR1_REG_USART_ENABLE;
}

static void disable_uart_peripheral(USART_TypeDef *uartx)
{
	uartx->CR1 &= ~USART_CR1_REG_USART_ENABLE;
}

static void config_receiver(USART_TypeDef *uartx, uint32_t re)
{
	if(re)
		uartx->CR1 |= USART_CR1_REG_RE_ENABLE;
	else
		uartx->CR1 &= ~USART_CR1_REG_RE_ENABLE;
}

static void config_transmitter(USART_TypeDef *uartx, uint32_t te)
{
	if(te)
		uartx->CR1 |= USART_CR1_REG_TE_ENABLE;
	else
		uartx->CR1 &= ~USART_CR1_REG_TE_ENABLE;
}

/**
	* @brief  API to do UART Peripheral initialization   
	* @param  *handle : pointer to the handle structure of the UART peripheral  
	* @retval None
	*/	
void stm32f407_uart_init(uart_handle_t *handle)
{
	/* Config Word length  */
	config_wordlength(handle->Instance,handle->init.Wordlenth);
	
	/* Config Oversampling */
	config_oversampling(handle->Instance,handle->init.Oversampling);
	
	/* Config Parity */
	//config_parity(handle->Instance,handle->init.Parity);
	
  /*Enable the Receive block of the UART Peripheral */ 
	config_receiver(handle->Instance,1);
	
	/*Enable the Transmitter block of the UART Peripheral */
	config_transmitter(handle->Instance,1);
	
	/*  Config  uart Stopbits */
	config_stopbits(handle->Instance,handle->init.Stopbits);
	
	/* Config uart Baud Rate */
	config_baudrate(handle->Instance,handle->init.BaudRate);
	
	/* Enable Uart */
	enable_uart_peripheral(handle->Instance);
	
	handle->tx_state  = UART_STATE_READY;
	handle->rx_state  = UART_STATE_READY;
	handle->ErrorCode = UART_ERROR_NONE;
	
}

static void config_txe_interrupt(USART_TypeDef *uartx, uint32_t ir_control)
{
	if(ir_control)
		uartx->CR1 |= USART_CR1_REG_TXE_INTERRUPT_ENABLE;
	else
		uartx->CR1 &= ~USART_CR1_REG_TXE_INTERRUPT_ENABLE;
		
}

/**
	* @brief  API to do UART data Transmission
	* @param  *uart_handle : pointer to the handle structure of the UART Peripheral 
  * @param  *buffer : holds the pointer to the TX buffer 
  * @param  len : len of the data to be TXed
	* @retval None
	*/
void stm32f407_uart_tx(uart_handle_t *handle, uint8_t *buffer, uint32_t len)
{
	handle->pTxbuffer = buffer;
	handle->TxXfersize = len;
	handle->TxXfercount =len;
	
	/* Enable Uart */
	enable_uart_peripheral(handle->Instance);
	
	handle->tx_state = UART_STATE_BUSY_TX;
	/* Enable the TXE interrupt */
	config_txe_interrupt(handle->Instance,1);
}

static void config_rxe_interrupt(USART_TypeDef *uartx, uint32_t rxne)
{
	if(rxne)
		uartx->CR1 |=	USART_CR1_REG_RXNE_INTERRUPT_ENABLE;
	else
		uartx->CR1 &= ~USART_CR1_REG_RXNE_INTERRUPT_ENABLE;
		
}

static void config_parity_error_interrupt(USART_TypeDef *uartx, uint32_t pene)
{
	if(pene)
		uartx->CR1 |=USART_CR1_REG_PE_INTERRUPT_ENABLE;
	else
		uartx->CR1 &=~USART_CR1_REG_PE_INTERRUPT_ENABLE;
	
}

static void config_frame_noise_error_interrupt(USART_TypeDef *uartx, uint32_t eie_en)
{
	if(eie_en)
		uartx->CR3 |= USART_CR3_REG_ERR_INT_EN;
	else
		uartx->CR3 &= ~USART_CR3_REG_ERR_INT_EN;
	
}
/**
	* @brief  API to do UART data Reception  
	* @param  *handle : pointer to the handle structure of the UART peripheral  
  * @param  *buffer : holds the pointer to the RX buffer 
  * @param  len : len of the data to be RXed
	* @retval None
	*/
void stm32f407_uart_rx( uart_handle_t *handle,uint8_t *buffer, uint32_t len)
{
	uint32_t val;
	handle->pRbuffer = 	buffer;
	handle->Rxfersize =	len;
	handle->RxFercount = len;
	
	/* Enable the UART Parity Error Interrupt */
	config_parity_error_interrupt(handle->Instance,1);
	
	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	config_frame_noise_error_interrupt(handle->Instance,1);
	
	/* Enable Uart */
	enable_uart_peripheral(handle->Instance);
	
	handle->rx_state = UART_STATE_BUSY_RX;
	
	val = handle->Instance->DR;
	/* Enable the RxE interrupt */
	config_rxe_interrupt(handle->Instance,1);
	
	
}

static void config_tcie_interrupt(USART_TypeDef *uartx, uint32_t tcie_en)
{
	if(tcie_en)
		uartx->CR1 |= USART_CR1_REG_TCIE_INTERRUPT_ENABLE;
	else
		uartx->CR1 &= ~USART_CR1_REG_TCIE_INTERRUPT_ENABLE;
	
}
static void uart_handle_txe_interrupt(uart_handle_t *uartx)
{
	uint32_t tmp1 = 0;
	uint8_t		val;
	tmp1 = uartx->tx_state;
	if(tmp1 == UART_STATE_BUSY_TX)
	{
			
		val = (uint8_t)(*uartx->pTxbuffer++ & (uint8_t)0x00ff);
		uartx->Instance->DR = val;
		
		if(--uartx->TxXfercount == 0)
		{
			// Disable TxE Interrupt 
			config_txe_interrupt(uartx->Instance,0);
			
			// Enable TCIE Interrupt , TC flag will be set once Transmission is complete 
			config_tcie_interrupt(uartx->Instance,1);
			
		}
	}
	
}

static void uart_handle_rxe_interrupt(uart_handle_t *uartx)
{
	uint32_t tmp1 = uartx->rx_state;
	
	if(tmp1 == UART_STATE_BUSY_RX)
	{
		/* Is parity Enable */
		if(uartx->init.Parity == PARITY_NONE)
			*uartx->pRbuffer++ = (uartx->Instance->DR & 0x00FF);
		else
			*uartx->pRbuffer++ = (uartx->Instance->DR & 0x007F);
		
		/* Check the Count  */
		if(--uartx->RxFercount == 0)
		{
			/* Disable RxNE flag */ 
			config_rxe_interrupt(uartx->Instance,0);
			
		/*Disable the UART Parity Error Interrupt */
		config_parity_error_interrupt(uartx->Instance,0);
	
		/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
		config_frame_noise_error_interrupt(uartx->Instance,0);
			
			/* Make the Uart receiverstate ready */
			uartx->rx_state = UART_STATE_READY;
			
			/* Call the Application call back*/
			if(uartx->rx_cmp_cb)
			{
				uartx->rx_cmp_cb(&uartx->Rxfersize);
			}
	}
			
		
}
}

static void uart_handle_tc_interrupt(uart_handle_t *uartx)
{
	/* Disable the Transmission complete flag */
	config_tcie_interrupt(uartx->Instance,0);
	uartx->tx_state = UART_STATE_READY;
	
	/* Call the Application Call back */
	
}


static void uart_clear_error_flag(uart_handle_t *uartx)
{
	uint32_t val;
	val = uartx->Instance->SR;
	val = uartx->Instance->DR;
	
}

/**
  * @brief  UART error callbacks.
  * @param  huart: pointer to a uart_handle_t structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
 static void uart_error_cb(uart_handle_t *huart)
{
	while(1)
	{
		
		// Turn Red Led On 
	}
}
/**
  * @brief  This API handles UART interrupt request.
  * @param  huart: pointer to a uart_handle_t structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void stm32f407_uart_interrupt_handle(uart_handle_t *uart)
{
	
	uint32_t temp1 = 0, temp2 =0;
	
	/* check for Parity error */
	temp1 = uart->Instance->SR & USART_SR_REG_PARITY_ERROR;
	temp1 = uart->Instance->CR1 & USART_CR1_REG_PE_INTERRUPT_ENABLE;
	
	if( temp1 && temp2)
	{
		uart_clear_error_flag(uart);
		uart->ErrorCode |= UART_ERROR_PE;
	}
	
	/* Check for  Frame error */
	temp1 = uart->Instance->SR & USART_SR_REG_FRAMEING_ERROR;
	temp2 = uart->Instance->CR3 & USART_CR3_REG_ERR_INT_EN;
	if( temp1 && temp2)
	{
		uart_clear_error_flag(uart);
		uart->ErrorCode |= UART_ERROR_FE;
	}
	
	/******* Uart in Receiver mode ***********************/
	temp1 = 	uart->Instance->SR & USART_SR_REG_RXNE;
	temp2 = 	uart->Instance->CR1 & USART_CR1_REG_RXNE_INTERRUPT_ENABLE;
	
	if( temp1 && temp2)
	{
		uart_handle_rxe_interrupt(uart);
		
	}
	
	/******* Uart in Transmitter mode ***********************/
	temp1 = 	uart->Instance->SR & USART_SR_REG_TXE;
	temp2 = 	uart->Instance->CR1 & USART_CR1_REG_TXE_INTERRUPT_ENABLE;
	
	if( temp1 && temp2)
	{
		uart_handle_txe_interrupt(uart);
	}
	
	/* Uart in mode transmitter end mode ***********/
	temp1 = uart->Instance->SR & USART_SR_REG_TC_COMPLETE;
	temp2 = uart->Instance->CR1 & USART_CR1_REG_TCIE_INTERRUPT_ENABLE;
	
	if( temp1 && temp2)
	{
		uart_handle_tc_interrupt(uart);
	
	}
	
	if( uart->ErrorCode != UART_ERROR_NONE)
	{
		uart->rx_state =  UART_STATE_READY;
		uart->tx_state =  UART_STATE_READY;
		uart_error_cb(uart);
		
	}
}

