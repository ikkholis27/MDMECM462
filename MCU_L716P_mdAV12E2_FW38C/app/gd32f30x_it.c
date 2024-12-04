#include "gd32f30x_it.h"
#include "usart.h"


/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
}

void USART0_IRQHandler(void)
{   
    uart0_handle();
/*	    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){
        uint8_t ch = usart_data_receive(USART0);
        usart_data_transmit(USART1, ch);
    }
	*/
}

void USART1_IRQHandler(void)
{
	uart1_handle();
	/*
    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE)){
        uint8_t ch = usart_data_receive(USART1);
				uart1_rxbuf[uart1_index]= ch;
				uart1_index++;
			if(uart1_index>3){
				if(uart1_rxbuf[uart1_index-1]=='Y' && uart1_rxbuf[uart1_index-2]=='D' && uart1_rxbuf[uart1_index-3]=='A'){
					rx1_flag=1;
					uart1_index=0;
				}
			}
        usart_data_transmit(USART0, ch);
    }
	*/
}

