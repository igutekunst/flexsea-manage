//****************************************************************************
// MIT Media Lab - Biomechatronics
// Jean-Francois (Jeff) Duval
// jfduval@media.it.edu
// 05/2015
//****************************************************************************
// fm_stm32f4xx_it: Interrupt Handlers
//****************************************************************************
// Licensing: Please refer to 'software_license.txt'
//****************************************************************************

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "fm_stm32f4xx_it.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

volatile unsigned int spi_bytes_ready = 0;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern TIM_HandleTypeDef htim7;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

//...

//****************************************************************************
// Public Function(s)
//****************************************************************************

//System Timer, 1ms ISR
void SysTick_Handler(void)
{
	// Decrement to zero the counter used by the delay routine.
	if(timer_delayCount != 0u)
	{
		--timer_delayCount;
	}

	//For USB delays:
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void SPI4_IRQHandler(void)
{
	//When is it happening?
	//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1);
	//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 0);

	//Increment bytes counter
	spi_bytes_ready++;

	HAL_SPI_IRQHandler(&spi4_handle);
}

void EXTI4_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

//Should not be used, everything is done via DMA
void USART1_IRQHandler(void)
{
	HAL_USART_IRQHandler(&husart1);
}

//Should not be used, everything is done via DMA
void USART3_IRQHandler(void)
{
	HAL_USART_IRQHandler(&husart3);
}

//Should not be used, everything is done via DMA
void USART6_IRQHandler(void)
{
	HAL_USART_IRQHandler(&husart6);
}

//DMA2 Stream2 - USART1 RX
void DMA2_Stream2_IRQHandler(void)
{
	HAL_NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);

	HAL_DMA_IRQHandler(&hdma2_str2_ch4);
}

//DMA2 Stream7 - USART1 TX
void DMA2_Stream7_IRQHandler(void)
{
	HAL_NVIC_ClearPendingIRQ(DMA2_Stream7_IRQn);

	HAL_DMA_IRQHandler(husart1.hdmatx);
}

//DMA1 Stream1 - USART3 RX
void DMA1_Stream1_IRQHandler(void)
{
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn);

	HAL_DMA_IRQHandler(&hdma1_str1_ch4);
}

//DMA1 Stream3 - USART3 TX
void DMA1_Stream3_IRQHandler(void)
{
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn);

	HAL_DMA_IRQHandler(husart3.hdmatx);
}

//DMA2 Stream1 - USART6 RX
void DMA2_Stream1_IRQHandler(void)
{
	HAL_NVIC_ClearPendingIRQ(DMA2_Stream1_IRQn);

	HAL_DMA_IRQHandler(&hdma2_str1_ch5);
}

//DMA2 Stream6 - USART6 TX
void DMA2_Stream6_IRQHandler(void)
{
	HAL_NVIC_ClearPendingIRQ(DMA2_Stream6_IRQn);

	HAL_DMA_IRQHandler(husart6.hdmatx);
}

//DMA1 Stream 0: I2C1 RX
void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hi2c1.hdmarx);
}

//DMA1 Stream 6: I2C1 TX
void DMA1_Stream6_IRQHandler(void)
{
	HAL_DMA_IRQHandler(hi2c1.hdmatx);
}

//USB:
void OTG_FS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

void TIM7_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim7);

	//FlexSEA timebase:
	timebases();

}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}

void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
	/* These are volatile to try and prevent the compiler/linker optimising them
	away as the variables never actually get used.  If the debugger won't show the
	values of the variables, make them global my moving their declaration outside
	of this function. */
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r4;
	volatile uint32_t r12;
	volatile uint32_t lr; /* Link register. */
	volatile uint32_t pc; /* Program counter. */
	volatile uint32_t psr;/* Program status register. */

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    /* When the following line is hit, the variables contain the register values. */
    for( ;; ) {

    }
}

void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while(1)
	{
	}
}

void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while(1)
	{
	}
}

void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while(1)
	{
	}
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}
