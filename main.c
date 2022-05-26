#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include <string.h>

#define WRITE_REGISTER(addr, value)    *((unsigned long int *)(addr)) = value
#define READ_REGISTER(addr, mask)      *((unsigned long int *)(addr)) & (mask)


#define CSR 0xE000E010
#define RVR 0xE000E014
#define CVR 0xE000E018

/* for UART */
#define UART_SR USART1_BASE+0x00
#define UART_DR USART1_BASE+0x04
#define UART_BRR USART1_BASE+0x08
#define UART_CR1 USART1_BASE+0x0C
#define UART_CR3 USART1_BASE+0x14

#define DMA2_S7CR	DMA2_BASE + 0xB8
#define DMA2_S7PAR 	DMA2_BASE + 0xC0
#define DMA2_S7M0AR DMA2_BASE + 0xC4
#define DMA2_HISR 	DMA2_BASE + 0x04
#define DMA2_LISR 	DMA2_BASE + 0x00
#define DMA2_S7NDTR DMA2_BASE + 0xBC
#define DMA2_HIFCR	DMA2_BASE + 0x0C
#define DMA2_LIFCR 	DMA2_BASE + 0x08


#define LED_2 GPIO_Pin_6
#define LED_3 GPIO_Pin_7

#define ON_LED 0
#define OFF_LED 1

#define BTN_IS_HIT 0
#define USR_BTN_GPIO GPIOE
#define USR_BTN_K0 GPIO_Pin_4
#define DELAY_TIME_MS 500
#define DELAY_TIME_SEND_MS 2000
#define DELAY_CHECK_UART 500


uint8_t state_7 = 0;
uint32_t oldtime_led6 = 0;
uint32_t oldtime_send = 0;
uint32_t oldtime_led7 = 0;
uint32_t oldtime_uart = 0;
uint32_t oldtime_recv_uart = 0;
uint32_t size = 0;


uint32_t ms_now = 0;
void SysTick_Handler(void)
{
	ms_now++;
}


void INIT_GPIO() {
	/* Set option for GPIO */
	GPIO_InitTypeDef GPIO_InitDef;
	GPIO_InitDef.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitDef.GPIO_OType = GPIO_OType_PP; /* push pull for out put */
	GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitDef);
}

void set_pin_as_af() {
	/* part 1 */
	USART_DeInit(USART1);
	GPIO_InitTypeDef     GPIO_InitStruct;


	/***/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	// Initialize pins as alternating function
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}



unsigned char data_arr[255];
uint32_t i = 0;
unsigned char gs_send_data = 0x55;


void set_memsize_persize() {
	unsigned int value = 0;
	/* set memory data size to 8-bit */
	value = READ_REGISTER(DMA2_S7CR, ~(3<<13));
	WRITE_REGISTER(DMA2_S7CR, value);

	/* set peripheral data size to 8-bit */
	value = READ_REGISTER(DMA2_S7CR, ~(3<<11));
	WRITE_REGISTER(DMA2_S7CR, value);
}
void clear_events_flags() {
	unsigned int value = 0;
	/* clear events */
	value = READ_REGISTER(DMA2_HIFCR, ~(1<<27));
	value |= 1<<27;
	WRITE_REGISTER(DMA2_HIFCR, value);

	value = READ_REGISTER(DMA2_HIFCR, ~(1<<26));
	value |= 1<<26;
	WRITE_REGISTER(DMA2_HIFCR, value);

	value = READ_REGISTER(DMA2_HIFCR, ~(1<<25));
	value |= 1<<25;
	WRITE_REGISTER(DMA2_HIFCR, value);

	/* clear flag */
	value = READ_REGISTER(DMA2_HIFCR, ~(1<<27));
	value &= 0<<27;
	WRITE_REGISTER(DMA2_HIFCR, value);

	value = READ_REGISTER(DMA2_HIFCR, ~(1<<26));
	value &= 0<<26;
	WRITE_REGISTER(DMA2_HIFCR, value);

	value = READ_REGISTER(DMA2_HIFCR, ~(1<<25));
	value &= 1<<25;
	WRITE_REGISTER(DMA2_HIFCR, value);
}
/* Setup DMA */
void setup_dma_uart() {
	unsigned int value = 0;

	/* Write the USART_DR address in the DMA control register as DESTINATION */

	/* disable stream7 */
	value = READ_REGISTER(DMA2_S7CR, ~1);
	WRITE_REGISTER(DMA2_S7CR, value);

	/* channel selected is 4 */
	value = READ_REGISTER(DMA2_S7CR, ~(1<<25));
	WRITE_REGISTER(DMA2_S7CR, value);
	value = READ_REGISTER(DMA2_S7CR, ~(1<<26));
	WRITE_REGISTER(DMA2_S7CR, value);
	value = READ_REGISTER(DMA2_S7CR, ~(1<<27));
	value |= 1<<27;
	WRITE_REGISTER(DMA2_S7CR, value);

	/* set transfer mode is memory to peripheral: 01 */
	value = READ_REGISTER(DMA2_S7CR, ~(1<<6));
	value |= 1<<6;
	WRITE_REGISTER(DMA2_S7CR, value);
	value = READ_REGISTER(DMA2_S7CR, ~(1<<7));
	WRITE_REGISTER(DMA2_S7CR, value);

	/* setup DR as destination */
	WRITE_REGISTER(DMA2_S7PAR, UART_DR);

	/* Configure Priority level to high(10) */
	value = READ_REGISTER(DMA2_S7CR, ~(1<<16));
	WRITE_REGISTER(DMA2_S7CR, value); /* clear bit 16 */
	value = READ_REGISTER(DMA2_S7CR, ~(1<<17));
	value |= 1<<17;
	WRITE_REGISTER(DMA2_S7CR, value); /* set bit 17 */

	/* set memory size and peripheral size */
	set_memsize_persize();


	/* Enable interrupt transmit complete */
	value = READ_REGISTER(DMA2_S7CR, ~(1<<4));
	value |= 1<<4;
	WRITE_REGISTER(DMA2_S7CR, value);

	/* clear events and flags */
	clear_events_flags();

	/* Enable DMA
	value = READ_REGISTER(DMA2_S7CR, ~1);
	value |= 1;
	WRITE_REGISTER(DMA2_S7CR, value);
	*/
}


void start_dma_stream() {
	unsigned int value = 0;

	value = READ_REGISTER(DMA2_S7CR, ~1);
	value |= 1;
	WRITE_REGISTER(DMA2_S7CR, value);
}
void stop_dma_stream() {
	unsigned int value = 0;

	value = READ_REGISTER(DMA2_S7CR, ~1);
	value |= 0;
	WRITE_REGISTER(DMA2_S7CR, value);
}

void trans_1_byte(unsigned char data) {
	unsigned int value = 0;
	do {
		/* */
		value = READ_REGISTER(UART_SR, 1<<6);
	} while (value ==0);
	/* disable stream7 */
	stop_dma_stream();

	/* FOR ARR- BYTE setup variable as source */
	WRITE_REGISTER(DMA2_S7M0AR, (unsigned int)&data);

	/* disable increment */
	value = READ_REGISTER(DMA2_S7CR, ~(1<<10));
	WRITE_REGISTER(DMA2_S7CR, value);

	/* transmit only 1 byte */
	WRITE_REGISTER(DMA2_S7NDTR, 1);

	clear_events_flags();

	/* Enable stream7 */
	start_dma_stream();
}

void trans_arr(unsigned char *adrr_data, unsigned int len_data) {
	/* strlen */
	unsigned int value = 0;
	/* disable stream7 */
	stop_dma_stream();

	/* FOR ARR- BYTE setup variable as source */
	WRITE_REGISTER(DMA2_S7M0AR, (unsigned int)adrr_data);

	/* memory increment */
	value = READ_REGISTER(DMA2_S7CR, ~(1<<10));
	value |= 1 <<10;
	WRITE_REGISTER(DMA2_S7CR, value);

	/* FOR_ARR, BYTE setup bytes size */
	WRITE_REGISTER(DMA2_S7NDTR, len_data);

	clear_events_flags();

	/* Enable stream7 */
	start_dma_stream();
}



void set_up_uart1() {
	unsigned int value = 0;
	/* setup baud rate <=> 115200 */
	WRITE_REGISTER(UART_BRR, 0x8B);

	/* enable Receive and Transmit */
	value = READ_REGISTER(UART_CR1, ~(3<<2));
	value = value | (3<<2);
	WRITE_REGISTER(UART_CR1, value);

	/* parity control disable */
	value = READ_REGISTER(UART_CR1, ~(1<<10));
	value = value | (0<<10);
	WRITE_REGISTER(UART_CR1, value);

	/* setup word length */
	value = READ_REGISTER(UART_CR1, ~(1<<12));
	value = value | (0<<12);
	WRITE_REGISTER(UART_CR1, value);

	/* OVERsampling by 16 */
	value = READ_REGISTER(UART_CR1, ~(1<<15));
	value = value | (0<<15);
	WRITE_REGISTER(UART_CR1, value);


	/* Enable DMA transmitter */
	value = READ_REGISTER(UART_CR3, ~(1<<7));
	value = value | (1<<7);
	WRITE_REGISTER(UART_CR3, value);

	/* Enable UART */
	value = READ_REGISTER(UART_CR1, ~(1<<13));
	value = value | (1<<13);
	WRITE_REGISTER(UART_CR1, value);
}


void DMA2_Stream7_IRQHandler(void) {
	/* code here */
	GPIO_WriteBit(GPIOA, GPIO_Pin_7, ON_LED);

	/* Disable DMA */

}




int main(void)
{
	memset(data_arr, 0, 255);
	data_arr[0] ='A';
	data_arr[1] = 'B';
	data_arr[2] = 'F';
	data_arr[3] = 'G';

	/* Default frequency */
	RCC_DeInit();
	SystemCoreClockUpdate();

	/* Enable Clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	/* setup Systick */
	SysTick_Config(SystemCoreClock/1000);

	/* setup LED */
	INIT_GPIO();
	GPIO_WriteBit(GPIOA, GPIO_Pin_6, OFF_LED);
	GPIO_WriteBit(GPIOA, GPIO_Pin_7, OFF_LED);


	/* setup UART */
	set_pin_as_af();
	/* STEP 1 */
	set_up_uart1();

	/* clear bit TC in SR register */
	unsigned int value1 = 0;
	value1 = READ_REGISTER(UART_SR, ~(1<<6));
	WRITE_REGISTER(UART_SR, value1);

	/* step 2*/
	setup_dma_uart();

	trans_1_byte('T');
	trans_1_byte('.');
	trans_1_byte(',');
	unsigned int lendata = strlen(data_arr);
	trans_arr(&data_arr[0], lendata);
	while (1) {
	}
}




