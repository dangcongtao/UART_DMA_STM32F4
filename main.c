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



unsigned char data_arr[255] = "ABCD";
uint32_t i = 0;
unsigned char gs_send_data = 0x55;

/* Setup DMA */
void setup_dma() {
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

	/* setup variable as source */
	WRITE_REGISTER(DMA2_S7M0AR, &gs_send_data);

	/* setup DR as destination */
	WRITE_REGISTER(DMA2_S7PAR, UART_DR);


	/* setup bytes size */
	unsigned int len_str = 4; /* number bytes want to load into source */
	WRITE_REGISTER(DMA2_S7NDTR, len_str);

	/* Configure Priority level to high(10) */
	value = READ_REGISTER(DMA2_S7CR, ~(1<<16));
	WRITE_REGISTER(DMA2_S7CR, value); /* clear bit 16 */
	value = READ_REGISTER(DMA2_S7CR, ~(1<<17));
	value |= 1<<17;
	WRITE_REGISTER(DMA2_S7CR, value); /* set bit 17 */

	/* set memory data size to 8-bit */
	value = READ_REGISTER(DMA2_S7CR, ~(3<<13));
	WRITE_REGISTER(DMA2_S7CR, value);

	/* set peripheral data size to 8-bit */
	value = READ_REGISTER(DMA2_S7CR, ~(3<<11));
	WRITE_REGISTER(DMA2_S7CR, value);

	/* Enable interrupt transmit complete */
	value = READ_REGISTER(DMA2_S7CR, ~(1<<4));
	value |= 1<<4;
	WRITE_REGISTER(DMA2_S7CR, value);

	/* clear events and flags */
	/* clear flag */
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

	/* Enable DMA */
	value = READ_REGISTER(DMA2_S7CR, ~1);
	value |= 1;
	WRITE_REGISTER(DMA2_S7CR, value);
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
	value |= 1;
	WRITE_REGISTER(DMA2_S7CR, value);
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









char send_byte_uart(unsigned int data) {
	unsigned int value = 0;
	value = READ_REGISTER(UART_SR, (1<<7));
	/* TX is empty */
	if (value != 0) {
		WRITE_REGISTER(UART_DR, data);
	}

	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) != SET);

	return value;
}

/* Handle Receive */
void receive_data() {
	unsigned int value = 0;
	uint16_t temp = 0;
	do {
		value = READ_REGISTER(UART_SR, (1<<5));
		value = value >> 5;
		if (value == 1) {
			temp = READ_REGISTER(UART_DR, 0x1FF);
			send_byte_uart(temp);
			GPIO_WriteBit(GPIOA, GPIO_Pin_7, ON_LED);
			oldtime_led7 = ms_now;
		} else if (ms_now - oldtime_led7 > DELAY_TIME_MS) {
			GPIO_WriteBit(GPIOA, GPIO_Pin_7, OFF_LED);
		}

	} while (value == 1);
}

void send_data_str_uart1(unsigned char data[], unsigned int size) {
	unsigned int value = 0;
	for(int i = 0; i < size; i ++) {
		while (value != 1) {
			value = READ_REGISTER(UART_SR, (1<<7));
			value = value >> 7;
		}
		value = 0;
		WRITE_REGISTER(UART_DR, data[i]);
	}
}

void send_data_str_uart2(char *data) {
	unsigned int len = strlen(data);

	for(int i = 0; i < len; i ++) {
		send_byte_uart(data[i]);
	}
}

void send_byte_dma(unsigned char data) {
	stop_dma_stream();
	gs_send_data = data;

	setup_dma();
}
void send_data_using_dma() {

}
int main(void)
{
	memset(data_arr, 0, 255);


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
	setup_dma();
//	DMA_MemoryTargetConfig()
//	DMA_MemoryTargetConfig()
//	DMA
	while (1) {
		send_byte_dma('Y');
	}
}



