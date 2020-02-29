#include <stdio.h>
#include "stm32f1xx.h"
#include "usb.h"

// The current clock frequency
uint32_t SystemCoreClock=8000000;

// Counts milliseconds
volatile uint32_t systick_count=0;

// Interrupt handler
void SysTick_Handler(void)
{
    systick_count++;
}

// Delay some milliseconds
void delay_ms(int ms)
{
    uint32_t start=systick_count;
    while (systick_count-start<ms);
}

// Change system clock to 48Mhz using 8Mhz crystal
// Called by Assembler startup code
void SystemInit(void)
{
    // Because the debugger switches PLL on, we may
    // need to switch back to the HSI oscillator without PLL

    // Switch to HSI oscillator
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);

    // Wait until the switch is done
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI) {}

    // Disable the PLL, then we can configure it
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);

    // Flash latency 1 wait state
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_0);

    // Enable HSE oscillator
    SET_BIT(RCC->CR, RCC_CR_HSEON);

    // Wait until HSE oscillator is ready
    while(!READ_BIT(RCC->CR, RCC_CR_HSERDY)) {}

    // 48Mhz using the 8Mhz HSE oscillator with 6x PLL, lowspeed I/O runs at 24Mhz
    WRITE_REG(RCC->CFGR, RCC_CFGR_PLLSRC + RCC_CFGR_PLLMULL6 + RCC_CFGR_PPRE1_DIV2);

    // Enable PLL
    SET_BIT(RCC->CR, RCC_CR_PLLON);

    // Wait until PLL is ready
    while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY)) {}

    // Select PLL as clock source
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

    // Set USB prescaler to 1, skip in case of 72 MHz
    MODIFY_REG(RCC->CFGR, RCC_CFGR_USBPRE, RCC_CFGR_USBPRE);

    // Update variable
    SystemCoreClock=48000000;
}

void init_io()
{
    // Enable USB
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN);

    // Enable Port A and C
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN + RCC_APB2ENR_IOPCEN);

    // PA5 = Output (LED on Nucleo-64 board)
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF5 + GPIO_CRL_MODE5, GPIO_CRL_MODE5_0);

    // PC13 = Output (LED on Blue-Pill board)
    MODIFY_REG(GPIOC->CRH, GPIO_CRH_CNF13 + GPIO_CRH_MODE13, GPIO_CRH_MODE13_0);
}

// Alternative: Redirect standard output to the USB port, e.g. from puts() or printf()
int _write(int file, char *ptr, int len)
{
    for (int i=0; i<len; i++)
    {
        UsbCharOut(*ptr++);
    }
    return len;
}

int main(void) {
  // Initialize system timer
  SysTick_Config(SystemCoreClock / 1000);

  init_io();
  UsbSetup();

  while (1) {
    if (UsbRxAvail() == true) {
      if (UsbTxReady() == true)
        UsbCharOut(UsbGetChar());
    }
  }
}
