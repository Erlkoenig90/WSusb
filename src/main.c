#include "usb.h"

#define RCC_APB2ENR			(*(volatile uint32_t*) (0x40021018))
#define GPIOC_CRH			(*(volatile uint32_t*) (0x40011004))
#define GPIOC_BSRR			(*(volatile uint32_t*) (0x40011010))
#define RCC_APB2ENR_IOPCEN	(1 << 4)
#define	GPIO_BSRR_BR12		(1 << 28)
#define	GPIO_BSRR_BS12		(1 << 12)

int main(void) {
	/* Nur für STM32-Olimexino: Setze PC12 auf high um USB-Pullup noch deaktiviert zu lassen */
/*	RCC_APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC_CRH = 0x44474444;
	GPIOC_BSRR = GPIO_BSRR_BS12; */

	UsbSetup();

	/* Nur für STM32-Olimexino: Setze PC12 auf low um USB-Pullup zu aktivieren */
/*	GPIOC_BSRR = GPIO_BSRR_BR12; */

	while (1) {
		if (UsbRxAvail() == true) {
			if (UsbTxReady() == true) {
				UsbCharOut(UsbGetChar());
			}
		}
	}
}
