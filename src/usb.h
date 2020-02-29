/* USB als COM Port betreiben */

#ifndef USB_H
#define USB_H

#include <stdint.h>
#include <stdbool.h>

/* Liste der von aussen zu benutzenden Funktionen */
bool UsbRxAvail(void);   /* true, wenn Char's vom Host abholbar sind */
char UsbGetChar(void);   /* liest ein Char vom Host */
bool UsbTxReady(void);   /* true, wenn mindestens 1 Char gesendet werden kann */
bool UsbTxEmpty(void);   /* true, wenn der Sendepuffer total leer ist */
int UsbTxFree(void);     /* Anzahl freier Plätze im Sendepuffer */
char UsbCharOut(char c); /* sendet ein Char zum Host */
void UsbStrOut(char* S); /* sendet einen String zum Host */
uint16_t UsbSetup(void);     /* Starten des USB-Cores */

#endif
