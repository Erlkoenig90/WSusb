/* USB als COM Port betreiben */

#ifndef USB_H
#define USB_H

#include <stdint.h>
#include <stdbool.h>

/* Liste der von aussen zu benutzenden Funktionen */
bool UsbRxAvail(void);   /* true, wenn Char's vom Host abholbar sind */
char UsbGetChar(void);   /* liest ein Char vom Host */
bool UsbConfigured (void); /* true, wenn der Host per SET_CONFIGURATION eine Konfiguration aktiviert hat. Vorher ist keine VCP-Kommunikation möglich. */
bool UsbTxReady(void);   /* true, wenn mindestens 1 Char gesendet werden kann */
bool UsbTxEmpty(void);   /* true, wenn der Sendepuffer total leer ist */
int UsbTxFree(void);     /* Anzahl freier Plätze im Sendepuffer */
char UsbCharOut(char c); /* sendet ein Char zum Host */
void UsbStrOut(char* S); /* sendet einen String zum Host */
void UsbTxFlush (void);	 /* Sende alles im Puffer befindliche ab (asynchron, nicht-blockierend) */
uint16_t UsbSetup(void);     /* Starten des USB-Cores */

#endif
