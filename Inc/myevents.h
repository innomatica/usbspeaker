#ifndef __MY_EVENTS_H
#define __MY_EVENTS_H

/** Pushbutton input event
 *
 * Event Data: (PBTN_ID)(EVT_TYPE)
 *
 *	* PBTN_ID: id of the pushbutton that generated the event
 *	* EVT_TYPE: type of the event such as single click, double click,
 */
#define EVT_PBTN_INPUT			0x10		///< event code for pushbutton input
#define PBTN_SCLK				0x01		///< single click
#define PBTN_LCLK				0x02		///< long click
#define PBTN_DCLK				0x03		///< double click
#define PBTN_TCLK				0x04		///< triple click
#define PBTN_DOWN				0x05		///< button state down
#define PBTN_ENDN				0x06		///< button state changed to up
/** UART packet received
 *
 * Event Data: (UART PACKET)
 */
#define EVT_UART_RXPKT			0x20		///< event code for UART RX packet

#endif // __MY_EVENTS_H
