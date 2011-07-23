// ==============================================================================
// uDMX.c
// firmware for usb to dmx interface
//
// License:
// The project is built with AVR USB driver by Objective Development, which is
// published under an own licence based on the GNU General Public License (GPL).
// usb2dmx is also distributed under this enhanced licence. See Documentation.
//
// target-cpu: ATMega8 @ 12MHz
// created 2006-02-09 mexx
//
// version 1.2b
// 2007-01-05 me@anyma.ch added uBOOT boot-loader support
// 2008-12-12 malte@maltepoeggel.de fixed timing problems for some dmx hardware
// 2011-07-22 malte@maltepoeggel.de included detection for external power supply,
//                                  updated (and patched) avr v-usb version
// ==============================================================================

#define F_CPU        12000000               		// 12MHz processor 
 
// ==============================================================================
// includes
// ------------------------------------------------------------------------------
// AVR Libc (see http://www.nongnu.org/avr-libc/)
#include <avr/io.h>			// include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support
#include <avr/wdt.h>		// include watchdog timer support
#include <avr/sleep.h>		// include cpu sleep support
#include <util/delay.h>

// USB driver by Objective Development (see http://www.obdev.at/products/avrusb/index.html)
#include "usbdrv/usbdrv.h"

// local includes
#include "../uDMX_cmds.h"		// USB command and error constants


// ==============================================================================
// Constants
// ------------------------------------------------------------------------------
#define NUM_CHANNELS 512		// number of channels in DMX-512

// values for dmx_state
#define dmx_Off 0
#define dmx_NewPacket 1
#define dmx_NewWait 2	
#define dmx_InGap 3
#define dmx_InPacket 4
#define dmx_ChWait 5
#define dmx_ChGap 6
#define dmx_EndOfPacket 7
#define dmx_InBreak 8
#define dmx_InMAB 9

// values for usb_state
#define usb_Idle 0
#define usb_ChannelRange 1


// PORTC States for leds
#define LED_YELLOW 0x10
#define LED_GREEN 0x1
#define LED_BOTH 0x0
#define LED_NONE 0x11
#define LED_KEEP_ALIVE 200;

// Software jumper to initiate firmware updates via built in bootloader
#define UBOOT_SOFTJUMPER_ADDRESS	0x05
#define UBOOT_SOFTJUMPER		0xd9

typedef unsigned char  u08;
typedef   signed char  s08;
typedef unsigned short u16;
typedef   signed short s16;


// convenience macros (from Pascal Stangs avrlib)
#ifndef BV
	#define BV(bit)			(1<<(bit))
#endif
#ifndef cbi
	#define cbi(reg,bit)	reg &= ~(BV(bit))
#endif
#ifndef sbi
	#define sbi(reg,bit)	reg |= (BV(bit))
#endif
// ==============================================================================
// Globals
// ------------------------------------------------------------------------------
// dmx-related globals
static u08 dmx_data[NUM_CHANNELS];
static u16 out_idx;			// index of next frame to send
static u16 packet_len = 0;	// we only send frames up to the highest channel set
static u08 dmx_state;

// usb-related globals
static u08 usb_state;
static u16 cur_channel, end_channel;
static u08 reply[8];

//led keep alive counter
static u16 lka_count;

// ==============================================================================
// - sleepIfIdle
// ------------------------------------------------------------------------------
void sleepIfIdle()
{
	if(TIFR & BV(TOV1)) {
		cli();
		if(!(GIFR & BV(INTF1))) {
			// no activity on INT1 pin for >3ms => suspend:
			
			// turn off leds
			PORTC = LED_NONE;
			
			// - reconfigure INT1 to level-triggered and enable for wake-up
			cbi(MCUCR, ISC10);
			sbi(GICR, INT1);
			// - go to sleep
			wdt_disable();
			sleep_enable();
			sei();
			sleep_cpu();
			
			// wake up
			sleep_disable();
			// - reconfigure INT1 to any edge for SE0-detection
			cbi(GICR, INT1);
			sbi(MCUCR, ISC10);
			// - re-enable watchdog
			wdt_reset();
			wdt_enable(WDTO_1S);
		}
		sei();
		// clear INT1 flag
		sbi(GIFR, INTF1);
		// reload timer and clear overflow
		TCCR1B = 1;
		TCNT1 = 25000;		// max ca. 3ms between SE0
		sbi(TIFR, TOV1);
	}
}


// ------------------------------------------------------------------------------
// - INT1_vec (dummy for wake-up)
// ------------------------------------------------------------------------------
ISR(INT1_vect) {}



// ------------------------------------------------------------------------------
// - Write to EEPROM
// ------------------------------------------------------------------------------


static void eepromWrite(unsigned char addr, unsigned char val)
{
    while(EECR & (1 << EEWE));
    EEARL = addr;
    EEDR = val;
    cli();
    EECR |= 1 << EEMWE;
    EECR |= 1 << EEWE;  /* must follow within a couple of cycles -- therefore cli() */
    sei();
}

// ------------------------------------------------------------------------------
// - USB Reset
// ------------------------------------------------------------------------------


 void usbReset(void) {
 	u08  i, j;
	DDRB = ~0;			// output SE0 for USB reset
    j = 0;
    while(--j) {        // USB Reset by device only required on Watchdog Reset
        i = 0;
        while(--i);     // delay >10ms for USB reset
    }
    DDRB = ~USBMASK;    // all outputs except USB data
}

// ==============================================================================
// - init
// ------------------------------------------------------------------------------
void init(void)
{
	dmx_state = dmx_Off;
	lka_count = 0xffff;
	
	// configure IO-Ports; most are unused, we set them to outputs to have defined voltages
	DDRB = 0xFF;		// all unused except PB0 and 1 for USB; these are configured later
	DDRC = 0xFF;		// unused except PC0 and PC 4 for LEDS (outputs anyway...)
	DDRD = 0xD3;		// unused except PD2 + 3 (INT0 + 1), PD1 (TX)
						// and PD5 for Hardware Bootloader-Reset (pull to ground to force Bootloader)
						// INT0 is used by USB driver, INT1 for bus activity detection (sleep)
	
	//welcome light	
	PORTC = LED_BOTH;
		
	// init uart
	UBRRL = F_CPU/4000000 - 1; UBRRH =  0; // baud rate 250kbps
	UCSRA =  0; // clear error flags
	UCSRC =  BV(URSEL) | BV(USBS) | (3 << UCSZ0); // 8 data bits, 2 stop bits, no parity (8N2)
	UCSRB =  0; // don't turn on UART jet...
	
	// init timer0 for DMX timing
	TCCR0 = 2; // prescaler 8 => 1 clock is 2/3 us
	
	// set powerstate
	setPowerState(0); // bus powered
		
	// init usb
	wdt_enable(WDTO_1S);	// enable watchdog timer
        PORTB = 0;				// no pullups on USB pins
	usbReset();
        usbInit();

        // power state detection (dc in) - input
	DDRB &= ~(1<<PB3);
	PORTB &= ~(1<<PB3);
	
	// init Timer 1  and Interrupt 1 for usb activity detection:
	// - set INT1 to any edge (polled by sleepIfIdle())
	cbi(MCUCR, ISC11);
	sbi(MCUCR, ISC10);
	
	// set sleep mode to full power-down for minimal consumption
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	// - set Timer 1 prescaler to 64 and restart timer
	TCCR1B = 3;
	TCNT1 = 0;
	sbi(TIFR, TOV1);
	
	PORTC = LED_GREEN;

	sei();
}


// ==============================================================================
// - usbFunctionSetup
// ------------------------------------------------------------------------------
uchar usbFunctionSetup(uchar data[8])
{
	usbMsgPtr = reply;
	reply[0] = 0;
    if(data[1] == cmd_SetSingleChannel) {
		lka_count = 0;
		// get channel index from data.wIndex and check if in legal range [0..511]
		u16 channel = data[4] | (data[5] << 8);
		if(channel > 511) { reply[0] = err_BadChannel; return 1; }
		// get channel value from data.wValue and check if in legal range [0..255]
		if(data[3]) { reply[0] = err_BadValue; return 1; }
		dmx_data[channel] = data[2];
		// update dmx state
		if(channel >= packet_len) packet_len = channel+1;
		if(dmx_state == dmx_Off) dmx_state = dmx_NewPacket;
	}
	else if(data[1] == cmd_SetChannelRange) {
		lka_count = 0;
		// get start and end channel index
		cur_channel = data[4] | (data[5] << 8);
		end_channel = cur_channel + (data[2] | (data[3] << 8));
		// check for legal channel range
		if((end_channel - cur_channel) > (data[6] | (data[7] << 8)))
			{ reply[0] = err_BadValue; cur_channel = end_channel = 0; return 1; }
		if((cur_channel > 511) || (end_channel > 512)) 
			{ reply[0] = err_BadChannel; cur_channel = end_channel = 0; return 1; }
		// update usb state and wait for channel data
		usb_state = usb_ChannelRange;
		return 0xFF;
		
	} else if(data[1] == cmd_StartBootloader) {
	
		eepromWrite(UBOOT_SOFTJUMPER_ADDRESS,UBOOT_SOFTJUMPER); 	// set software jumper
		usbReset();													// disconnect uDMX to reconnect as uBOOT
		
	}
	
	return 0;
}


// ------------------------------------------------------------------------------
// - usbFunctionWrite
// ------------------------------------------------------------------------------
uchar usbFunctionWrite(uchar* data, uchar len)
{
	if(usb_state != usb_ChannelRange) { return 0xFF; }
	lka_count = 0;
	// update channel values from received data
	uchar* data_end = data + len;
	for(; (data < data_end) && (cur_channel < end_channel); ++data, ++cur_channel)
		dmx_data[cur_channel] = *data;
	// update state
	if(cur_channel >= end_channel) usb_state = usb_Idle;
	if(cur_channel > packet_len) packet_len = cur_channel;
	if(dmx_state == dmx_Off) dmx_state = dmx_NewPacket;
	return 1;
}


// ==============================================================================
// - main
// ------------------------------------------------------------------------------
int main(void)
{               
	init();

        // to detect pc connection while dc powered
        uint8_t pwr_state_last = 1;

	while(1) {

                if(pwr_state_last == 1)
                 {
                  // usb powered
                  if(!(PINB & (1<<PINB3))) 
                   {
                    pwr_state_last = 0;
                    setPowerState(0); // bus powered
                   }
                 } else {
                  // dc only
                  if(PINB & (1<<PINB3))
                   {
                    // reset usb due pc connection
                    setPowerState(1); // self powered
                    usbReset();
                    lka_count = 0;
                    // power state detection (dc in) - input
	            DDRB &= ~(1<<PB3);
	            PORTB &= ~(1<<PB3);
                    pwr_state_last = 1;
                   }                
                 }

		PORTC = LED_GREEN;
		
		if (lka_count < 0xfff ) { 
			lka_count++;
			PORTC = LED_BOTH;
		}
		
		// usb-related stuff
                wdt_reset();
		usbPoll();
		if(!packet_len) continue;
		
		// do dmx transmission
		switch(dmx_state) {
			case dmx_NewPacket: {
				// start a new dmx packet:
				sbi(UCSRB, TXEN);	// enable UART transmitter
				out_idx = 0;		// reset output channel index
				sbi(UCSRA, TXC);	// reset Transmit Complete flag
				UDR =  0;		// send start byte
				dmx_state = dmx_NewWait;
				break;
			}

			case dmx_NewWait: {
				if(UCSRA & BV(UDRE)) {
					sbi(SFIOR, PSR10);	// reset timer prescaler
					TCNT0 = 200;		// 30 clks = 20us
					sbi(TIFR, TOV0);	// clear timer overflow flag
					dmx_state = dmx_InGap;
				}
				break;
			}

			case dmx_InGap: {
				if(TIFR & BV(TOV0)) {
					// end of MARK AFTER BREAK; start new dmx packet
					dmx_state = dmx_InPacket;
				}
				break;
			}

			case dmx_InPacket: {
				if(UCSRA & BV(UDRE)) {
					// send next byte of dmx packet
					if(out_idx < packet_len) { 
						UDR = dmx_data[out_idx++];
						dmx_state = dmx_ChWait; 
					}
					else dmx_state = dmx_EndOfPacket;
				}
				break;
			}
			
			case dmx_ChWait: {
				if(UCSRA & BV(UDRE)) {
					sbi(SFIOR, PSR10);	// reset timer prescaler
					TCNT0 = 200;		// 30 clks = 20us
					sbi(TIFR, TOV0);	// clear timer overflow flag
					dmx_state = dmx_ChGap;
				}
				break;
			}

			case dmx_ChGap: {
				if(TIFR & BV(TOV0)) {
					// end of MARK AFTER BREAK; start new dmx packet
					dmx_state = dmx_InPacket;
				}
				break;
			}

			case dmx_EndOfPacket: {
				if(UCSRA & BV(TXC)) {
					// send a BREAK:
					cbi(UCSRB, TXEN);	// disable UART transmitter
					cbi(PORTD, 1);		// pull TX pin low
					
					sbi(SFIOR, PSR10);	// reset timer prescaler
					TCNT0 = 123;		// 132 clks = 88us
					sbi(TIFR, TOV0);	// clear timer overflow flag
					dmx_state = dmx_InBreak;
				}
				break;
			}
			case dmx_InBreak: {
				if(TIFR & BV(TOV0)) {
					sleepIfIdle();	// if there's been no activity on USB for > 3ms, put CPU to sleep
					
					// end of BREAK: send MARK AFTER BREAK
					sbi(PORTD, 1);		// pull TX pin high
					sbi(SFIOR, PSR10);	// reset timer prescaler
					TCNT0 = 243;		// 12 clks = 8us
					sbi(TIFR, TOV0);	// clear timer overflow flag
					dmx_state = dmx_InMAB;
				}
				break;
			}
			
			case dmx_InMAB: {
				if(TIFR & BV(TOV0)) {
					// end of MARK AFTER BREAK; start new dmx packet
					dmx_state = dmx_NewPacket;
				}
				break;
			}
		}
	}
	return 0;
}

