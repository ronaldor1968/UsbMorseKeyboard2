/* Name: main.c
 * Project: Keyboard test
 * Author: Ronaldo Oliveira, baseado no exemplo de Christian Starkjohann and others
 * Creation Date: 2013-06-23
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>

#include "keycode.h"
#include "usbdrv.h"


#define BUTTON_PORT_B PORTB       	/* PORTx - register for BUTTON 1,2 e 3 output */
#define BUTTON_PIN_B  PINB        	/* PINx - register for BUTTON 1,2 e 3 */
#define BUTTON_BIT_B1 PB1          	/* bit for BUTTON 1 input/output */
#define BUTTON_BIT_B2 PB3          /* bit for BUTTON 3 input/output */
#define BUTTON_BIT_B3 PB4          /* bit for BUTTON 4 input/output */
#define SIZE 59
#define ALL_SIZE SIZE*4
#define WAIT_DOT 10
#define WAIT_END 45


/* ------------------------------------------------------------------------- */

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};

/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */
 
PROGMEM const uchar keyinfo[ALL_SIZE] = 
{0,1,KEY_E,0,
 0,2,KEY_I,0,
 0,3,KEY_S,0,
 0,4,KEY_H,0,
 0,5,KEY_5,0,
 0,6,KEY_ENTER,0,
 1,1,KEY_T,0,
 1,2,KEY_A,0,
 1,3,KEY_U,0,
 1,4,KEY_V,0,
 1,5,KEY_4,0,
 1,6,KEY_DELETE,KEY_CTRL + KEY_ALT,
 2,2,KEY_N,0,
 2,3,KEY_R,0,
 2,4,KEY_F,0,
 3,2,KEY_M,0,
 3,3,KEY_W,0,
 3,4,KEY_SPACE,0,
 3,5,KEY_3,0,
 3,6,KEY_7,KEY_SHIFT,
 4,3,KEY_D,0,
 4,4,KEY_L,0,
 5,3,KEY_K,0,
 5,4,KEY_C,0,
 6,3,KEY_G,0,
 6,4,KEY_P,0,
 7,3,KEY_O,0,
 7,4,KEY_J,0,
 7,5,KEY_2,0,
 8,4,KEY_B,0,
 8,5,KEY_6,KEY_SHIFT,
 9,4,KEY_X,0,
 9,7,KEY_4,KEY_SHIFT,
 10,4,KEY_C,0,
 11,4,KEY_Y,0,
 12,4,KEY_Z,0,
 12,6,KEY_MINUS,KEY_SHIFT,
 13,4,KEY_Q,0,
 13,6,KEY_SLASH,KEY_SHIFT,
 14,4,KEY_J,0,
 15,5,KEY_1,0,
 16,5,KEY_6,0,
 17,5,KEY_0,KEY_SHIFT,
 18,5,KEY_7,KEY_SHIFT,
 18,6,KEY_2,KEY_SHIFT,
 20,5,KEY_SEMICOLON,0,
 21,6,KEY_PERIOD,0,
 22,5,KEY_8,KEY_SHIFT,
 24,5,KEY_7,0,
 26,6,KEY_2,KEY_RIGHT_ALT,
 28,5,KEY_8,0,
 30,5,KEY_9,0,
 30,6,KEY_MINUS,0,
 33,6,KEY_SLASH,0,
 42,6,KEY_COMMA,KEY_SHIFT,
 43,6,KEY_1,KEY_SHIFT,
 45,6,KEY_9,KEY_SHIFT,
 51,6,KEY_COMMA,0,
 120,7,KEY_PERIOD,0
 };
 

/* ------------------------------------------------------------------------- */

static uchar    reportBuffer[2];    /* buffer for HID reports */
static uchar    idleRate;           /* in 4 ms units */
static uchar    keyToSend = 0;		/* keys to send */
unsigned int	timerDelay = 0;		/* counter for delay period */

static uchar command = 0;
static uchar lastcommand = 0;
static uchar nextkey = 0;
static uchar keymod  = 0; /* no modifiers */

static uchar bits_mask = 0;
static uchar mask_size = 0;

static uchar keybuffer[5];
static uchar keybuffersize = 0;

static uchar ntokey(uchar n) {
	if (n == 0) {
		return KEY_0;
	}
	return KEY_Z + n;	
}

static void checkButtonChange(void) {
	command = 0;
	if (bit_is_set(BUTTON_PIN_B, BUTTON_BIT_B1) == 0) {
		command = 1;
	} else if (bit_is_set(BUTTON_PIN_B, BUTTON_BIT_B2) == 0) {
		command = 2;
	}
	switch (command) {
		case 0: if (lastcommand == 1) {
					// verifica se é um ponto ou um traco
					mask_size ++;  // tamanho
					bits_mask = bits_mask * 2 ; // onde existe tracos
					if (timerDelay > WAIT_DOT) { 
						bits_mask = bits_mask + 1;
					}
					// reinicia contador
					timerDelay = 0;
				} else if (lastcommand == 2) {
					/*
					// verifica se é um ponto ou um traco
					if (timerDelay > WAIT_DOT) { 
						nextkey = KEY_SLASH;
					} else {
						nextkey = KEY_PERIOD;
					}
					keyToSend = 2;
					*/
					keybuffersize = 0;
					keybuffer[keybuffersize++] = KEY_ENTER;
					unsigned int t0 = timerDelay / 10;
					uchar t1 = timerDelay % 10;
					keybuffer[keybuffersize++] = ntokey(t1);
					t1 = t0 % 10;
					keybuffer[keybuffersize++] = ntokey(t1);
					t0 = t0 / 10;
					t1 = t0 % 10;
					keybuffer[keybuffersize++] = ntokey(t1);
					
					/*
					t0 = t0 / 10;
					t1 = t0 % 10;
					keybuffer[keybuffersize++] = ntokey(t1);
					*/ 
					// reinicia contador
					timerDelay = 0;
				} else if (lastcommand == 0) {
					// verifica o timeout
					if (mask_size == 0) {
						timerDelay = 0;
					}
					if (timerDelay > WAIT_END) { //1 segundo
						// se timeout, define a tecla a enviar
						nextkey = 0;
						keymod = 0;
						uchar i = 0;
						timerDelay = 0;
						for (; i < SIZE; i++) {
							// multiplica por 4
							uchar k = i * 4;
							if (pgm_read_byte(&keyinfo[k++]) == bits_mask) { // bits
								if (pgm_read_byte(&keyinfo[k++]) == mask_size) { // size
									nextkey = pgm_read_byte(&keyinfo[k++]);
									keymod = pgm_read_byte(&keyinfo[k]);
									keyToSend = 2;
								}
							}
						}
						bits_mask = 0;
						mask_size = 0;
					} else {
						if (keybuffersize > 0) {
							keybuffersize--;
							nextkey = keybuffer[keybuffersize];
							keymod = 0;
							keyToSend = 2;
						}
					}
				}
				break;
		case 1: if (lastcommand == 0) {
					// mudou para pressionado, inicia contador
					timerDelay = 0;
				}
				break;
		case 2: if (lastcommand == 0) {
					timerDelay = 0;
				} 
				break;
	}
	timerDelay++;
	lastcommand = command;  

}

static void buildReport(void)
{
	reportBuffer[0] = keymod;    
	reportBuffer[1] = nextkey;
	
}

static void verifyCommand(void)
{
	if(TIFR & (1 << TOV1)){
        TIFR = (1 << TOV1); /* clear overflow */
        
        checkButtonChange();
		
    }
}

/* ------------------------------------------------------------------------- */

static void timerInit(void)
{
    TCCR1 = 0x0b;           /* select clock: 16.5M/1k -> overflow rate = 16.5M/256k = 62.94 Hz */
}


/* ------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* ------------------------------------------------------------------------- */

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    usbMsgPtr = reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            buildReport();
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
	return 0;
}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void    usbEventResetReady(void)
{
    calibrateOscillator();
    eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
}

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */



int main(void)
{
	uchar   i;
	uchar   calibrationValue;

    calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }
    //odDebugInit();	
    usbDeviceDisconnect();
    for(i=0;i<20;i++){  /* 300 ms disconnect */
        _delay_ms(15);
    }
    usbDeviceConnect();

    wdt_enable(WDTO_1S);
    
    /* turn on internal pull-up resistor for the switches */
    BUTTON_PORT_B |= _BV(BUTTON_BIT_B1);
	BUTTON_PORT_B |= _BV(BUTTON_BIT_B2);
	BUTTON_PORT_B |= _BV(BUTTON_BIT_B3);
    
    timerInit();
	
    usbInit();
    sei();
    for(;;){    /* main event loop */
        wdt_reset();
        usbPoll();

		/* A USB keypress cycle is defined as a scancode being present in a report, and
		then absent from a later report. Example, to press and release the Caps Lock key, instead of
		holding it down, we need to send the report with the Caps Lock scancode and
		then an empty report. */

		if (keyToSend > 0) {
			if(usbInterruptIsReady()){ /* we can send another key */
				buildReport();
				usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
				keyToSend--;
				nextkey = 0;
				keymod = 0; /* no modifiers */
			}
		} else {
			verifyCommand();
		}
    }
    return 0;
}
