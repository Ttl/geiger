#include "geiger.h"
#define ACMUX _SFR_MEM8(0x7D)

#define bit_set(p,m) ((p) |= (1<<m))
#define bit_clear(p,m) ((p) &= ~(1<<m))

/* Event counter */
static volatile uint8_t count = 0;
/* Time of last event */
static volatile uint8_t tx_event_time, event_time;
/* Send count over USB */
static bool send_count = 0;
/* Events counter for transfer */
static volatile uint8_t tx_count = 0;
/* Extension for TMR0 */
static volatile uint8_t tmr0_count = 0;
static volatile bool usb_connected = 0;
/* Use analog comparator feedback loop */
static volatile bool use_comparator = 1;
/* Transfer event counter */
static volatile bool tx_flag = 0;
static bool buzzer_enabled = 1;
/* Tick buzzer */
static volatile bool tick_pending = 0;

/* 1 when slept for 1ms */
static volatile bool sleep_over = 0;

static FILE USBSerialStream;

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = 0,
				.DataINEndpoint           =
					{
						.Address          = CDC_TX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint =
					{
						.Address          = CDC_RX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.NotificationEndpoint =
					{
						.Address          = CDC_NOTIFICATION_EPADDR,
						.Size             = CDC_NOTIFICATION_EPSIZE,
						.Banks            = 1,
					},
			},
	};

void usb_rx(void) {
    /*  Handle messages from host */
    char ReceivedByte;
    if((ReceivedByte = fgetc(&USBSerialStream)) == EOF) {
        return;
    }
    /*  Increase HV supply voltage */
    if (ReceivedByte == 'u') {
        if (OCR1A < ICR1-1)
            OCR1A += 1;
    }
    /*  Increase HV supply voltage with bigger step */
    if (ReceivedByte == 'U') {
        if (OCR1A < ICR1) {
            OCR1A += 10;
        } else {
            OCR1A = ICR1-1;
        }
    }
    /*  Decrease HV supply voltage */
    if (ReceivedByte == 'd') {
        if (OCR1A > 1)
            OCR1A -= 1;
    }
    /*  Decrease HV supply voltage with bigger step */
    if (ReceivedByte == 'D') {
        if (OCR1A > 1) {
            OCR1A -= 10;
        } else {
            OCR1A = 1;
        }
    }
    /*  Comparator status. Over 400V? */
    if (ReceivedByte == 'a') {
        fprintf(&USBSerialStream, "a=%c\r\n", bit_is_clear(ACSR, ACO)?'y':'n');
    }
    /*  NMOS gate PWM duty cycle x/1024 */
    if (ReceivedByte == 'm') {
        fprintf(&USBSerialStream, "d=%d\r\n", OCR1A);
    }
    /*  Ring bell */
    if (ReceivedByte == 'z') {
        tick_pending = 1;
    }
    /*  Check buzzer on/off */
    if (ReceivedByte == 'b') {
        if(buzzer_enabled) {
            fprintf(&USBSerialStream, "Buzzer on\r\n");
        } else {
            fprintf(&USBSerialStream, "Buzzer off\r\n");
        }
    }
    /*  Buzzer on/off */
    if (ReceivedByte == 'B') {
        buzzer_enabled = !buzzer_enabled;
        if(buzzer_enabled) {
            fprintf(&USBSerialStream, "Buzzer on\r\n");
        } else {
            fprintf(&USBSerialStream, "Buzzer off\r\n");
        }
    }
    /*  Check comparator status */
    if (ReceivedByte == 'c') {
        if (use_comparator) {
            fprintf(&USBSerialStream, "Comparator on\r\n");
        } else {
            fprintf(&USBSerialStream, "Comparator off\r\n");
        }
    }
    /* Enable sending count over USB */
    if (ReceivedByte == 's') {
        send_count = 1;
    }
    /* Disable sending */
    if (ReceivedByte == 'S') {
        send_count = 0;
    }
    /*  Change comparator status */
    if (ReceivedByte == 'C') {
        use_comparator = !use_comparator;
        if (use_comparator) {
            fprintf(&USBSerialStream, "Comparator on\r\n");
        } else {
            fprintf(&USBSerialStream, "Comparator off\r\n");
        }
    }
    return;
}

void setupHardware(void) {
    /* PC6 = NMOSG, PC7 = Buzzer */
    DDRC   = 0b11000000;
    DDRD   = 0b11111111;

    /* Setup HV boost converter PWM */
    /* No prescaler, fast PWM, TOP = ICR1,
     * set on compare match, clear on top */
    TCCR1A = 0b10000010;
    TCCR1B = 0b00011001;
    ICR1 = 1024;

    /*  Initial duty cycle x/1024 */
    OCR1A  = 10;

    /*  Set detect interrupt on PC4(PCINT10) */
    PCMSK1 = 0b00000100;
    PCICR  = 0b00000010;

    /*  Set analog comparator, compare to 1.1V bandgap, set on pin toggle */
    ACSR = 0b01000000;
    /*  Select AIN2 */
    ACMUX = 0x01;

    /*  Disable AIN2 digital buffer */
    DIDR1 = 0b00000010;

    /* Set timer0 for counting events */
    /* Period is 16.384ms */
    TCNT0 = 0x00;
    TCCR0A = 0x00;
    /*  Set prescaler to divide by 1024 for TMR0 */
    TCCR0B = 0b11000101;
    TIMSK0 = 0x01;

    /* Disable wtachdog */
    MCUSR &= ~(1 << WDRF);
	wdt_disable();
    /* Disable prescaler */
    clock_prescale_set(clock_div_1);

    return;
}

void sleep1ms(void) {
    /* Disable analog comparator to save power */
    ACSR = 0b11000000;

    sleep_over = 0;

    set_sleep_mode(SLEEP_MODE_IDLE);
    /* Set compare register to interrupt in 1ms */
    OCR0A = TCNT0 + 15;
    /*  Enable OCR0A interrupt */
    bit_set(TIMSK0, 1);
    /*  Loop to continue sleeping if any other interrupt wakes CPU */
    do {
        sleep_enable();
        sleep_cpu();
    } while(sleep_over == 0);
    /*  Done, disable sleep */
    sleep_disable();

    /*  Disable OCR0A interrupt */
    bit_clear(TIMSK0, 1);

    /* Enable analog comparator */
    ACSR = 0b01000000;
    return;
}

int main(void)
{
    uint8_t loops_ocr1a = 0;
    /* Cache for OCR1A when PWM is disabled */
    uint16_t OCR1A_save;
    bool usb_initialized = 0;
    /* Is HV supply voltage reached? If not take bigger steps */
    bool hv_reached = 0;

    setupHardware();

    sei(); /*  Enable interrupts */
    while(1)
    {
        if(!usb_initialized) {
            /*  Check if USB power is connected */
            if (bit_is_set(PINB,4)) {
                /* USB initialization */
                USB_Init();
                CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);
                usb_initialized = 1;
            }
        }

        if (usb_connected) {
            /*  Check mail */
            usb_rx();
            CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
            USB_USBTask();
            /*  Send counter over usb and reset */
            if (send_count && tx_flag) {
                tx_flag = 0;
                /*  Print number of events and time of last event */
                fprintf(&USBSerialStream, "c%3d,%3d\r\n", tx_count, tx_event_time);
                /*  TODO: LCD */
            }
        }

        /*  HV supply feedback */
        if( use_comparator ) {
            /*  Check if output voltage is over threshold */
            if (bit_is_set(ACSR, ACO)) {
                /*  Not over 400V */
                if ((loops_ocr1a % 10) == 0) {
                    if (OCR1A < 100) {
                        if (!hv_reached) {
                            OCR1A += 15;
                        } else {
                            OCR1A += 1;
                        }
                    }
                    if (OCR1A < 500) {
                        if (!hv_reached) {
                            OCR1A += 10;
                        } else {
                            OCR1A += 1;
                        }
                    }
                }
            } else if((loops_ocr1a == 100)) {
                /* Required voltage reached, decrease step size */
                hv_reached = 1;
                /*  Decrease OCR1A when after some time to avoid creeping upwards */
                if (OCR1A > 2) {
                    OCR1A--;
                }

                /* Reset loop counter */
                loops_ocr1a = 0;
            }
            loops_ocr1a++;
        }

        if (buzzer_enabled && tick_pending) {
            bit_set(PORTC, 7);
            tick_pending = 0;
        }
        sleep1ms();
        bit_clear(PORTC, 7);
    }
}

/*  Detect pin interrupt */
ISR(PCINT1_vect) {
    /*  Check if PC4 is 0 */
    if (bit_is_clear(PINC,4)) {
        count++;
        tick_pending = 1;
        event_time = TCNT0;
    }
}

/*  Event count timer, period 1024*256/(16*10^6) = 16.384ms */
ISR(TIMER0_OVF_vect) {
    tx_count = count;
    count = 0;
    tx_flag = 1;
    tx_event_time = event_time;
    event_time = 0;
}

/* Interrupt sleep */
ISR(TIMER0_COMPA_vect) {
    sleep_over = 1;
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
    usb_connected = 1;
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
    usb_connected = 0;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

