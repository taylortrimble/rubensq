#include <p18f4550.h>

#pragma config FOSC=INTOSC_HS, PLLDIV=2, CPUDIV=OSC1_PLL2, USBDIV=2, IESO=ON
#pragma config WDT=OFF, PWRT=ON, BOR=OFF
#pragma config LVP=OFF, MCLRE=ON, PBADEN=OFF, STVREN=ON
#pragma config FCMEN = ON


//**********************************************************************************************************


#define STATE_BUTTON 0
#define STATE_WAIT 1

#undef ANALOG

//PR2 value for the number of humps
// For old values, consult the project wiki
rom static unsigned char PR2Value[] =
{
    1,      //0 - off
    255,    //1
    154,    //2
    124,    //3
    99,     //4
    83,     //5
    76,     //6
    67,     //7
    54,     //8
    45,     //9
    255,    //flare
    1,      //0 - off
    1       //dummy
};

//column numbers
rom static unsigned char columnLookUp[] =
{
    0x00,   //0
    0xEF,   //1
    0xDF,   //2
    0xBF    //3
};

volatile unsigned char gColumn = 0;
volatile unsigned char gWaitColumn = 0;


//**********************************************************************************************************


void high_isr(void);
void low_isr(void);

void setPWM(unsigned char x);
void changeColumn(void);
unsigned char PWMFromKeyPad(unsigned char column, unsigned char row);
unsigned char numberOFButtonsPressed(unsigned char port);
unsigned char rowFromPort(unsigned char port);

#pragma code high_isr_entry=8
void high_isr_entry(void)
{
    _asm goto high_isr _endasm
}

#pragma code low_isr_entry=0x18
void low_isr_entry(void)
{
    _asm goto low_isr _endasm
}

#pragma code
#pragma interrupt high_isr
void high_isr(void)
{
    static unsigned char state = STATE_BUTTON;
    static unsigned char port;
    static unsigned char row;

#ifndef ANALOG
    if (INTCONbits.TMR0IF) {
        TMR0H = 0xd8;           //2^16-10000
        TMR0L = 0xf0;
        INTCONbits.TMR0IF = 0;

        port = PORTB;

        if (state == STATE_BUTTON)
        {
            if (numberOFButtonsPressed(port) == 1)
            {
                row = rowFromPort(port);
                if (PWMFromKeyPad(gColumn,row) == PR2Value[6]) LATDbits.LATD1 = 0;
                setPWM(PWMFromKeyPad(gColumn, row));
                state = STATE_WAIT;
            }
            else
            {
                changeColumn();     //move on to next column but only if there is not button or waiting
            }
        }
        else if (state == STATE_WAIT)
        {
            if (PORTB&0x0F == 0x0F) state = STATE_BUTTON;
        }
    }
#endif

#ifdef ANALOG
    if (PIR1bits.ADIF)
    {
        PIR1bits.ADIF = 0;
        setPWM(ADRESH);
        LATB = ~PR2;                //negative logic
        ADCON0bits.GO = 1;
    }
#endif
}

#pragma interruptlow low_isr
void low_isr(void)
{
}


//**********************************************************************************************************



void setup(void)
{
    OSCCON = 0x63;          //Fosc = 4MHz, using internal osc

    // Setup Debug LED
    LATD = 0xFF;
    TRISDbits.TRISD1 = 0;

#ifndef ANALOG
    LATB = 0;               //latches always output ground
    TRISB = 0xFF;           //not outputting to keypad yet
    INTCON2bits.RBPU = 0;   //turns on internal pullups
#endif

#ifdef ANALOG
    LATB = 0xFF;
    TRISB = 0;
#endif

    //TMR0 setup
    T0CON = 0b01001000;     //off, 16 bit, int clk, no pre
    TMR0H = 0xd8;           //2^16-10000
    TMR0L = 0xf0;

    //TMR2 setup
    T2CON = 0b00000011;     //post = 1, pre = 16, off


    //PWM setup
    //step 1 - set period
    PR2 = 0b00001000;       //initially does not matter, set abit higher to avoid irratic behavior

    //step 2 - set duty
    CCPR1L = 0b00000000;    //initally duty = 0%

    //step 3 - set pin to output
    TRISCbits.TRISC2 = 0;

    //step 4 - set TMR2 prescaler
    T2CONbits.T2CKPS1 = 1;  //TMR2pre = 16

    //step 5 - config CCP1
    CCP1CON = 0b00001111;   //8 bit pwm mode


    T0CONbits.TMR0ON = 1;   //turns on TMR0
    T2CONbits.TMR2ON = 1;   //turns on TMR2

#ifdef ANALOG
    ADCON1 = 0x0E;          //Vdd, Vss, AN0 only
    ADCON0 = 0x00;          //select AN0
    ADCON2 = 0x39;          //very long acquisition time, right justify (keep MSB)
    ADCON0bits.ADON = 1;

    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;

    ADCON0bits.GO = 1;      //start AD conversions
#endif

    //enable global interupts

    INTCONbits.TMR0IF = 0;  //clears TMR0IF
    INTCONbits.TMR0IE = 1;  //enable TMR0IF

    INTCONbits.GIE = 1;     //enable global interupts
    INTCONbits.PEIE = 1;    //enable periferal interupts

}//end setup

void setPWM(unsigned char x)
{
    PR2 = x;
    CCPR1L = x/2;           //PWM duty = 50%
}//end setPWM

void changeColumn(void)
{
    gColumn++;

    if (gColumn > 3)        //1, 2, or 3 only
        gColumn = 1;

    TRISB = columnLookUp[gColumn];
}//end changeColumn

unsigned char PWMFromKeyPad(unsigned char column, unsigned char row)
{
    unsigned char temp;

    temp = ((row*3) + column);  //compiler bug, keep parens
    if (temp == 11) temp = 0;

    if (column == 3) LATDbits.LATD1 = 0;
    return PR2Value[temp];
}//end PWMFromKeyPad

unsigned char numberOFButtonsPressed(unsigned char port)
{
    unsigned char temp;
    unsigned char count = 0;
    unsigned char i;

    temp = port & 0x0f;

    for(i = 0; i < 4; i++)
    {
        if ((temp & (1<<i)) == 0x00)
            count++;
    }

    return count;
}//end numberOFButtonsPressed

//never call unless a single button is pressed
unsigned char rowFromPort(unsigned char port)
{
    unsigned char temp;
    unsigned char i;

    temp = port & 0x0f;

    for(i = 0; i < 4; i++)
    {
        if ((temp & (1<<i)) == 0x00)
            return i;
    }

    return 0;               //never hits here if called only when single button is pressed
}//end rowFromPort

void main(void)
{

    setup();
    // setPWM(PR2Value[4]); //debug
    while (1)
    {

    }//end infinite while

}//end main

