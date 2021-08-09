/*
 *                                                           PIC24FJ128GA204
 *               +----------------+               +--------------+                +-----------+                +--------------+
 * J4_MOSI <>  1 : RB9/RP9        :    LED2 <> 12 : RA10         :   J4_SDA <> 23 : RB2/RP2   : X2-32KHZ <> 34 : RA4/SOSCO    :
 *  RGB_G  <>  2 : RC6/RP22       :         <> 13 : RA7          :   J4_SCL <> 24 : RB3/RP3   :     LED1 <> 35 : RA9          :
 *  RGB_B  <>  3 : RC7/RP23       :         <> 14 : RB14/RP14    :      POT <> 25 : RC0/RP16  :    J4_CS <> 36 : RC3/RP19     :
 *     S2  <>  4 : RC8/RP24       :         <> 15 : RB15/RP15    :          <> 26 : RC1/RP17  :   J4_SCK <> 37 : RC4/RP20     :
 *     S1  <>  5 : RC9/RP25       :     GND -> 16 : AVSS         :          <> 27 : RC2/RP18  :    RGB_R <> 38 : RC5/RP21     :
 *    3v3  <>  6 : VBAT           :     3v3 -> 17 : AVDD         :      3v3 -> 28 : VDD       :      GND -> 39 : VSS          :
 *   10uF  ->  7 : VCAP           : ICD_VPP -> 18 : MCLR         :      GND -> 29 : VSS       :      3v3 -> 40 : VDD          :
 *         <>  8 : RB10/RP11/PGD2 :   J4_AN <> 19 : RA0/AN0      :          <> 30 : RA2/OSCI  :    J4_RX <> 41 : RB5/RP5/PGD3 :
 *         <>  9 : RB11/RP11/PGC2 :  J4_RST <> 20 : RA1/AN1      :          <> 31 : RA3/OSCO  :    J4_TX <> 42 : RB6/RP6/PGC3 :
 *         <> 10 : RB12/RP12      : ICD_PGD <> 21 : RB0/RP0/PGD1 :          <> 32 : RA8       :          <> 43 : RB7/RP7      :
 *         <> 11 : RB13/RP13      : ICD_PGC <> 22 : RB1/RP1/PGC1 : X2-32KHZ <> 33 : RB4/SOSCI :  J4_MISO <> 44 : RB8/RP8      :
 *               +----------------+               +--------------+                +-----------+                +--------------+
 *                                                              TQFP-44
 * 
 *
 * 
 */
// CONFIG4
#pragma config DSWDTPS = DSWDTPS1F      // Deep Sleep Watchdog Timer Postscale Select bits (1:68719476736 (25.7 Days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select (DSWDT uses LPRC as reference clock)
#pragma config DSBOREN = OFF            // Deep Sleep BOR Enable bit (DSBOR Disabled)
#pragma config DSWDTEN = OFF            // Deep Sleep Watchdog Timer Enable (DSWDT Disabled)
#pragma config DSSWEN = OFF             // DSEN Bit Enable (Deep Sleep operation is always disabled)
#pragma config PLLDIV = PLL4X           // USB 96 MHz PLL Prescaler Select bits (4x PLL selected)
#pragma config I2C1SEL = DISABLE        // Alternate I2C1 enable bit (I2C1 uses SCL1 and SDA1 pins)
#pragma config IOL1WAY = OFF            // PPS IOLOCK Set Only Once Enable bit (The IOLOCK bit can be set and cleared using the unlock sequence)

// CONFIG3
#pragma config WPFP = WPFP127           // Write Protection Flash Page Segment Boundary (Page 127 (0x1FC00))
#pragma config SOSCSEL = ON             // SOSC Selection bits (SOSC circuit selected)
#pragma config WDTWIN = PS25_0          // Window Mode Watchdog Timer Window Width Select (Watch Dog Timer Window Width is 25 percent)
#pragma config PLLSS = PLL_FRC          // PLL Secondary Selection Configuration bit (PLL is fed by the on-chip Fast RC (FRC) oscillator)
#pragma config BOREN = OFF              // Brown-out Reset Enable (Brown-out Reset Disabled)
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable (Disabled)
#pragma config WPCFG = WPCFGDIS         // Write Protect Configuration Page Select (Disabled)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMD = NONE            // Primary Oscillator Select (Primary Oscillator Disabled)
#pragma config WDTCLK = LPRC            // WDT Clock Source Select bits (WDT uses LPRC)
#pragma config OSCIOFCN = ON            // OSCO Pin Configuration (OSCO/CLKO/RA3 functions as port I/O (RA3))
#pragma config FCKSM = CSECMD           // Clock Switching and Fail-Safe Clock Monitor Configuration bits (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)
#pragma config FNOSC = FRC              // Initial Oscillator Select (Fast RC Oscillator (FRC))
#pragma config ALTCMPI = CxINC_RB       // Alternate Comparator Input bit (C1INC is on RB13, C2INC is on RB9 and C3INC is on RA0)
#pragma config WDTCMX = WDTCLK          // WDT Clock Source Select bits (WDT clock source is determined by the WDTCLK Configuration bits)
#pragma config IESO = OFF               // Internal External Switchover (Disabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler Select (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler Ratio Select (1:128)
#pragma config WINDIS = OFF             // Windowed WDT Disable (Standard Watchdog Timer)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config ICS = PGx1               // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config LPCFG = OFF              // Low power regulator control (Disabled - regardless of RETEN)
#pragma config GWRP = OFF               // General Segment Write Protect (Write to program memory allowed)
#pragma config GCP = OFF                // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (Disabled)
/*
 * Defines for system oscillator frequency.
 * Make sure that the PIC initialization selects this frequency.
 */
#define FSYS (32000000ul)
#define FCY  (FSYS/2ul)
/*
 * Target specific Special Function Register definitions
 */
#include <xc.h>
/*
 * Standard header files
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define _ISR_AUTO_PSV __attribute__((__interrupt__, __auto_psv__))

/*
 * Initialize this PIC
 */
void PIC_Init(void)
{
    uint16_t ClockSwitchTimeout;

    /* 
     * Disable all interrupt sources 
     */ 
    
    __builtin_disi(0x3FFF); /* disable interrupts for 16383 cycles */ 
    IEC0 = 0; 
    IEC1 = 0; 
    IEC2 = 0; 
    IEC3 = 0; 
    IEC4 = 0; 
    IEC5 = 0; 
    IEC6 = 0; 
    IEC7 = 0; 
    __builtin_disi(0x0000); /* enable interrupts */ 
    
    INTCON1bits.NSTDIS = 1; /* Disable interrupt nesting */

    if(!OSCCONbits.CLKLOCK) /* if primary oscillator switching is unlocked */
    {
        /* Select primary oscillator as FRC */
        __builtin_write_OSCCONH(0b000);

        /* Request switch primary to new selection */
        __builtin_write_OSCCONL(OSCCON | (1 << _OSCCON_OSWEN_POSITION));

        /* wait, with timeout, for clock switch to complete */
        for(ClockSwitchTimeout=10000; --ClockSwitchTimeout && OSCCONbits.OSWEN;);

        CLKDIV   = 0x0000; /* set for FRC clock 8MHZ operations */

        /* Select primary oscillator as FRCPLL */
        __builtin_write_OSCCONH(0b001);
        /* Request switch primary to new selection */
        __builtin_write_OSCCONL(OSCCON | (1 << _OSCCON_OSWEN_POSITION));
        
        /* ALERT: This may be required only when the 96MHz PLL is used */
        CLKDIVbits.PLLEN = 1;

        /* wait, with timeout, for clock switch to complete */
        for(ClockSwitchTimeout=10000; --ClockSwitchTimeout && OSCCONbits.OSWEN;);

        /* wait, with timeout, for the PLL to lock */
        for(ClockSwitchTimeout=10000; --ClockSwitchTimeout && !OSCCONbits.LOCK;);
        
        /* at this point the system oscillator should be 32MHz */
    }
    
    /* Turn on Secondary Oscillation Amplifier */
    __builtin_write_OSCCONL(OSCCON | (1 << _OSCCON_SOSCEN_POSITION));
    
    /* Turn off all analog inputs */
    ANSA = 0;
    ANSB = 0;
    ANSC = 0;
}

void delay_us(unsigned short delay)
{
    if (delay > (uint16_t)(16383.0 / (FCY/1E6)))
    {
        asm("   repeat  #16383\n"
            "   clrwdt        \n"
            ::);

    }
    else
    {
        asm("   repeat  %0    \n"
            "   clrwdt        \n"
        :: "r" (delay*(uint16_t)(FCY/1000000ul)-1));
    }
}
/*
 * WARNING: Not a portable function.
 *          Maximum 16MHz instruction cycle clock.
 *          Minimum  8Khz instruction cycle clock.
 */
void delay_ms(unsigned long delay)
{
    if(delay)
    {
        asm("1: repeat  %0    \n"
            "   clrwdt        \n"
            "   sub  %1,#1    \n"
            "   subb %d1,#0   \n"
            "   bra  nz,1b    \n"
        :: "r" (FCY/1000ul-6ul), "C" (delay));
    }
}

/*
 * Initialize the UART
 */
void UART_Init(void)
{
/**    
     Set the UART1 module to run at 9600 baud with RB6 as TX out and RB5 as RX in.
*/
     //USED IN PRELIMINARY TESTING
    //TRISBbits.TRISB6 = 0;
    //LATBbits.LATB6 = 1;
    
    TRISCbits.TRISC8 = 0;
    LATCbits.LATC8 = 1;
    __builtin_write_OSCCONL(OSCCON & ~_OSCCON_IOLOCK_MASK); // unlock PPS (peripheral pin select)
    
    //USED IN PRELIMINARY TESTING
    //RPINR18bits.U1RXR = 0x0005;    //RB5->UART1:U1RX
    //RPOR3bits.RP6R    = 0x0003;    //RB6->UART1:U1TX
    
    //ON ACTUAL BOARD
    RPINR18bits.U1RXR = 0x0019;    //RC9->UART1:U1RX
    //  This sets U1RXR register to RP25 (0x0019) / RC9      
    
    RPOR12bits.RP24R = 0x0003;    //RC8->UART1:U1TX
    //  This assigns pin RP24 to U1TX function (0x0003)
    //  SEE TABLE 11-4 FOR FULL LIST OF UART OUTPUT FUNCTIONS
    
    __builtin_write_OSCCONL(OSCCON | _OSCCON_IOLOCK_MASK); // lock PPS
    
    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; URXINV disabled; UEN TX_RX; 
    // Data Bits = 8; Parity = None; Stop Bits = 1;
    U1MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit
    // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; URXEN disabled; OERR NO_ERROR_cleared; URXISEL 4 Bytes (Interrupt after receiving 4 bytes); UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
    U1STA = 0xC0;
    // BaudRate = 9600; Frequency = 16000000 Hz; U1BRG 416; 
    U1BRG = 0x1A0;
    // ADMADDR 0; ADMMASK 0; 
    U1ADMD = 0x00;
    // T0PD 1 ETU; PTRCL T0; TXRPT Retransmits the error byte once; CONV Direct; SCEN disabled; 
    U1SCCON = 0x00;
    // TXRPTIF disabled; TXRPTIE disabled; WTCIF disabled; WTCIE disabled; PARIE disabled; GTCIF disabled; GTCIE disabled; RXRPTIE disabled; RXRPTIF disabled; 
    U1SCINT = 0x00;
    // GTC 0; 
    U1GTC = 0x00;
    // WTCL 0; 
    U1WTCL = 0x00;
    // WTCH 0; 
    U1WTCH = 0x00;
    
    U1MODEbits.UARTEN = 1;   // enabling UART ON bit
    U1STAbits.UTXEN = 1;
    
    IFS0bits.U1RXIF = 0;  //initialize interrupt to zero  

}

uint8_t UART1_Read(void)
{
    //while(!(U1STAbits.URXDA == 1)); //wait until receive buffer receives something
    while(!U1STAbits.URXDA);
    if ((U1STAbits.OERR == 1))
    {
        U1STAbits.OERR = 0; //clearing receive buffer if Overrun Error Status Bit is enabled
    }
    
    return U1RXREG; //return value in U1RXREG
}

void UART1_Write(uint8_t txData)
{
    while(U1STAbits.UTXBF == 1); //wait while Transfer Buffer is full

    U1TXREG = txData;    // Write the data byte to the USART.
}

bool UART1_IsRxReady(void)
{
    return U1STAbits.URXDA;
}

bool UART1_IsTxReady(void)
{
    return ((!U1STAbits.UTXBF) && U1STAbits.UTXEN );
}

bool UART1_IsTxDone(void)
{
    return U1STAbits.TRMT;
}

int __attribute__((__section__(".libc.write"))) write(int handle, void *buffer, unsigned int len) 
{
    unsigned int i;

    for (i = len; i; --i)
    {
        UART1_Write(*(char*)buffer++);
    }
    return(len);
}

/*
 * Define receive buffer
 */
//separating into chunks
uint16_t dataArray1[1023];
uint16_t addressArray1[1023]; 

int counterSamplesReceived = 0; //keeps track of samples received via UART
int counterArray = 0; //keeps track of where in array data should be saved

//made global variable, not local variable in interrupt function
int16_t divisorUnit = 0;
int16_t numSamples = 0;
uint16_t channel = 0;
uint16_t valueRead = 0; //variable to store contents read from UART buffer

int readCase = 0; //creating variable to control logic for what is being read
//if 0, then first values are being read.
//  -These are the number of sample points and divisor_unit
//if 1, then sample points and respective addresses are being read

int beginRead = 0; //used for testing read function

void _ISR_AUTO_PSV _U1RXInterrupt()
{    
    while (UART1_IsRxReady() == 1) //says will always evaluate to true in compiler -- observationally not true
    {
        switch(readCase)
        {
            case 0:
                //first getting contents for number of samples
                valueRead = UART1_Read();
                valueRead <<= 8;
                valueRead += UART1_Read();
                numSamples = valueRead;
                
                //now getting number of divisor units
                valueRead = UART1_Read();
                valueRead <<= 8;
                valueRead += UART1_Read();
                divisorUnit = valueRead;
                
                //need to add code for getting output channel from UI
                valueRead = UART1_Read();
                channel = valueRead;
                
                if (channel == 0x0001)
                {
                    valueRead = 1;
                }
                
                readCase++;
                break;
                
            case 1:
                //storing values of address into array
                valueRead = UART1_Read();
                valueRead <<= 8; //shift left 8 bits
                valueRead += UART1_Read();
                addressArray1[counterArray] = valueRead; //store data point into dataArray
        
                
                valueRead = UART1_Read();
                valueRead <<= 8; //shift left 8 bits
                valueRead += UART1_Read();
                dataArray1[counterArray] = valueRead; //store data point into dataArray
                
                counterArray++;
                counterSamplesReceived++;

                break;
            
        }
        
        if ( (counterSamplesReceived > 0 && counterSamplesReceived%1023 == 0) || counterSamplesReceived == numSamples)
        {
            //set Write signal high telling FPGA that data is loaded and ready to send
            LATBbits.LATB13 = 1;
            
            int checkReadPin = 0;
            
            //wait for signal that FPGA is ready to receive data
            //this will be used when integrating systems
            //while (checkReadPin != 1)
                //checkReadPin = PORTBbits.RB13;
                            
            //function to send divisor_Unit and channel
            writeDivisorUnit_command(divisorUnit, channel);
            
            writeToBus();
            counterArray = 0; //reset to zeroth element in array
            
            if (counterSamplesReceived != numSamples)
            {
                UART1_Write(66); //send 'B' to MCU to signal transfer of new data block if max number of samples has not been reached
                //if statement will probably not be necessary when always sending max number of samples
            }
            
            if (counterSamplesReceived == numSamples)
            {
                readCase = 0; //once all data has been received, reset to read first line of new data file 
                counterSamplesReceived = 0;
                
                //reset file parameter variables to prepare for next file received
                //numSamples = 0;
                divisorUnit = 0;
                channel = 0;
                UART1_Write(66); //send 'B' to MCU to signal transfer of new data block if max number of samples has not been reached

                
                //UART1_Write(66); //send 'B' to MCU to signal transfer of new data block if max number of samples has not been reached
                beginRead = 1; //set to 1 to begin read, put after write function to avoid infinite loop in UI

            }
            
            //set Write signal low until more data is stored
            LATBbits.LATB13 = 0;
        }
                
    }
    
    IFS0bits.U1RXIF = 0;    

}

void clockWrite()
{
    //turn pin defined as clock (RA1 - 'CLK' on PCB schematic) high, then low to signal to FPGA to store contents on bus
    LATAbits.LATA1 = 1;
    LATAbits.LATA1 = 0;
    
}

void noOperation_command()
{
    //command pin defined as output and set to high to signal incoming command
    TRISBbits.TRISB11 = 0;
    LATBbits.LATB11 = 1;

    //set bus to '00' which signifies write command
    LATAbits.LATA9 = 0;
    LATCbits.LATC3 = 0;
    clockWrite(); //write contents of bus to FPGA   
    
    LATBbits.LATB11 = 0; //set command pin low
}

void writeCommand()
{
    //command pin set to high to signal incoming command
    LATBbits.LATB11 = 1;
    
    //set bus to '01' which signifies write command
    LATAbits.LATA9 = 1;
    LATCbits.LATC3 = 0;
    clockWrite(); //write contents of bus to FPGA
    
    //set back to low
    LATBbits.LATB11 = 0;
}

void readCommand()
{
    //command pin set to high to signal incoming command
    LATBbits.LATB11 = 1;
    
    //set bus to '10' which signifies write command
    LATAbits.LATA9 = 0;
    LATCbits.LATC3 = 1;
    clockWrite(); //write contents of bus to FPGA
    
    LATBbits.LATB11 = 0;

    readFromFPGA();
}

void writeDivisorUnit_command(uint16_t divisor_unit, uint16_t channel)
{
    //send 11 on bus
    //then send the divisor unit
    //need to find largest possible divisor unit size
    //command pin set to high to signal incoming command
    LATBbits.LATB11 = 1;
    
    //set bus to '11' which signifies write command
    LATAbits.LATA9 = 1;
    LATCbits.LATC3 = 1;
    clockWrite(); //write contents of bus to FPGA
    
    LATBbits.LATB11 = 0;
    
    int pin = 0;
    
    //check if output channel is 0 or 1
    if (channel == 0x01)
    {
        pin = 1;
    }
    
    else pin = 0;
    
    LATAbits.LATA9 = pin;
    clockWrite();
    
    //need to check if can send both divisor unit and channel bit in one cycle, rather than separately 
     int i = 0;
     int bitcount = 0;
        for (i = 0; i < 2; i++)
        {
            for (bitcount = 0; bitcount < 8; bitcount++)
            {
                if (divisor_unit & 0x01)
                {
                    pin = 1;
                }
                
                else pin = 0;
                
                switch (bitcount)
                {
                    case 0:
                        LATAbits.LATA9 = pin;
                        break;
                            
                    case 1:
                        LATCbits.LATC3 = pin;
                        break;
                        
                    case 2:
                        LATCbits.LATC4 = pin;
                        break;
                        
                    case 3:
                        LATCbits.LATC5 = pin;
                        break;
                        
                    case 4:
                        LATBbits.LATB5 = pin;
                        break;
                        
                    case 5:
                        LATBbits.LATB6 = pin;
                        break;
                        
                    case 6:
                        LATBbits.LATB7 = pin;
                        break;           

                    case 7:
                        LATBbits.LATB8 = pin;
                        break;  
                            
                }
                
                divisor_unit >>= 1; //shift next bit to LSB place for comparison and writing
            }
            
            clockWrite();
        }
}

void configureReadPins()
{
    //configuring pins for bus
    TRISAbits.TRISA9 = 1; //configure A9 as input
    TRISCbits.TRISC3 = 1; //configure C3 as input
    TRISCbits.TRISC4 = 1; //configure C4 as input
    TRISCbits.TRISC5 = 1; //configure C5 as input
    TRISBbits.TRISB5 = 1; //configure B5 as input
    TRISBbits.TRISB6 = 1; //configure B6 as input
    TRISBbits.TRISB7 = 1; //configure B7 as input
    TRISBbits.TRISB8 = 1; //configure B8 as input
    
    //Setting Read pin to output and Write pin to input
    TRISBbits.TRISB12 = 0;
    TRISBbits.TRISB13 = 1;
}


bool checkArray[1023]; //this array stores booleans of check between contents of data sent to FPGA and data received from FPGA
int readCounter = 0;
void readFromFPGA()
{
    configureReadPins();
    
    int i = 0;
    int bitcount = 0;
    uint16_t pin = 0;
    int index = 0;
    uint16_t addressRead = 0;
    uint16_t sampleRead = 0;
     
    uint16_t comparisonAddress =0;
    uint16_t comparisonSample = 0;
    LATBbits.LATB12 = 1; //tell FPGA MCU is ready to read
     
    //while(PORTAbits.RA1 != 1); //wait for FPGA to set clock pin high in order to start reading the contents of the bus
     
    while (index < 1024 && (numSamples != readCounter))
    {
        comparisonAddress = addressArray1[index];
        comparisonSample = dataArray1[index];
                
        for (i = 0; i < 2; i++)
        {
            for (bitcount = 0; bitcount < 8; bitcount++)
            {   
                switch (bitcount)
                {
                    case 7:
                        pin = PORTAbits.RA9;
                        addressRead += pin;
                        break;
                            
                    case 6:
                        pin = PORTCbits.RC3;
                        addressRead += pin;
                        break;
                        
                    case 5:
                        pin = PORTCbits.RC4;
                        addressRead += pin;
                        break;
                        
                    case 4:
                        pin = PORTCbits.RC5;
                        addressRead += pin;                       
                        break;
                        
                    case 3:
                        pin = PORTBbits.RB5;
                        addressRead += pin;
                        break;
                        
                    case 2:
                        pin = PORTBbits.RB6;
                        addressRead += pin;
                        break;
                        
                    case 1:
                        pin = PORTBbits.RB7;
                        addressRead += pin;                        
                        break;           

                    case 0:
                        pin = PORTBbits.RB8;
                        addressRead += pin;                        
                        break;  
                            
                }
                //reads in value from left to right in bits
                //AKA case 0 will read RB8, which gives MSB value of 8 bits received
                if (bitcount != 7)
                    addressRead <<= 1; //shift next bit to left to get in proper place
            }
        }
        
        //reading corresponding sample point now
        for (i = 0; i < 2; i++)
        {
            for (bitcount = 0; bitcount < 8; bitcount++)
            {   
                switch (bitcount)
                {
                    case 7:
                        pin = PORTAbits.RA9;
                        sampleRead += pin;
                        break;
                            
                    case 6:
                        pin = PORTCbits.RC3;
                        sampleRead += pin;
                        break;
                        
                    case 5:
                        pin = PORTCbits.RC4;
                        sampleRead += pin;
                        break;
                        
                    case 4:
                        pin = PORTCbits.RC5;
                        sampleRead += (pin & 0x0);
                        break;
                        
                    case 3:
                        pin = PORTBbits.RB5;
                        sampleRead += (pin == 0x0);
                        break;
                        
                    case 2:
                        pin = PORTBbits.RB6;
                        sampleRead += (pin == 0x0);
                        break;
                        
                    case 1:
                        pin = PORTBbits.RB7;
                        sampleRead += pin;                        
                        break;           

                    case 0:
                        pin = PORTBbits.RB8;
                        sampleRead += pin;                        
                        break;  
                            
                }
                if (i == 0 || (i == 1 && bitcount != 7) )
                    sampleRead <<= 1; //shift next bit to left to get in proper place
                
                pin = 0;
            }
        }
        /*
        if (addressArray1[index] == addressRead && dataArray1[index] == sampleRead)
            checkArray[index] = true;
        */
        bool checkValue = false;
        
       if (dataArray1[index] == sampleRead)
            checkValue = true;
        else
            checkValue = false;
        
        if (index != 1023)
            readCounter++;
        
        addressRead = 0;
        sampleRead = 0;
        index++;
    }
    
}

void configureWritePins()
{
    //configuring pins for bus
    TRISAbits.TRISA9 = 0; //configure A9 as output
    TRISCbits.TRISC3 = 0; //configure C3 as output
    TRISCbits.TRISC4 = 0; //configure C4 as output
    TRISCbits.TRISC5 = 0; //configure C5 as output
    TRISBbits.TRISB5 = 0; //configure B5 as output
    TRISBbits.TRISB6 = 0; //configure B6 as output
    TRISBbits.TRISB7 = 0; //configure B7 as output
    TRISBbits.TRISB8 = 0; //configure B8 as output
    
    //set clock as output pin

    // command pin set to output
    TRISBbits.TRISB11 = 0;
    
    //Setting Read pin to input and Write pin to output
    TRISBbits.TRISB12 = 1;
    TRISBbits.TRISB13 = 0;
}

int totalCounter = 0; //keeps count of all the sample points sent
void writeToBus()
{  
    //counter for index of bits to write to port and dataArray and addressArray
    int bitcount = 0;
    int arrayIndex = 0;
    
    //variables to store data received over UART
    uint16_t dataAddress = 0;
    uint16_t dataSample = 0;
    
    //pin that holds binary value of the LSB of sample point
    int pin = 0;
    writeCommand(); //let FPGA know it is about to receive a command
    
    int index = 0; //keeps index of data being sent
    //need to add logic for not sending full 1024 points in case that max resolution is below 1024
    //current logic is to keep count of total number of samples sent and send until reaches numSamples - totalCounter = 0 or 1024 samples are sent
    //totalCounter is variable outside of this function so will be able to keep accurate total of data points sent
    while (index < 1024 && (numSamples != totalCounter))
    {
        dataAddress = addressArray1[index];
        dataSample = dataArray1[index];
        
        //sending address of sample point first
        //then sending corresponding sample point
        int i = 0;
        for (i = 0; i < 2; i++)
        {
            for (bitcount = 0; bitcount < 8; bitcount++)
            {
                if (dataAddress & 0x01)
                {
                    pin = 1;
                }
                
                else pin = 0;
                
                switch (bitcount)
                {
                    case 0:
                        LATAbits.LATA9 = pin;
                        break;
                            
                    case 1:
                        LATCbits.LATC3 = pin;
                        break;
                        
                    case 2:
                        LATCbits.LATC4 = pin;
                        break;
                        
                    case 3:
                        LATCbits.LATC5 = pin;
                        break;
                        
                    case 4:
                        LATBbits.LATB5 = pin;
                        break;
                        
                    case 5:
                        LATBbits.LATB6 = pin;
                        break;
                        
                    case 6:
                        LATBbits.LATB7 = pin;
                        break;           

                    case 7:
                        LATBbits.LATB8 = pin;
                        break;  
                            
                }
                
                dataAddress >>= 1; //shift next bit to LSB place for comparison and writing
            }
            
            clockWrite();
        }
        
        //now sending sample points
        for (i = 0; i < 2; i++)
        {
            for (bitcount = 0; bitcount < 8; bitcount++)
            {
                if (dataSample & 0x01)
                {
                    pin = 1;
                }
                
                else pin = 0;
                
                switch (bitcount)
                {
                    case 0:
                        LATAbits.LATA9 = pin;
                        break;
                            
                    case 1:
                        LATCbits.LATC3 = pin;
                        break;
                        
                    case 2:
                        LATCbits.LATC4 = pin;
                        break;
                        
                    case 3:
                        LATCbits.LATC5 = pin;
                        break;
                        
                    case 4:
                        LATBbits.LATB5 = pin;
                        break;
                        
                    case 5:
                        LATBbits.LATB6 = pin;
                        break;
                        
                    case 6:
                        LATBbits.LATB7 = pin;
                        break;           

                    case 7:
                        LATBbits.LATB8 = pin;
                        break;  
                            
                }
                
                
                dataSample >>= 1; //shift next bit to LSB place for comparison and writing

            }

            clockWrite();
        }
        if (index != 1023)
        {
            totalCounter++;
        }
        index++;
    }
    
    //if (totalCounter == numSamples)
    //    noOperation_command(); //this tells the FPGA that the MCU does not have any further data to send
}

/*
 * Main Application
 */
int main(void) 
{
    /*
     * Application initialization
     */
    PIC_Init();
    UART_Init();
    configureWritePins();

    //interrupts
    IEC0bits.U1RXIE = 1; //enable UART1 Receive Interrupt
    IPC2bits.U1RXIP = 0b111; //Set interrupt to highest priority
    
    //printf("\r\nPIC24FJ128GA204 built on " __DATE__ " at " __TIME__ " Start\r\n");
   
    /*
     * Application process loop
     */
    for(;;)
    {
        //noOperation_command();
        //writeCommand();
        //if (beginRead == 1)
        //{
        //    readFromFPGA();
        //}
    }

    return 0;
}