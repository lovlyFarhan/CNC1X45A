
<!-- saved from url=(0058)http://www.mikrocontroller.net/attachment/18859/USI_UART.c -->
<html><head><meta http-equiv="Content-Type" content="text/html; charset=ISO-8859-1"></head><body><pre style="word-wrap: break-word; white-space: pre-wrap;">#include "USI_UART.h"


//********** USI UART Defines **********//

#define DATA_BITS                 8
#define START_BIT                 1
#define STOP_BIT                  1
#define HALF_FRAME                5

#define USI_COUNTER_MAX_COUNT     16
#define USI_COUNTER_SEED_TRANSMIT (USI_COUNTER_MAX_COUNT - HALF_FRAME)
#define INTERRUPT_STARTUP_DELAY   (0x11 / TIMER_PRESCALER)
//#define TIMER0_SEED               (256 - ( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ))
#define TIMER0_SEED               (256 - ( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ))+9

#if ( (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 3/2) &gt; (256 - INTERRUPT_STARTUP_DELAY) )
    #define INITIAL_TIMER0_SEED       ( 256 - (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 1/2) )
    #define USI_COUNTER_SEED_RECEIVE  ( USI_COUNTER_MAX_COUNT - (START_BIT + DATA_BITS) )
#else
    #define INITIAL_TIMER0_SEED       ( 256 - (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 3/2) )
    #define USI_COUNTER_SEED_RECEIVE  (USI_COUNTER_MAX_COUNT - DATA_BITS)
#endif

#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1 )
#if ( UART_RX_BUFFER_SIZE &amp; UART_RX_BUFFER_MASK )
    #error RX buffer size is not a power of 2
#endif

#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1 )
#if ( UART_TX_BUFFER_SIZE &amp; UART_TX_BUFFER_MASK )
    #error TX buffer size is not a power of 2
#endif

/* General defines */
#define TRUE                      1
#define FALSE                     0

//********** Static Variables **********//
register unsigned char USI_UART_TxData asm("r15");				// Tells the compiler to use Register 15 instead of SRAM

static unsigned char          UART_RxBuf[UART_RX_BUFFER_SIZE];  // UART buffers. Size is definable in the header file.
static volatile unsigned char UART_RxHead;
static volatile unsigned char UART_RxTail;
static unsigned char          UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;

static volatile union USI_UART_status                           // Status byte holding flags.
{
    unsigned char status;
    struct
    {
        unsigned char ongoing_Transmission_From_Buffer:1;
        unsigned char ongoing_Transmission_Of_Package:1;
        unsigned char ongoing_Reception_Of_Package:1;
        unsigned char reception_Buffer_Overflow:1;
        unsigned char flag4:1;
        unsigned char flag5:1;
        unsigned char flag6:1;
        unsigned char flag7:1;
    };
} USI_UART_status = {0};


//********** USI_UART functions **********//

// Reverses the order of bits in a byte.
// I.e. MSB is swapped with LSB, etc.
unsigned char Bit_Reverse( unsigned char x )
{
    x = ((x &gt;&gt; 1) &amp; 0x55) | ((x &lt;&lt; 1) &amp; 0xaa);
    x = ((x &gt;&gt; 2) &amp; 0x33) | ((x &lt;&lt; 2) &amp; 0xcc);
    x = ((x &gt;&gt; 4) &amp; 0x0f) | ((x &lt;&lt; 4) &amp; 0xf0);
    return x;    
}

// Flush the UART buffers.
void USI_UART_Flush_Buffers( void )  
{  
    UART_RxTail = 0;
    UART_RxHead = 0;
    UART_TxTail = 0;
    UART_TxHead = 0;
}

// Initialise USI for UART transmission.
void USI_UART_Initialise_Transmitter( void )                              
{
    cli();
    TCNT0  = 0x00;	//Z�hler0 auf 0 setzen
	TCCR0B = (0&lt;&lt;CS02)|(0&lt;&lt;CS01)|(1&lt;&lt;CS00);  			      // Reset the prescaler and start Timer0.
    TIFR   = (1&lt;&lt;TOV0);                                       // Clear Timer0 OVF interrupt flag.
    TIMSK |= (1&lt;&lt;TOIE0);                                      // Enable Timer0 OVF interrupt.
                                                                
    USICR  = (0&lt;&lt;USISIE)|(1&lt;&lt;USIOIE)|                         // Enable USI Counter OVF interrupt.
             (0&lt;&lt;USIWM1)|(1&lt;&lt;USIWM0)|                         // Select Three Wire mode.
             (0&lt;&lt;USICS1)|(1&lt;&lt;USICS0)|(0&lt;&lt;USICLK)|             // Select Timer0 OVER as USI Clock source.
             (0&lt;&lt;USITC);                                           
             
    USIDR  = 0xFF;                                            // Make sure MSB is '1' before enabling USI_DO.
    USISR  = 0xF0 |                                           // Clear all USI interrupt flags.
             0x0F;                                            // Preload the USI counter to generate interrupt at first USI clock.
    DDRB  |= (1&lt;&lt;PB1);                                        // Configure USI_DO as output.
                  
    USI_UART_status.ongoing_Transmission_From_Buffer = TRUE;
                  
    sei();
}

// Initialise USI for UART reception.
// Note that this function only enables pinchange interrupt on the USI Data Input pin.
// The USI is configured to read data within the pinchange interrupt.
void USI_UART_Initialise_Receiver( void )        
{  
    PORTB |=   (1&lt;&lt;PB1)|(1&lt;&lt;PB0);							// Enable pull up on USI DO and DI
    DDRB  &amp;= ~((1&lt;&lt;PB1)|(1&lt;&lt;PB0));							// Set USI DI and DO
    USICR  =  0;                                            // Disable USI.
    GIFR   =  (1&lt;&lt;PCIF);                                    // Clear pin change interrupt flag.
    GIMSK |=  (1&lt;&lt;PCIE);                                    // Enable pin change interrupt for PORTB
    PCMSK |=  (1&lt;&lt;PCINT0);									// Enable pin change interrupt for PB0
}

// Puts data in the transmission buffer, after reverseing the bits in the byte.
// Initiates the transmission rutines if not already started.
void USI_UART_Transmit_Byte( unsigned char data )          
{
    unsigned char tmphead;

    tmphead = ( UART_TxHead + 1 ) &amp; UART_TX_BUFFER_MASK;        // Calculate buffer index.
    while ( tmphead == UART_TxTail );                           // Wait for free space in buffer.
    UART_TxBuf[tmphead] = Bit_Reverse(data);                    // Reverse the order of the bits in the data byte and store data in buffer.
    UART_TxHead = tmphead;                                      // Store new index.
    
    if ( !USI_UART_status.ongoing_Transmission_From_Buffer )    // Start transmission from buffer (if not already started).
    {
        while ( USI_UART_status.ongoing_Reception_Of_Package ); // Wait for USI to finsh reading incoming data.
        USI_UART_Initialise_Transmitter();              
    }
}

// Returns a byte from the receive buffer. Waits if buffer is empty.
unsigned char USI_UART_Receive_Byte( void )                
{
    unsigned char tmptail;
        
    while ( UART_RxHead == UART_RxTail );                 // Wait for incomming data 
    tmptail = ( UART_RxTail + 1 ) &amp; UART_RX_BUFFER_MASK;  // Calculate buffer index 
    UART_RxTail = tmptail;                                // Store new index 
    return Bit_Reverse(UART_RxBuf[tmptail]);              // Reverse the order of the bits in the data byte before it returns data from the buffer.
}

// Check if there is data in the receive buffer.
unsigned char USI_UART_Data_In_Receive_Buffer( void )        
{
    return ( UART_RxHead != UART_RxTail );                // Return 0 (FALSE) if the receive buffer is empty.
}


// ********** Interrupt Handlers ********** //

// The pin change interrupt is used to detect USI_UART reseption.
// It is here the USI is configured to sample the UART signal.
ISR(SIG_PIN_CHANGE)
{                                                                    
    if (!( PINB &amp; (1&lt;&lt;PB0) ))                                     // If the USI DI pin is low, then it is likely that it
    {                                                             //  was this pin that generated the pin change interrupt.
        TCNT0  = INTERRUPT_STARTUP_DELAY + INITIAL_TIMER0_SEED;   // Plant TIMER0 seed to match baudrate (incl interrupt start up time.).
        TCCR0B  = (0&lt;&lt;CS02)|(0&lt;&lt;CS01)|(1&lt;&lt;CS00);  			      // Reset the prescaler and start Timer0.
        TIFR   = (1&lt;&lt;TOV0);                                       // Clear Timer0 OVF interrupt flag.
        TIMSK |= (1&lt;&lt;TOIE0);                                      // Enable Timer0 OVF interrupt.
                                                                    
        USICR  = (0&lt;&lt;USISIE)|(1&lt;&lt;USIOIE)|                         // Enable USI Counter OVF interrupt.
                 (0&lt;&lt;USIWM1)|(1&lt;&lt;USIWM0)|                         // Select Three Wire mode.
                 (0&lt;&lt;USICS1)|(1&lt;&lt;USICS0)|(0&lt;&lt;USICLK)|             // Select Timer0 OVER as USI Clock source.
                 (0&lt;&lt;USITC);                                           
                                                                  // Note that enabling the USI will also disable the pin change interrupt.
        USISR  = 0xF0 |                                           // Clear all USI interrupt flags.
                 USI_COUNTER_SEED_RECEIVE;                        // Preload the USI counter to generate interrupt.
                                                                  
        GIMSK &amp;=  ~(1&lt;&lt;PCIE);                                     // Disable pin change interrupt for PORTB
        
        USI_UART_status.ongoing_Reception_Of_Package = TRUE;             
    }
}

// The USI Counter Overflow interrupt is used for moving data between memmory and the USI data register.
// The interrupt is used for both transmission and reception.
ISR(SIG_USI_OVERFLOW)
{
    unsigned char tmphead,tmptail;
    
    // Check if we are running in Transmit mode.
    if( USI_UART_status.ongoing_Transmission_From_Buffer )      
    {
        // If ongoing transmission, then send second half of transmit data.
        if( USI_UART_status.ongoing_Transmission_Of_Package )   
        {                                   
            USI_UART_status.ongoing_Transmission_Of_Package = FALSE;    // Clear on-going package transmission flag.
            
            USISR = 0xF0 | (USI_COUNTER_SEED_TRANSMIT);                 // Load USI Counter seed and clear all USI flags.
            USIDR = (USI_UART_TxData &lt;&lt; 3) | 0x07;                      // Reload the USIDR with the rest of the data and a stop-bit.
        }
        // Else start sending more data or leave transmit mode.
        else
        {
            // If there is data in the transmit buffer, then send first half of data.
            if ( UART_TxHead != UART_TxTail )                           
            {
                USI_UART_status.ongoing_Transmission_Of_Package = TRUE; // Set on-going package transmission flag.
                
                tmptail = ( UART_TxTail + 1 ) &amp; UART_TX_BUFFER_MASK;    // Calculate buffer index.
                UART_TxTail = tmptail;                                  // Store new index.            
                USI_UART_TxData = UART_TxBuf[tmptail];                  // Read out the data that is to be sent. Note that the data must be bit reversed before sent.
                                                                        // The bit reversing is moved to the application section to save time within the interrupt.
                USISR  = 0xF0 | (USI_COUNTER_SEED_TRANSMIT);            // Load USI Counter seed and clear all USI flags.
                USIDR  = (USI_UART_TxData &gt;&gt; 2) | 0x80;                 // Copy (initial high state,) start-bit and 6 LSB of original data (6 MSB
                                                                        //  of bit of bit reversed data).                
            }
            // Else enter receive mode.
            else
            {
                USI_UART_status.ongoing_Transmission_From_Buffer = FALSE; 
                
                TCCR0B  = (0&lt;&lt;CS02)|(0&lt;&lt;CS01)|(0&lt;&lt;CS00);                // Stop Timer0.
                PORTB |=   (1&lt;&lt;PB1)|(1&lt;&lt;PB0);							// Enable pull up on USI DO and DI
                DDRB  &amp;= ~((1&lt;&lt;PB1)|(1&lt;&lt;PB0));							// Set USI DI and DO
                USICR  =  0;                                            // Disable USI
                GIFR   =  (1&lt;&lt;PCIF);                                    // Clear pin change interrupt flag
                GIMSK |=  (1&lt;&lt;PCIE);                                    // Enable pin change interrupt for PORTB
			    PCMSK |=  (1&lt;&lt;PCINT0);									// Enable pin change interrupt for PB0
            }
        }
    }
    
    // Else running in receive mode.
    else                                                                
    {              
        USI_UART_status.ongoing_Reception_Of_Package = FALSE;           

        tmphead     = ( UART_RxHead + 1 ) &amp; UART_RX_BUFFER_MASK;        // Calculate buffer index.
        
        if ( tmphead == UART_RxTail )                                   // If buffer is full trash data and set buffer full flag.
        {
            USI_UART_status.reception_Buffer_Overflow = TRUE;           // Store status to take actions elsewhere in the application code
        }
        else                                                            // If there is space in the buffer then store the data.
        {
            UART_RxHead = tmphead;                                      // Store new index.
            UART_RxBuf[tmphead] = USIDR;                                // Store received data in buffer. Note that the data must be bit reversed before used. 
        }                                                               // The bit reversing is moved to the application section to save time within the interrupt.
                                                                
        TCCR0B  = (0&lt;&lt;CS02)|(0&lt;&lt;CS01)|(0&lt;&lt;CS00);                // Stop Timer0.
        PORTB |=  (1&lt;&lt;PB1)|(1&lt;&lt;PB0);							// Enable pull up on USI DO and DI
        DDRB  &amp;= ~((1&lt;&lt;PB1)|(1&lt;&lt;PB0));							// Set USI DI and DO
        USICR  =  0;                                            // Disable USI.
        GIFR   =  (1&lt;&lt;PCIF);                                    // Clear pin change interrupt flag.
        GIMSK |=  (1&lt;&lt;PCIE);									// Enable pin change interrupt for PORTB
	    PCMSK |=  (1&lt;&lt;PCINT0);									// Enable pin change interrupt for PB0
    }
    
}
// Timer0 Overflow interrupt is used to trigger the sampling of signals on the USI ports.
ISR(SIG_OVERFLOW0)
{

    TCNT0 += TIMER0_SEED;                   // Reload the timer,
                                            // current count is added for timing correction.
}
</pre></body></html>