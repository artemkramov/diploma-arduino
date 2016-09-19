#include "ManchesterGlobal.h"
#include "ManchesterRX.h"

volatile uint64_t samples = 0;

volatile uint8_t samplesReady = 0;

volatile int8_t sampleCount = DATA_SAMPLES - 1;

uint8_t receiving = 0, data = 0, connectionCount = 0;

int levelBit = 70;

/* Timer2 reload value, globally available */
unsigned int tcnt2;
 
/* Toggle HIGH or LOW digital write */
int toggle = 0;

void setup(){
   pinMode(LED, OUTPUT);
   pinMode(AN_PIN, OUTPUT);
   pinMode(RX_PIN, INPUT);
   pinMode(DG_PIN, INPUT);
   initRX();
   Serial.begin(250000);
}

void loop(){
   struct Queue q;
   q.size = 0;
   uint16_t chksm;
   
   startRX();
   Serial.println("0");        
}

void initRX(void)
{    
    TCCR1A = 0;     // set entire TCCR1A register to 0
    TCCR1B = 0;     // same for TCCR1B
  
    // set compare match register to desired timer count:
    OCR1A = 200;
    
    TCCR1B |= (1 << CS11);
    // turn on CTC mode:
    TCCR1B |= (1 << WGM12);
    // enable timer compare interrupt:
    TIMSK1 |= (1 << OCIE1A);
    sei();          // enable global interrupts
}

uint32_t getSamples(void)
{
    /* Reset flag */
    samplesReady = 0;
    return samples;
}

uint16_t computeChecksum(const uint8_t* data, uint16_t bytes)
{
    /* * * * * * * * * * * * * * * * * * * * * * * * * *
     *                                                 *
     *  Apparently 0xFF (255) is preferred over 0      *
     *  because it provides better shielding against   *
     *  overflow and prevents the checksum from ever   *
     *  becoming zero, therefore providing a checksum  *
     *  computed flag when the returned value is non-  *
     *  zero. Taken from here: http://goo.gl/Cjt4AO    *
     *                                                 *
     * * * * * * * * * * * * * * * * * * * * * * * * * */
    
    uint16_t sum1 = 0xFF;
    uint16_t sum2 = 0xFF;
    
    uint16_t wordLen;
    
    while (bytes)
    {
        /* maximum word size before overflow would occur */
        wordLen = (bytes > 20) ? 20 : bytes;
        
        bytes -= wordLen;
        
        do
        {
            sum1 += *(data++);
            sum2 += sum1;
        }
        
        while (--wordLen);
        
        /* * * * * * * * * * * * * * * * * * * * * *
         * Do first modular reduction: mask off
         * lower byte and add right shifted upper
         * byte to it
         * * * * * * * * * * * * * * * * * * * * * */
        
        sum1 = (sum1 & 0xFF) + (sum1 >> 8);
        sum2 = (sum2 & 0xFF) + (sum2 >> 8);
    }
    
    /* Safety reduction */
    sum1 = (sum1 & 0xFF) + (sum1 >> 8);
    sum2 = (sum2 & 0xFF) + (sum2 >> 8);
    
    /* Return both sums in a 16 bit value */
    return sum2 << 8 | sum1;
}

void pushQ(struct Queue* queue, const uint8_t c)
{
    if (queue->size < BUFFER_SIZE)
    { queue->data[queue->size++] = c; }
}

void startRX(void)
{
    uint8_t preambleBit, lows = 0, highs = 0;
    
    //SET_BIT(PORTB,LED);
    
    /* Reset */
    //TCNT0 = TCNT1 = 0;
    
    /* Enable while waiting for the preamble. Is turned false when the connection
     times out. */
    receiving = 1;
    
    /* Going to record individual bits now */
    sampleCount = 3;
    
    /* Reset this. Otherwise all RX attempts will fail after the first one. Not sure why. */
    samples = 0;

    int cycles = 0;
    
    /* receiving becomes false if the connection times out, so this will not go on forever */
    while (receiving)
    {
        if (samplesReady)
        {
            /* Grab bit */
            preambleBit = getSamples();
            
            sampleCount = 3;
            
            Serial.println(preambleBit);
            /* The preamble consists of 6 low bits (10) and 2 high bits (01), everything
             else or any other order than that resets the counter values to 0 */

//            if (preambleBit == 0b1010) {
//              if (++lows > 6) {
//                break;
//              }
//            }
//            else {
//              lows = 0;
//            }
//
//            continue;
            
//            if (preambleBit == MAN_LOW)
//            {
//                if (!highs) ++lows;
//                
//                else lows = highs = 0;
//            }
//            
//            else if (preambleBit == MAN_HIGH)
//            {
//                if (lows >= 6)
//                {
//                    if (++highs >= 2) break;
//                }
//                
//                else lows = highs = 0;
//            }
//            
//            else
//            {
//                lows = highs = 0;
//                
//                /*
//                 * If there's a one-sample error, make the next samples finish earlier so the
//                 * next sampling starts one sample earlier. This means that it takes at most
//                 * 3 samplings to get valid bits. The timing is not a problem since the Pin
//                 * change interrupt makes sure that any pin changes occur exactly between two
//                 * samples, therefore there can only be sample errors and not timing errors
//                 * (I think).
//                 */
//                
//                sampleCount = 2;
//            }
        }
    }
    Serial.println("here");
    /* Normal sampling now */
    sampleCount = 3;
}

void stopRX(void)
{
    /* Stop the wait for the preamble if it's still running,
     else stop waiting for samples in main */
    receiving = 0;
    
    /* Disable sampling Timer/Counter */
    CLEAR_BIT(TIMSK0,OCIE0A);
    
    //CLEAR_BIT(PORTB,LED);
}

uint8_t interpretSamples(const uint32_t samps)
{
    int8_t i = 7;
    
    uint8_t bit;
    
    for (; i >= 0; --i)
    {
        /* Grab the current bit */
        bit = (samps >> (i*4)) & 0x0F;
        
        if (bit == MAN_HIGH) SET_BIT(data,i);
        
        else if (bit == MAN_LOW) CLEAR_BIT(data,i);
        
        else return 0;
    }
    
    return 1;
}

ISR(TIMER1_COMPA_vect)
{
     //int rawData = analogRead(RX_PIN);
     //int sample = rawData > levelBit ? 1 : 0;
     int sample = digitalRead(DG_PIN);
     digitalWrite(LED, sample == 0 ? HIGH : LOW);
     /* If the current sample reads high, set the current bit in the samples */
    if (sample == 1)
    {
        /* Reset timer now, after sampling */
        //TCNT1 = 1;
        SET_BIT(samples,sampleCount);
    }
    
    /* Else clear the current bit */
    else
    {
        /* Reset timer now, after sampling */
        //TCNT1 = 1;
        CLEAR_BIT(samples,sampleCount);
    }
    /* If the bit is finished, set the samplesReady flag */
    if (!sampleCount--)
    {
      samplesReady = 1;
      sampleCount = 3; //DATA_SAMPLES - 1;
    }
}
