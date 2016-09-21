#define LED 13
#define RX_PIN A3
 
/* Toggle HIGH or LOW digital write */
volatile int toggle = 0;

volatile int sampleReady = 0;

volatile int sample = 0;

int upperThreshold = 70;

int lowerThreshold = 40;

void initRX(void)
{    
    TCCR1A = 0;     // set entire TCCR1A register to 0
    TCCR1B = 0;     // same for TCCR1B
  
    // set compare match register to desired timer count:
    OCR1A = 250;
    
    TCCR1B |= (1 << CS11);
    TCCR1B |= (1 << CS10);
    
    // turn on CTC mode:
    TCCR1B |= (1 << WGM12);
    // enable timer compare interrupt:
    TIMSK1 |= (1 << OCIE1A);
    sei();          // enable global interrupts
}

void setup(){
   pinMode(LED, OUTPUT);
   pinMode(RX_PIN, INPUT);
   initRX();
   Serial.begin(115200);
}

void loop(){
  while (sampleReady) {
    Serial.println(sample);   
  }
}


ISR(TIMER1_COMPA_vect)
{
   cli();
   int rawData = analogRead(RX_PIN);
   if (rawData < lowerThreshold || rawData > upperThreshold) {
       sample = rawData > upperThreshold ? 1 : 0;
       digitalWrite(LED, toggle == 0 ? HIGH : LOW);
       toggle = ~toggle;
       sampleReady = 1;
   }
   sei();
}
