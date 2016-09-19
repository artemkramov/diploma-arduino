#define OUT_PIN 4

int toggle = 0;

void setup() {
  pinMode(OUT_PIN, OUTPUT);
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B

  // set compare match register to desired timer count:
  OCR1A = 400;
  
  TCCR1B |= (1 << CS11);
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void loop() {
  // put your main code here, to run repeatedly:

}

ISR(TIMER1_COMPA_vect)
{
   digitalWrite(OUT_PIN, toggle ? HIGH : LOW);
   toggle = ~toggle;
}
