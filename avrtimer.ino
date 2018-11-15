void timer2SetUp()
{
  //set timer2 interrupt at 1kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 1khz increments
  OCR2A = 250;
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 64 prescaler
  TCCR2B |= (1 << CS22);   
  
}
void timer2Start()
{
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);  
}
void timer2Stop()
{
  // enable timer compare interrupt
  TIMSK2 &= ~(1 << OCIE2A);  
}
