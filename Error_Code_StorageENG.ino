#include <avr/io.h>
#include <avr/interrupt.h>

uint8_t DTCflag = 0;
uint8_t DTC = 0;
uint8_t cnt1s = 0;
uint8_t printDTC = 0;

//Definition of ring buffer with size and structure
#define RINGBUFFER_SIZE 10 
struct ringbuffer
{
  uint8_t data[RINGBUFFER_SIZE];
  uint8_t head;
  uint8_t tail;
};
ringbuffer RingBuffer;  

void newDTC(uint8_t DTC) //Function with transfer of DTC value as new DTC
{
  if (RingBuffer.head == (RINGBUFFER_SIZE - 1)) 
  {
    RingBuffer.data[0] = DTC;
  } else {
    RingBuffer.data[RingBuffer.head] = DTC;
  }
  RingBuffer.head = (RingBuffer.head + 1) % RINGBUFFER_SIZE;
  return;
}

int main()
{
  init();
  Serial.begin(115200);
  Serial.println("HP2 A4 Fehlerspeicher");
  RingBuffer.head = 0;
  RingBuffer.tail = 0;

  //INPUTS
  DDRD &= ~(1 << DDD2);  //INT0
  DDRD &= ~(1 << DDD3);  //INT1
  DDRC &= ~(1 << DDC0);  //PCINT8

  
  //enable internal pull-ups
  PORTC |= (1 << PORTC0);
  PORTD |= (1 << PORTD2) | (1 << PORTD3);
  DDRB |= (1 << DDB5);  //Debug LED

  //Interrupts Configuration
  EICRA |= (1 << ISC01);
  EICRA &= ~(1 << ISC00);
  EIMSK |= (1 << INT0);
  
  EICRA |= (1 << ISC11);
  EICRA &= ~(1 << ISC10);
  EIMSK |= (1 << INT1);

  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8);

  // Timer Settings & Prescaler
  TCCR1A = 0;              
  TCCR1B = 0;              
  TCCR1B |= (1 << WGM12);  //CTC Modus 

  OCR1A = 15625;                        
  TCCR1B |= (1 << CS12) | (1 << CS10);  

  //Timer1 Output Compare Interrupt A 
  TIMSK1 |= (1 << OCIE1A); // When Timer1 reaches the value in OCR1A (15625) bzw. 1 Sekunde, it triggers an interrupt, and the ISR TIMER1_COMPA_vect runs

  sei();

  while (1)
  {
    if (DTCflag == 1)  
    {
      newDTC(DTC);  
      DTCflag = 0;
    }
    if (cnt1s == 1)  
    {
      Serial.println("DTC buffer - Ringbuffer:");
      
      for (printDTC = 0; printDTC < RINGBUFFER_SIZE; printDTC++) 
      {
        DTC = RingBuffer.data[printDTC];
        Serial.print(DTC);
        Serial.print(" | ");
      }
      Serial.println(); 
      cnt1s = 0;
    }
  }
}

ISR(INT0_vect)
{
  DTCflag++;
  DTC = 0xA1;
  Serial.println(DTC);
}

ISR(INT1_vect)
{
  DTCflag++;
  DTC = 0xB2;
  Serial.println(DTC);
}

ISR(PCINT1_vect)
{
  if (PINC & (1 << PC0)) 
  {
    DTCflag++;
    DTC = 0xC3;
    Serial.println(DTC);
  }
}

ISR(TIMER1_COMPA_vect)
{
  cnt1s = 1; // 
  PORTB ^= (1 << PD5); 
}
