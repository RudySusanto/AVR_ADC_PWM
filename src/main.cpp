#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define LED_PIN    (PB5) // Arduino PB5/D13 pin 
#define PWM_PIN    (PB1) // Arduino OC1A/PB1/D9 pin

#define FREQ_HZ    (1000UL)
#define PRESCALER  (8)
#define TOP_VALUE  (F_CPU/(2 * PRESCALER * FREQ_HZ))

volatile uint32_t sum = 0;
volatile uint8_t count = 0;

// The ADC interrupt flag will be cleared automatically
// after the ISR(ADC_vect) is called. 
ISR(ADC_vect) { 
  uint16_t adcValue = ADC; // Save the current ADC value
  PINB |= (1<<LED_PIN); // Toggle PB5
  count++;         // Increment sample count
  sum += adcValue; // Add sample to sum
  if (count == 4) { // 4 samples
    // Update the PWM duty cycle: (sum/4/1024) * TOP_VALUE
    OCR1A = (uint16_t)(TOP_VALUE*sum>>12);
    sum   = 0; // Clear sum of samples
    count = 0; // Reset sample count
  }
  ADCSRA |= (1<<ADIF); // Clear the ADC interrupt flag (not necessary)
}

void initADC() {
  // Set PC0/A0 as an input pin
  DDRC &= ~(1 << DDC0);
  // Disable Digital Input Buffer on A0
  DIDR0 |= (1<<ADC0D);
  // Set reference voltage to AVCC
  ADMUX = (1 << REFS0); 
  // Right-Adjust Result (ADLAR=0)
  ADMUX &= ~(1<<ADLAR); 
  // Select A0 as ADC input ADMUX[3:0]="0000"
  ADMUX &= ~((1<<MUX3)|(1<<MUX2)|(1<<MUX1)|(1<<MUX0));
  // Set ADC prescaler to 128 (ADPS[2:0]="111")
  // 16MHz/128 = 125kHz ADC clock
  ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
  // Enable ADC (Free running), enable ADC interrupt 
  ADCSRB = 0; // ADTS[2:0]="000"
  ADCSRA |= (1<<ADEN) | (1<<ADATE) | (1<<ADIE) | (1<<ADSC);
  //ADCSRA |= (1<<ADSC); // Start the next conversion
}

// Use Timer1 to create a PWM signal (PWM Phase-Correct Mode)
void initTimer1() { 
  TCCR1A = TCCR1B = 0;
  TCNT1 = 0x0000;
  // Set the ICR1 register to define the TOP value
  ICR1 = (uint16_t)(TOP_VALUE);
  // Set the OCR1A register to define the PWM duty cycle
  OCR1A = 0;
  // Set Timer1 in Phase-correct PWM mode (mode 10)
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13);
  // Set the output compare mode to clear OC1A on compare match 
  // and set on TOP (non-inverting)
  TCCR1A |= (1 << COM1A1);
  // Set the prescaler to 8 (CLK/8)
  TCCR1B |= (1 << CS11);
}

int main(void) {
  uint16_t value;
  DDRB |= (1 << LED_PIN);
  DDRB |= (1 << PWM_PIN);
  initADC();
  initTimer1();
  sei(); // Enable global interrupts
  while (1) {}
}