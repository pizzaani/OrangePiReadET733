#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <wiringPi.h>

#define INT_PIN  29

volatile uint32_t counter = 0;

void risingEdge(void)
{
  digitalRead(INT_PIN);
  printf("Calling %s with counter at %d\n",__func__, counter);
  counter++;
}

int main(int argc, char **argv)
{
  printf("This is the start of readET733!\n");
  if (wiringPiSetup() != 0)
  {
    printf("wiringPi Setup failed! Exiting!\n");
    return 1; 
  }

  int pin = INT_PIN;
  pinMode(pin, INPUT);
  pullUpDnControl (pin, PUD_DOWN); 

  if (wiringPiISR (pin, INT_EDGE_RISING, &risingEdge) != 0)
  {
    printf("wiringPi ISR failed! Exiting!\n");
    return 1;
  }

  while(1)
  {
    delay(1000);
    printf("Slept 1000 ms\n");
  }

  printf("Returning...\n");
  return 0;
}
