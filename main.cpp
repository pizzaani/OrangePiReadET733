#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <wiringPi.h>

#define INT_PIN  29

volatile int counter = 0;

void bothEdge(void)
{
  //This is a workaround to clear ISR
  int v = digitalRead(INT_PIN)*2-1;
  
  counter+=v;
  printf("Calling %s with counter at %+d and read = %d\n",__func__, counter, v);
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

  if (wiringPiISR (pin, INT_EDGE_BOTH, &bothEdge) != 0)
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
