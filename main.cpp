#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <wiringPi.h>

#define INT_PIN  29

// defines for the ET-733 thermometer
#define MAX_NOF_PREAMBLE_EDGES        8
#define MIN_NOF_PREAMBLE_EDGES        6
#define MEAN_PREAMBLE_EDGES_US        5250
#define DVTN_PREAMBLE_EDGES_US        50
#define LOWER_LIMIT_PREAMBLE_EDGES_US (MEAN_PREAMBLE_EDGES_US-DVTN_PREAMBLE_EDGES_US)
#define UPPER_LIMIT_PREAMBLE_EDGES_US (MEAN_PREAMBLE_EDGES_US+DVTN_PREAMBLE_EDGES_US)

#define NOF_DATA_BYTES                13
#define FULL_MANCHESTER_TIME_US       500
#define HALF_MANCHESTER_TIME_US       250
#define DVTN_MANCHESTER_TIME_US       125

#define LOWER_LIMIT_FULL_MANCHESTER_TIME_US (FULL_MANCHESTER_TIME_US-DVTN_MANCHESTER_TIME_US)
#define UPPER_LIMIT_FULL_MANCHESTER_TIME_US (FULL_MANCHESTER_TIME_US+DVTN_MANCHESTER_TIME_US)
#define LOWER_LIMIT_HALF_MANCHESTER_TIME_US (HALF_MANCHESTER_TIME_US-DVTN_MANCHESTER_TIME_US)
#define UPPER_LIMIT_HALF_MANCHESTER_TIME_US (HALF_MANCHESTER_TIME_US+DVTN_MANCHESTER_TIME_US)


//for tesing
#define TST_EDGE_COUNTER  (NOF_DATA_BYTES*2*8)

const unsigned char headerData[] = {0xAA, 0x99, 0x95};

enum t_state {DEFAULT, PREAMBLE, DATA};

volatile t_state state = DEFAULT;

volatile int counter = 0;
volatile int preambleCounter = 0;

volatile unsigned int currUs = 0;
volatile int diffUs = 0;
volatile int diffUs_fg = 0;
volatile unsigned int currMs = 0;
volatile int diffMs = 0;

volatile bool b_ignoreNextEdge = false;
// for Manchester decoding
//volatile unsigned int manchesterBitCounter = 0;
volatile bool b_manchester = true;
volatile unsigned int currDataByte = 0;
volatile unsigned int currBit = 0;
unsigned char data[NOF_DATA_BYTES];
volatile int ret_data = 0;

//for testing
volatile int arrayUs[TST_EDGE_COUNTER];
volatile unsigned int index_arrayUs = 0;

//for temperature
int probe1 = -500;
int probe2 = -500;
/**
  * Add new found bit to data
  * return true if successful, false otherwise 
  */
inline int addBit(unsigned char databit)
{
  data[currDataByte] |= databit << (7-currBit);
  currBit++;
  if (currBit == 8)
  {
    currBit = 0;
    currDataByte++;
  }
  if (currDataByte == NOF_DATA_BYTES)
  {
    return 2; //data is full
  }
  return 1;   //data not full yet
}


uint16_t shiftreg(uint16_t currValue)
{
  uint8_t msb = (currValue >> 15) & 0x01;
  currValue <<= 1;
  if (msb == 0x01)
  {
    currValue ^= 0x1021;
  }
  return currValue;
}

// data = nibbles 6-17 in binary form, i.e. 0x6A = 13 = b0111, 0x59 = 02 = b0010
uint16_t calculate_checksum(uint32_t data)
{
  uint16_t mask = 0x3331; //initia value of LFSR
  uint16_t csum = 0x0000;
  /*
  printf("Data intput:\n")
  for(int i = 23; i >= 0; --i)
  {
    printf("%02x", (data >> i) & 0x01);
  }
  */
  for(int i = 0; i < 24; ++i)
  {
    if((data >> i) & 0x01)
    {
      csum ^= mask;
    }
    mask = shiftreg(mask);
  }
  return csum;
}

// 0x AA9995       6A|59  xxxxx           xxxxx           xxxxxxxx
//    header(0-5)  (6-7)  Probe1(8-12)    Probe2(13-17)   Checksum(18-25)   26 Nibbles = 13 bytes
// 5=0, 6=1, 9=2, A=3

// data = 13 byte
// header =           3 byte
// syncdata =         1 byte
// probedata =  2x3.5 7 byte ??
// checksum =         2 byte
bool evaluateData(unsigned char *data_in, int *probe1_out, int *probe2_out)
{
  //check header
  if(memcmp(headerData, data_in, 3) != 0)
  {
    printf("Wrong header!");
    return false;
  }
  
  //disassemble data into nibbles
  uint32_t realData = 0x00000000;
  unsigned char currByte = 0x00;
  unsigned char nibble = 0x00;
  unsigned char result = 0x00;
  for (int k = 0; k < 6; ++k)
  {
    currByte = data_in[3+k];
    for(int l = 1; l >= 0; --l)
    {
      realData <<= 2;
      nibble = (currByte >> 4*l) & 0x0F;
      //printf("nibble = 0x%02x | ", nibble);
      result = ((nibble>>2) & 0x02) + ((nibble>>1) & 0x01); //5->00, 6->01, 9->10, A -> 11
      //printf("result = 0x%02x | ", result);
      realData |= (result & 0x00000003);
      //printf("realdata = 0x%08x | ", result);
    }
    //printf("\tdata_in = 0x%02x --> 0x%08x\n", currByte, realData);
  }
  // calculate temperatures
  *probe1_out = (int)((realData & 0x000FFC00)>>10)-532;
  *probe2_out = (int)((realData & 0x000003FF)>>00)-532;
  printf("***************************************\n");
  printf("* Probe #1 = %+03d°C | Probe #2 = %+03d°C *\n", *probe1_out, *probe2_out);
  printf("***************************************\n");
  diffMs = millis() - currMs;
  currMs = millis();
  printf("* Time since last call = %6.2fs       *\n", ((float)diffMs)/1000.0f);
  printf("***************************************\n");
  
  //calculate checksum
  uint16_t checksum = calculate_checksum(realData);
  printf("\tchecksum = 0x%04x \n", checksum);
  //TODO: control checksum with received checksum!!!
  return true;
}


void showCurrData(void)
{
  printf("Current Data:\n");
  for (unsigned int k = 0; k <= currDataByte; ++k)
  {
    printf("0x%02x ", data[k]);
  }
  printf("\n");
}

void showTestData(void)
{
  printf("Current Test Data:\n");
  for (unsigned int k = 0; k < index_arrayUs; ++k)
  {
    printf("% 6dus ", arrayUs[k]);
  }
  printf("\n");
}


void resetData(const char *str)
{
  //showCurrData();
  //showTestData();
  memset((void*)data, 0x00, NOF_DATA_BYTES);
  b_manchester = true;
  currDataByte = 0;
  currBit = 0;
  ret_data = 0;
  state = DEFAULT;
  //printf("Data reset!!\n");
  printf("Reason for reset: '%s'\n", str);
  
  //for testing
  index_arrayUs = 0;
  memset((void*)arrayUs, 0, TST_EDGE_COUNTER);
}

void bothEdge(void)
{
  //This is a workaround to clear ISR
  //int v = digitalRead(INT_PIN)*2-1;
  bool b_rising = (digitalRead(INT_PIN)==1);
  
  switch (state)
  {
    case DEFAULT:
    case PREAMBLE:
      if (b_rising)
      {
        diffUs = micros() - currUs;
        currUs = micros();
        
        if (preambleCounter == 0)
        {  
          preambleCounter++;
        }
        else
        {
          if ((diffUs > LOWER_LIMIT_PREAMBLE_EDGES_US ) && (diffUs < UPPER_LIMIT_PREAMBLE_EDGES_US) &&  preambleCounter < MAX_NOF_PREAMBLE_EDGES) 
          {
            preambleCounter++;
            state = PREAMBLE;
          }
          else
          {
            state = DEFAULT;
            preambleCounter = 0;
          }
          
          // check if preamble end has been reached
          if (preambleCounter == MAX_NOF_PREAMBLE_EDGES)
          {
            state = DATA;
            preambleCounter = 0;
            b_ignoreNextEdge = true;
          }
          
          /*
          else
          {
            if (preambleCounter >= MIN_NOF_PREAMBLE_EDGES)
            {
              state = DATA;
              // Take first data bit
              addBit(0x01);
              printf("preambleCounter = %d\n", preambleCounter);
              diffMs = millis() - currMs;
              currMs = millis();
              //printf("Time since last call = %6.2fs\n", ((float)diffMs)/1000.0f);
            }
            else
            {
              state = DEFAULT;
            }
            preambleCounter = 0;
            */
        }
      }
      else  //falling
      {
        //TODO: fill falling edge verification
      }
    break;  //DEFAULT & DATA
    
    //  Record data
    case DATA:
      diffUs = micros() - currUs;
      currUs = micros();
      //printf("Edge is %s\n",b_rising?"rising":"falling");
      //printf("diffUs = % 6dus.\n", diffUs);
      //state = DEFAULT;
      //break;
      
      // check if it is the first falling edge, which is still preamble
      if (b_ignoreNextEdge & (!b_rising))
      {
        b_ignoreNextEdge = false;
        break;
      }
      
      //for testing
      if (index_arrayUs < TST_EDGE_COUNTER)
      {
        arrayUs[index_arrayUs] = diffUs;
        index_arrayUs++;
      } 
      
      //printf("I am in data recording state with diffUs = %d us\n", diffUs);
      ret_data = 0;
      
      if ((diffUs > LOWER_LIMIT_FULL_MANCHESTER_TIME_US) && (diffUs < UPPER_LIMIT_FULL_MANCHESTER_TIME_US))
      {
        if (b_manchester)
        {
          ret_data = addBit(b_rising);
        }
        else  //error in data stream, reset everything
        {
          resetData("b_manchester == false");
        }
      }
      else if ((diffUs > LOWER_LIMIT_HALF_MANCHESTER_TIME_US) && (diffUs < UPPER_LIMIT_HALF_MANCHESTER_TIME_US))
      {
        // toggle
        b_manchester = !b_manchester;
        if (b_manchester)
        {
          ret_data = addBit(b_rising);
        }
      }
      else if ((diffUs > UPPER_LIMIT_HALF_MANCHESTER_TIME_US) && (currBit == 0) && (currDataByte == 0)) //this is first bit
      {
        ret_data = addBit(b_rising);
      }
      else  //error in data stream, reset everything
      {
        resetData("timing error in data stream");
      }
      
      // check if data is complete
      if (ret_data == 2)
      {
      /*
        printf("Data is completed:\n");
        for (unsigned int k = 0; k < NOF_DATA_BYTES; ++k)
        {
          printf("0x%02x ", data[k]);
        }
        printf("\n");
        */
        printf(" **** Data is completed ****\n");
        evaluateData(data, &probe1, &probe2);
        resetData("Data completed");
      }
    break;  //DATA
  
    default:  
      state = DEFAULT;
  } //end of switch(state)
}

int main(int argc, char **argv)
{
  printf("This is the start of readET733!\n");
  
  resetData("Start");
  
  if (wiringPiSetup() != 0)
  {
    printf("wiringPi Setup failed! Exiting!\n");
    return 1; 
  }

  int pin = INT_PIN;
  pinMode(pin, INPUT);

  if (wiringPiISR (pin, INT_EDGE_BOTH, &bothEdge) != 0)
  {
    printf("wiringPi ISR failed! Exiting!\n");
    return 1;
  }

  while(1)
  {
    //delay(1000);
    //printf("Slept 1000 ms\n");
  };

  printf("Returning...\n");
  return 0;
}
