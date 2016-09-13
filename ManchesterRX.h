//
//  ManchesterRX.h
//  ManchesterRX
//
//  Created by Peter Goldsborough on 02/06/14.
//  Copyright (c) 2014 Peter Goldsborough. All rights reserved.
//

#ifndef __ManchesterRX__
#define __ManchesterRX__

#include <stdint.h>

#define RX_PIN A3

#define LED 13

#define MAN_HIGH 0b0011
#define MAN_LOW  0b1100

#define CONNECTION_TIME 77

#define BUFFER_SIZE 128

struct Queue
{
    uint8_t data[BUFFER_SIZE];
    
    uint8_t size;
};

int8_t popQ(struct Queue* queue);

void pushQ(struct Queue* queue, const uint8_t c);

uint8_t interpretSamples(const uint32_t samps);

void initRX(void);

void startRX(void);
void stopRX(void);

uint32_t getSamples(void);

uint16_t computeChecksum(const uint8_t* data, uint16_t bytes);

#endif
