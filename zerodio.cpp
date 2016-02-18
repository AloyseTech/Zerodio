/*
 * Copyright AloyseTech(c) 2015
 * based on AudioZero library from Arduino Team
 *
 * Audio library for Arduino Zero.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "arduino.h"
#include "zerodio.h"
#include <SD.h>
#include <SPI.h>


/*Global variables*/
bool __audioFileReady = false;
volatile uint32_t __SampleIndex;
uint32_t __HeadIndex;
uint32_t __NumberOfSamples; // Number of samples to read in block
uint8_t *__WavSamples;
File __audioFile;
int __Volume;
uint32_t fileSize=0;
uint32_t elapsedBytes=0;
uint32_t srate=0;

void ZerodioClass::begin(uint32_t sampleRate) {
	
	__audioFileReady = false;
	__SampleIndex = 0;					//in order to start from the beginning
	__NumberOfSamples = 256;            //samples to read to have a buffer
    srate=sampleRate;
	/*Allocate the buffer where the samples are stored*/
	__WavSamples = (uint8_t *) malloc(__NumberOfSamples * sizeof(uint8_t));
	
	/*Modules configuration */
  	dacConfigure();
	tcConfigure(sampleRate);
    
    
}

void ZerodioClass::end() {
	tcDisable();
	tcReset();
	analogWrite(A0, 0);
    
}


void ZerodioClass::play(const char *fname) {
    if(__audioFileReady)
        __audioFile.close();
    __audioFile = SD.open(fname);
    fileSize=__audioFile.size();
    
    if(!__audioFile){
        end();
        return;
    }
    
    for(int i =0; i<44; i++)
        __audioFile.read();
    
    __audioFile.read(__WavSamples, __NumberOfSamples);
    __HeadIndex = 0;
    
    /*once the buffer is filled for the first time the counter can be started*/
    tcStartCounter();
    
    __audioFileReady = true;
    
}

uint32_t ZerodioClass::duration()
{
    if(__audioFileReady)
        return (fileSize/srate);
    return 0;
}

uint32_t ZerodioClass::remaining()
{
    if(__audioFileReady)
        return ((fileSize-elapsedBytes)/srate);
    return 0;
}


bool ZerodioClass::isPLaying()
{
    return __audioFileReady;
}


/**
 * Configures the DAC in event triggered mode.
 *
 * Configures the DAC to use the module's default configuration, with output
 * channel mode configured for event triggered conversions.
 */
void ZerodioClass::dacConfigure(void){
	analogWriteResolution(10);
	analogWrite(A0, 0);
}

/**
 * Configures the TC to generate output events at the sample frequency.
 *
 * Configures the TC in Frequency Generation mode, with an event output once
 * each time the audio sample frequency period expires.
 */
 void ZerodioClass::tcConfigure(uint32_t sampleRate)
{
	// Enable GCLK for TCC2 and TC5 (timer counter input clock)
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
	while (GCLK->STATUS.bit.SYNCBUSY);

	tcReset();

	// Set Timer counter Mode to 16 bits
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

	// Set TC5 mode as match frequency
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;

	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;

	TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
	while (tcIsSyncing());
	
	// Configure interrupt request
	NVIC_DisableIRQ(TC5_IRQn);
	NVIC_ClearPendingIRQ(TC5_IRQn);
	NVIC_SetPriority(TC5_IRQn, 0);
	NVIC_EnableIRQ(TC5_IRQn);

	// Enable the TC5 interrupt request
	TC5->COUNT16.INTENSET.bit.MC0 = 1;
	while (tcIsSyncing());
}	


bool ZerodioClass::tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void ZerodioClass::tcStartCounter()
{
  // Enable TC

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}

void ZerodioClass::tcReset()
{
  // Reset TCx
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

void ZerodioClass::tcDisable()
{
  // Disable TC5
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}

ZerodioClass AudioPlayer;

#ifdef __cplusplus
extern "C" {
#endif

void Audio_Handler (void)
{
    if (__audioFile.available()) {
        
        
            uint32_t current__SampleIndex = __SampleIndex;
            
            if (current__SampleIndex > __HeadIndex) {
                __audioFile.read(&__WavSamples[__HeadIndex], current__SampleIndex - __HeadIndex);
                __HeadIndex = current__SampleIndex;
            }
            else if (current__SampleIndex < __HeadIndex) {
                __audioFile.read(&__WavSamples[__HeadIndex],__NumberOfSamples-1 - __HeadIndex);
                __audioFile.read(__WavSamples, current__SampleIndex);
                __HeadIndex = current__SampleIndex;
            }
        
        
        if (__SampleIndex < __NumberOfSamples - 1)
        {
            analogWrite(A0, __WavSamples[__SampleIndex++]);
            elapsedBytes++;
            // Clear the interrupt
            //TC5->COUNT16.INTFLAG.bit.MC0 = 1;
        }
        else
        {
            __SampleIndex = 0;
            //TC5->COUNT16.INTFLAG.bit.MC0 = 1;
        }
    }
    else if (__audioFileReady){
        __audioFile.close();
        __audioFileReady = false;
        elapsedBytes=0;
        //tc reset
        //TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
        //while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
        //while (TC5->COUNT16.CTRLA.bit.SWRST);
    
        //tc disable
        TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
        while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
        
        analogWrite(A0, 512);
    }

    // Clear the interrupt
    TC5->COUNT16.INTFLAG.bit.MC0 = 1;
  
}

void TC5_Handler (void) __attribute__ ((weak, alias("Audio_Handler")));

#ifdef __cplusplus
}
#endif