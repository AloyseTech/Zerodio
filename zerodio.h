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
	
#ifndef ZERODIO_H
#define ZERODIO_H

#include "Arduino.h"
#include "Print.h"

#include <SD.h>
#include <SPI.h>


class ZerodioClass{
public:

	ZerodioClass() {};
	void begin(uint32_t sampleRate);
    void play(const char *fname) ;
	void end();

private:
	void dacConfigure(void);
	void tcConfigure(uint32_t sampleRate);
	bool tcIsSyncing(void);
	void tcStartCounter(void);
	void tcReset(void);
	void tcEnable(void);
	void tcDisable(void);
};

extern ZerodioClass AudioPlayer;
#endif