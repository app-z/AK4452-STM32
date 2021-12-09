/*
 * This file is part of the firmware for the Andy's Workshop USB Microphone.
 * Copyright 2021 Andy Brown. See https://andybrown.me.uk for project details.
 * This project is open source subject to the license published on https://andybrown.me.uk.
 */

#include "Application.h"

extern "C" {

Program program;

	void myMain() {
		program.run();
	}


	void setLevel(int16_t level) {
		program.setLevel(level);
	}


	void process(int16_t *iobuffer, int32_t nSamples) {
		program.process(iobuffer, nSamples);
	}

}


