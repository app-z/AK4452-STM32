/*
 * This file is part of the firmware for the Andy's Workshop USB Microphone.
 * Copyright 2021 Andy Brown. See https://andybrown.me.uk for project details.
 * This project is open source subject to the license published on https://andybrown.me.uk.
 */

#pragma once

/**
 * Main program class
 */

class Program {

  private:

    Audio _audio;
    VolumeControl _volumeControl;
    GraphicEqualizer _graphicEqualiser;

  public:
    Program();

    void run();

    void setLevel(int16_t level);
    void process(int16_t *iobuffer, int32_t nSamples);
};

inline Program::Program() :
    _audio(_volumeControl, _graphicEqualiser) {
}

inline void Program::setLevel(int16_t level) {
	_volumeControl.setVolume(level);
}

inline void Program::process(int16_t *iobuffer, int32_t nSamples) {
	_volumeControl.process(iobuffer, nSamples);
}

inline void Program::run() {

//	setLevel(30);
  // infinite loop

//  for (;;) {

    // process the mute button
//  }
}
