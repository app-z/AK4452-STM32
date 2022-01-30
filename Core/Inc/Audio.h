/*
 * This file is part of the firmware for the Andy's Workshop USB Microphone.
 * Copyright 2021 Andy Brown. See https://andybrown.me.uk for project details.
 * This project is open source subject to the license published on https://andybrown.me.uk.
 */

#pragma once

//#include "audioplay.h"

/**
 * Audio management functionality. Many of the methods in here are called from the
 * usbd_audio_if.cpp file.
 */

//extern uint8_t Audio_Buffer[AUDIO_BUFFER_SIZE];


class Audio {

  private:
    // 20ms of 64 bit samples ?

    volatile int16_t *_sampleBuffer;
    int16_t *_processBuffer;
    int16_t *_sendBuffer;

    VolumeControl &_volumeControl;
    GraphicEqualizer &_graphicEqualiser;
    bool _running;
    uint8_t _zeroCounter;

  public:
    Audio(VolumeControl &volumeControl, GraphicEqualizer &graphicEqualiser);

  public:
    static Audio *_instance;

  public:
    void setVolume(int16_t volume);

    void i2s_halfComplete();
    void i2s_complete();

    int8_t start();
    int8_t stop();
    int8_t pause();
    int8_t resume();

    const GraphicEqualizer& getGraphicEqualizer() const;

  private:
    void processData(volatile int offset);
};

/**
 * Constructor
 */

inline Audio::Audio(VolumeControl &volumeControl,  GraphicEqualizer &graphicEqualiser) :
    _volumeControl(volumeControl), _graphicEqualiser(graphicEqualiser) {


  // initialise variables

  Audio::_instance = this;
  _running = false;
  _zeroCounter = 0;

  // allocate buffers

//  _sampleBuffer = new int16_t[AUDIO_BUFFER_SIZE];      // 7680 bytes (*2 because samples are 64 bit ?)
//  _processBuffer = new int16_t[AUDIO_BUFFER_SIZE];         // 1920 bytes
//  _sendBuffer = new int16_t[AUDIO_BUFFER_SIZE];            // 1920 bytes

}

inline const GraphicEqualizer& Audio::getGraphicEqualizer() const {
  return _graphicEqualiser;
}

/**
 * Start the I2S DMA transfer (called from usbd_audio_if.cpp)
 */

inline int8_t Audio::start() {

  HAL_StatusTypeDef status;

  // HAL_I2S_Receive_DMA will multiply the size by 2 because the standard is 24 bit Philips.

//  if ((status = HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*) _sampleBuffer, MIC_SAMPLES_PER_PACKET * 2)) == HAL_OK) {
//    _running = true;
//  }

  return status;
}

/**
 * Stop the I2S DMA transfer
 */

inline int8_t Audio::stop() {

  HAL_StatusTypeDef status;

  if ((status = HAL_I2S_DMAStop(&hi2s2)) == HAL_OK) {
    _running = false;
  }
  return status;
}

/**
 * Pause I2S DMA transfer (soft-mute)
 */

inline int8_t Audio::pause() {

  HAL_StatusTypeDef status;

  if ((status = HAL_I2S_DMAPause(&hi2s2)) == HAL_OK) {
    _running = false;
  }

  return status;
}

/**
 * Resume I2S DMA transfer
 */

inline int8_t Audio::resume() {

  HAL_StatusTypeDef status;

  if ((status = HAL_I2S_DMAResume(&hi2s2)) == HAL_OK) {
    _running = true;
  }
  return status;
}

/**
 * Set the volume gain: the mute state is preserved
 */

inline void Audio::setVolume(int16_t volume) {

  // reduce resolution from 1/256dB to 1/2dB

  volume /= 128;

  // ensure SVC library limits are respected

  if (volume < -160) {
    volume = -160;
  } else if (volume > 72) {
    volume = 72;
  }

  _volumeControl.setVolume(volume);
}

/**
 * Get a reference to the graphic equalizer
 */

/**
 * 1. Transform the I2S data into 16 bit PCM samples in a holding buffer
 * 2. Use the ST GREQ library to apply a graphic equaliser filter
 * 3. Use the ST SVC library to adjust the gain (volume)
 * 4. Transmit over USB to the host
 *
 * We've got 10ms to complete this method before the next DMA transfer will be ready.
 */

inline void Audio::processData(volatile int offset) {

  // only do anything at all if we're connected

  if (_running) {

    // ensure that the mute state in the smart volume control library matches the mute
    // state of the hardware button. we do this here to ensure that we only call SVC
    // methods from inside an IRQ context.

      // adjust the gain (volume) using the ST SVC library

//      _volumeControl.process((int16_t*)&Audio_Buffer[offset], AUDIO_BUFFER_SIZE/2);

  }
}

