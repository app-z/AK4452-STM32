/*
 * This file is part of the firmware for the Andy's Workshop USB Microphone.
 * Copyright 2021 Andy Brown. See https://andybrown.me.uk for project details.
 * This project is open source subject to the license published on https://andybrown.me.uk.
 */

#pragma once
extern "C" {

   #include "main.h"
   #include "greq_glo.h"
   #include "svc_glo.h"

	extern I2S_HandleTypeDef hi2s2;
}

// include our classes
#include "VolumeControl.h"
#include "GraphicEqualizer.h"
#include "Audio.h"
#include "Program.h"

// for the Debug_Semihosting configuration

#ifdef SEMIHOSTING
#include <stdio.h>
#endif
