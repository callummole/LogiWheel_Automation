// LogiWheel.cpp : Defines the exported functions for the DLL application.
//

#pragma once

#include "stdafx.h"
// Logitech Wheel SDK
#pragma comment(lib, "LogitechSteeringWheelLib.lib")
#include "LogitechSteeringWheelLib.h"

#include <iostream>

#ifdef STEERING_API
#define STEERING_API __declspec(dllexport)
#else
#define STEERING_API __declspec(dllexport)
#endif


extern "C" __declspec(dllexport) BOOL initialise(HWND hndl) {

	return LogiSteeringInitializeWithWindow(TRUE, hndl);
}


extern "C" STEERING_API BOOL play_constant_force(const int magnitudePercentage) {
	if (LogiUpdate()) {
		return LogiPlayConstantForce(0, magnitudePercentage);
	}
}

extern "C" STEERING_API BOOL stop_constant_force() {
	if (LogiUpdate()) {
		return LogiStopConstantForce(0);
	}
}



extern "C" STEERING_API BOOL play_spring_force(const int offsetPercentage, const int saturationPercentage, const int coefficientPercentage) {
	if (LogiUpdate()) {
		return LogiPlaySpringForce(0, offsetPercentage, saturationPercentage, coefficientPercentage);
	}
}

extern "C" STEERING_API BOOL stop_spring_force() {
	if (LogiUpdate()) {
		return LogiStopSpringForce(0);
	}
}

extern "C" STEERING_API BOOL play_damper_force( const int coefficientPercentage) {
	if (LogiUpdate()) {
		return LogiPlayDamperForce(0, coefficientPercentage);
	}
}

extern "C" STEERING_API BOOL stop_damper_force() {
	if (LogiUpdate()) {
		return LogiStopDamperForce(0);
	}
}


extern "C" STEERING_API BOOL play_dirt_track(const int magnitudePercentage) {
	if (LogiUpdate()) {
		return LogiPlayDirtRoadEffect(0, magnitudePercentage);
	}
}

extern "C" STEERING_API BOOL stop_dirt_track() {
	if (LogiUpdate()) {
		return LogiStopDirtRoadEffect(0);
	}
}

extern "C" STEERING_API void shutdown() {
	if (LogiUpdate()) {
		return LogiSteeringShutdown();
	}
}

extern "C" STEERING_API int getState() {

DIJOYSTATE2ENGINES *ptr;
DIJOYSTATE2ENGINES wheelState;

	if (LogiUpdate()) {
		ptr = LogiGetStateENGINES(0);
		wheelState = *ptr;

		return wheelState.lX;


	}
}

