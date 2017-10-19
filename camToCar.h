#pragma once
#ifndef _camToCar_H_
#define _camToCar_H_

#include <stdint.h>

struct camToCar
{
	//data to send 
	uint64_t tStmp;
	int32_t id;
	int32_t latitude, longitude;

	int16_t vx, vy;

	int16_t heading;
	uint16_t length, width;
};

#endif