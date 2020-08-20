#ifndef __SHARED_TOOLS_H_INCLUDED__
#define __SHARED_TOOLS_H_INCLUDED__

#include <vector>

	union num32_t
	{
		int32_t i;
		uint32_t ui;
		float f;
		uint8_t c[4];
	};

	union num16_t
	{
		int16_t i;
		uint16_t ui;
		uint8_t c[2];
	};

#endif