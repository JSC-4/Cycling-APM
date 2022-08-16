#ifndef MAFILTER_H
#define MAFILTER_H

#include <stdint.h>

#define MA_FILTER_LENGTH 10

typedef struct {
	float 	buf[MA_FILTER_LENGTH];
	uint8_t bufIndex;

	float out;
} MAFilter;

void MA_Init(MAFilter *ma);
float MA_Update(MAFilter *ma, float input);

#endif //MAFILTER_H
