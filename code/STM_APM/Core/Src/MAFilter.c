#include "MAFilter.h"

static float MA_IMPULSE_RESPONSE[MA_FILTER_LENGTH] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f};


void MA_Init(MAFilter *fir) {

	/* Clear filter buffer */
	for (uint8_t n = 0; n < MA_FILTER_LENGTH; n++) {
		fir->buf[n] = 0.0f;
	}
	/* Reset buffer index */
	fir->bufIndex = 0;

	/* Clear filter output */
	fir->out = 0.0f;

}

float MA_Update(MAFilter *ma, float input) {

	/* Store latest sample in buffer */
	ma->buf[ma->bufIndex] = input;

	/* Increment buffer index and wrap around if necessary */
	ma->bufIndex++;

	if (ma->bufIndex == MA_FILTER_LENGTH) {
		ma->bufIndex = 0;
	}

	/* Compute new output sample (via convolution) */
	ma->out = 0.0f;
	uint8_t sumIndex = ma->bufIndex;

	for (uint8_t n = 0; n < MA_FILTER_LENGTH; n++) {
		/* Decrement index and wrap if necessary */
		if (sumIndex > 0) {
			sumIndex--;
		} else {
			sumIndex = MA_FILTER_LENGTH - 1;
		}
		/* Multiply impulse response with shifted input sample and add to output */
		ma->out += MA_IMPULSE_RESPONSE[n] * ma->buf[sumIndex];
	}
	/* Return filtered output */
	return ma->out;
}
