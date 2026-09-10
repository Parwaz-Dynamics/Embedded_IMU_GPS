/*
 * cdc_log.h
 *
 *  Non-blocking, queued logging over USB CDC.
 *
 *  cdcPush() never spins: it copies into a ring buffer and returns in a few
 *  microseconds. cdcPump() hands the oldest contiguous chunk to the USB stack
 *  when the previous transfer has completed. Loop timing is therefore
 *  decoupled from USB throughput.
 *
 *  If the ring is full the whole message is dropped and cdcDropCount is
 *  incremented -- a partial message would corrupt the log.
 */

#ifndef INC_CDC_LOG_H_
#define INC_CDC_LOG_H_

#include <stdint.h>

/* Must be a power of two. 8 kB buffers ~75 ms of traffic at 106 kB/s. */
#define CDC_TX_RING_SIZE 8192

/* Largest single chunk handed to the USB stack per pump. Keeps any one
 * transfer well inside a main-loop period. */
#define CDC_TX_CHUNK_MAX 512

extern volatile uint32_t cdcDropCount;   /* messages dropped, ring full   */
extern volatile uint32_t cdcMaxUsed;     /* high-water mark, for sizing   */

/* Queue a message. Returns 1 if queued, 0 if dropped. Never blocks. */
int  cdcPush(const uint8_t *data, uint16_t len);

/* Service the queue. Call once per main-loop iteration. Never blocks. */
void cdcPump(void);

#endif /* INC_CDC_LOG_H_ */
