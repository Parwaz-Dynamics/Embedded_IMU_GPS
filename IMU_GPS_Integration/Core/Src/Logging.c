/*
 * cdc_log.c
 *
 *  See cdc_log.h.
 *
 *  Note on zero-copy: CDC_Transmit_FS() does not copy the caller's buffer,
 *  it points the USB stack at it. We therefore hand it a pointer into the
 *  ring and only advance the tail once the transfer has completed, so the
 *  bytes stay valid for the whole transfer. This is exactly what the old
 *  code got wrong by reusing msgOut while a transfer was in flight.
 */

#include "logging.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t  ring[CDC_TX_RING_SIZE];
static volatile uint32_t head = 0;      /* next write position          */
static volatile uint32_t tail = 0;      /* oldest unsent byte           */
static uint16_t pendingLen = 0;         /* bytes handed to USB, in flight */

volatile uint32_t cdcDropCount = 0;
volatile uint32_t cdcMaxUsed   = 0;

static inline uint32_t ringUsed(void)
{
	return (head - tail) & (CDC_TX_RING_SIZE - 1);
}

static inline uint32_t ringFree(void)
{
	/* one byte reserved so head==tail always means empty */
	return CDC_TX_RING_SIZE - 1 - ringUsed();
}

/* Is the CDC IN endpoint still busy with the previous transfer? */
static int cdcTxBusy(void)
{
	USBD_CDC_HandleTypeDef *hcdc =
			(USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pClassData;

	if (hcdc == NULL)
		return 1;                      /* not enumerated yet */

	return (hcdc->TxState != 0);
}

int cdcPush(const uint8_t *data, uint16_t len)
{
	if (len == 0)
		return 1;

	if (len > ringFree()) {
		cdcDropCount++;                /* drop whole message, never partial */
		return 0;
	}

	uint32_t h = head;
	uint32_t first = CDC_TX_RING_SIZE - h;
	if (first > len)
		first = len;

	for (uint32_t i = 0; i < first; i++)
		ring[h + i] = data[i];
	for (uint32_t i = first; i < len; i++)
		ring[i - first] = data[i];

	head = (h + len) & (CDC_TX_RING_SIZE - 1);

	uint32_t used = ringUsed();
	if (used > cdcMaxUsed)
		cdcMaxUsed = used;

	return 1;
}

void cdcPump(void)
{
	/* 1. Retire the in-flight transfer if it has finished. */
	if (pendingLen != 0) {
		if (cdcTxBusy())
			return;                    /* still sending, come back later */

		tail = (tail + pendingLen) & (CDC_TX_RING_SIZE - 1);
		pendingLen = 0;
	}

	/* 2. Nothing queued, or endpoint not ready. */
	uint32_t used = ringUsed();
	if (used == 0)
		return;
	if (cdcTxBusy())
		return;

	/* 3. Send the largest contiguous chunk, capped. */
	uint32_t t = tail;
	uint32_t contiguous = CDC_TX_RING_SIZE - t;
	uint32_t n = (used < contiguous) ? used : contiguous;
	if (n > CDC_TX_CHUNK_MAX)
		n = CDC_TX_CHUNK_MAX;

	if (CDC_Transmit_FS(&ring[t], (uint16_t) n) == USBD_OK)
		pendingLen = (uint16_t) n;     /* tail advances when it completes */
}
