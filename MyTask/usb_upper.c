#include "usb_upper.h"
#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "config.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

UsbUpper_Data_t   g_upper_data;
SemaphoreHandle_t g_upper_sem;
volatile uint32_t g_upper_tx_drop   = 0u;
volatile uint32_t g_upper_frame_bad = 0u;

/* A USB CDC OUT packet can carry up to 64 bytes, so the staging buffer keeps
 * a multiple of the frame length. On overflow the oldest whole frame is moved
 * out instead of resetting the index, which keeps the frame boundary intact. */
#define RING_BUF_SIZE   (HOST_FRAME_LEN * 8u)

static uint8_t  ring_buf[RING_BUF_SIZE];
static uint16_t ring_idx = 0;

static void RingBuf_Push(const uint8_t *src, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++) {
        if (ring_idx >= sizeof(ring_buf)) {
            if (ring_idx >= HOST_FRAME_LEN) {
                memmove(ring_buf, ring_buf + HOST_FRAME_LEN,
                        (size_t)(ring_idx - HOST_FRAME_LEN));
                ring_idx = (uint16_t)(ring_idx - HOST_FRAME_LEN);
            } else {
                ring_idx = 0u;
            }
        }
        ring_buf[ring_idx++] = src[i];
    }
}

/* The frame carries no CRC, so a range check is the only guard against a
 * mis-framed packet turning into a bogus velocity command. NaN is detected
 * with self-comparison to stay independent of the isfinite macro. */
static uint8_t Upper_FrameValid(float vx, float vy, float omega)
{
    if ((vx != vx) || (vy != vy) || (omega != omega)) {
        return 0u;
    }
    if ((fabsf(vx) > MAX_ROBOT_VEL) || (fabsf(vy) > MAX_ROBOT_VEL)) {
        return 0u;
    }
    if (fabsf(omega) > MAX_ROBOT_OMEGA) {
        return 0u;
    }
    return 1u;
}

static void RingBuf_Align(void)
{
    uint16_t head_pos = ring_idx;

    for (uint16_t i = 0; i < ring_idx; i++) {
        if (ring_buf[i] == HOST_FRAME_HEAD) {
            head_pos = i;
            break;
        }
    }

    if (head_pos > 0) {
        memmove(ring_buf, ring_buf + head_pos, ring_idx - head_pos);
        ring_idx -= head_pos;
    }
}

void UsbUpper_Send(float vx, float vy, float omega)
{
    static uint8_t tx_frame[HOST_FRAME_LEN];

    if (USB_CDC_IsConfigured() == 0u) {
        return;
    }

    tx_frame[0] = HOST_FRAME_HEAD;
    memcpy(&tx_frame[1], &vx, 4);
    memcpy(&tx_frame[5], &vy, 4);
    memcpy(&tx_frame[9], &omega, 4);
    tx_frame[HOST_FRAME_LEN - 1] = HOST_FRAME_TAIL;

    /* The IN endpoint may still be busy with the previous transfer; retry
     * briefly instead of dropping the echo silently. */
    for (uint8_t n = 0u; n < 2u; n++) {
        if (CDC_Transmit_FS(tx_frame, HOST_FRAME_LEN) == USBD_OK) {
            return;
        }
        vTaskDelay(1);
    }

    g_upper_tx_drop++;
}

void UsbUpper_RecvCallback(uint8_t *src, uint16_t size)
{
    RingBuf_Push(src, size);

    while (ring_idx >= HOST_FRAME_LEN) {

        RingBuf_Align();

        if (ring_idx < HOST_FRAME_LEN) break;

        if (ring_buf[0] != HOST_FRAME_HEAD ||
            ring_buf[HOST_FRAME_LEN - 1] != HOST_FRAME_TAIL) {
            ring_buf[0] = 0x00;
            continue;
        }

        float vx, vy, omega;
        memcpy(&vx,    ring_buf + 1, 4);
        memcpy(&vy,    ring_buf + 5, 4);
        memcpy(&omega, ring_buf + 9, 4);

        /* Consume the frame before validating it, otherwise a frame that is
         * rejected here would keep the loop spinning on the same bytes. */
        memmove(ring_buf, ring_buf + HOST_FRAME_LEN,
                (size_t)(ring_idx - HOST_FRAME_LEN));
        ring_idx = (uint16_t)(ring_idx - HOST_FRAME_LEN);

        if (Upper_FrameValid(vx, vy, omega) != 0u) {
            g_upper_data.vx    = vx;
            g_upper_data.vy    = vy;
            g_upper_data.omega = omega;
            if (g_upper_sem != NULL) {
                xSemaphoreGive(g_upper_sem);
            }
        } else {
            g_upper_frame_bad++;
        }

        UsbUpper_Send(vx, vy, omega);
    }
}

void UsbUpper_Init(void)
{
    if (g_upper_sem == NULL) {
        g_upper_sem = xSemaphoreCreateBinary();
    }
    if (g_upper_sem == NULL) {
        return;                     /* out of heap: retry on the next call */
    }
    if (USB_CDC_Init(UsbUpper_RecvCallback, NULL, NULL) != pdPASS) {
        vSemaphoreDelete(g_upper_sem);
        g_upper_sem = NULL;         /* do not fake an initialized state */
    }
}
