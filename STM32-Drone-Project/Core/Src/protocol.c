#include "protocol.h"

/* ====== PARSER WEWNĘTRZNY ====== */

static ProtoFrame_t rxFrame;
static uint8_t rxIndex;

/* ====== CRC ======
 * Na razie prosty XOR:
 * DIR ^ CMD ^ LEN
 */
static uint8_t Protocol_CalcCRC(const ProtoFrame_t *f)
{
    return f->dir ^ f->cmd ^ f->len;
}

/* ====== API ====== */

void Protocol_Init(void)
{
    rxIndex = 0;
}

int Protocol_PushByte(uint8_t byte)
{
    ((uint8_t*)&rxFrame)[rxIndex++] = byte;

    if (rxIndex < sizeof(ProtoFrame_t))
        return 0;

    rxIndex = 0;

    /* walidacja ramki */
    if (rxFrame.start != FRAME_START)
        return 0;

    if (rxFrame.end != FRAME_END)
        return 0;

    if (Protocol_CalcCRC(&rxFrame) != rxFrame.crc)
        return 0;

    return 1; /* ramka OK */
}

const ProtoFrame_t* Protocol_GetFrame(void)
{
    return &rxFrame;
}

int Protocol_BuildResponse(const ProtoFrame_t *rx,
                           ProtoFrame_t *tx)
{
    if (!rx || !tx)
        return 0;

    /* domyślne pola */
    tx->start = FRAME_START;
    tx->dir   = DIR_STM_TO_PC;
    tx->len   = 0;
    tx->end   = FRAME_END;

    switch (rx->cmd)
    {
        case CMD_PING:
            tx->cmd = CMD_PONG;
            break;

        default:
            return 0; /* nieznana komenda */
    }

    tx->crc = Protocol_CalcCRC(tx);
    return 1;
}
