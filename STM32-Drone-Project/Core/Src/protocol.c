#include "protocol.h"
#include <string.h>

/* ====== Parser ====== */
static uint8_t rxBuf[PROTO_MAX_FRAME];
static uint8_t rxIndex = 0;
static uint8_t rxExpected = 0;

static ProtoRxFrame_t rxFrame;
static ProtoStatus_t last_error = ST_OK;

/* CRC = DIR ^ CMD ^ LEN ^ PAYLOAD... */
static uint8_t CalcCrc(uint8_t dir, uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint8_t crc = (uint8_t)(dir ^ cmd ^ len);
    for (uint8_t i = 0; i < len; i++)
        crc ^= payload[i];
    return crc;
}

void Protocol_Init(void)
{
    rxIndex = 0;
    rxExpected = 0;
    last_error = ST_OK;
    memset(&rxFrame, 0, sizeof(rxFrame));
}

ProtoResult_t Protocol_PushByte(uint8_t byte)
{
    if (rxIndex == 0)
    {
        if (byte != FRAME_START)
            return PROTO_INCOMPLETE;

        rxBuf[0] = byte;
        rxIndex = 1;
        rxExpected = 0;
        return PROTO_INCOMPLETE;
    }

    if (rxIndex >= sizeof(rxBuf))
    {
        rxIndex = 0;
        rxExpected = 0;
        last_error = ST_ERR_BAD_LEN;
        return PROTO_ERROR;
    }

    rxBuf[rxIndex++] = byte;

    if (rxIndex == 4)
    {
        uint8_t len = rxBuf[3];
        if (len > PROTO_MAX_PAYLOAD)
        {
            rxIndex = 0;
            rxExpected = 0;
            last_error = ST_ERR_BAD_LEN;
            return PROTO_ERROR;
        }
        rxExpected = (uint8_t)(6 + len);
    }

    if (rxExpected && rxIndex >= rxExpected)
    {
        uint8_t len = rxBuf[3];
        uint8_t crc_rx = rxBuf[4 + len];
        uint8_t end_rx = rxBuf[5 + len];

        rxIndex = 0;
        rxExpected = 0;

        if (rxBuf[0] != FRAME_START)
        {
            last_error = ST_ERR_BAD_START;
            return PROTO_ERROR;
        }

        if (end_rx != FRAME_END)
        {
            last_error = ST_ERR_BAD_END;
            return PROTO_ERROR;
        }

        if (rxBuf[1] != DIR_PC_TO_STM)
        {
            last_error = ST_ERR_BAD_DIR;
            return PROTO_ERROR;
        }

        uint8_t crc_calc = CalcCrc(rxBuf[1], rxBuf[2], len, &rxBuf[4]);
        if (crc_calc != crc_rx)
        {
            last_error = ST_ERR_BAD_CRC;
            return PROTO_ERROR;
        }

        rxFrame.start = rxBuf[0];
        rxFrame.dir   = rxBuf[1];
        rxFrame.cmd   = rxBuf[2];
        rxFrame.len   = len;

        memset(rxFrame.data, 0, sizeof(rxFrame.data));
        if (len > 0)
            memcpy(rxFrame.data, &rxBuf[4], len);

        rxFrame.crc = crc_rx;
        rxFrame.end = end_rx;

        last_error = ST_OK;
        return PROTO_OK;
    }

    return PROTO_INCOMPLETE;
}

const ProtoRxFrame_t* Protocol_GetFrame(void)
{
    return &rxFrame;
}

ProtoStatus_t Protocol_GetLastError(void)
{
    return last_error;
}

uint8_t Protocol_BuildFrame(uint8_t dir, uint8_t cmd,
                            const uint8_t *payload, uint8_t len,
                            uint8_t *out, uint8_t out_max)
{
    if (!out) return 0;
    if (len > PROTO_MAX_PAYLOAD) return 0;

    uint8_t total = (uint8_t)(6 + len);
    if (out_max < total) return 0;

    out[0] = FRAME_START;
    out[1] = dir;
    out[2] = cmd;
    out[3] = len;

    if (len > 0 && payload)
        memcpy(&out[4], payload, len);
    else if (len > 0)
        memset(&out[4], 0, len);

    out[4 + len] = CalcCrc(dir, cmd, len, &out[4]);
    out[5 + len] = FRAME_END;

    return total;
}

uint8_t Protocol_BuildStatusFrame(ProtoStatus_t status,
                                  uint8_t *out, uint8_t out_max)
{
    uint8_t payload = (uint8_t)status;
    return Protocol_BuildFrame(DIR_STM_TO_PC, CMD_STATUS, &payload, 1, out, out_max);
}
