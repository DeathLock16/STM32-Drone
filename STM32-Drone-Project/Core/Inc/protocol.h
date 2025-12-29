#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

/* ====== RAMKA ====== */
#define FRAME_START 0xAA
#define FRAME_END   0x55

#define PWM_MOTOR_COUNT   4
#define PWM_PAYLOAD_SIZE  (PWM_MOTOR_COUNT * 2)

/* Na razie max payload = PWM (8 bajtów) */
#define PROTO_MAX_PAYLOAD PWM_PAYLOAD_SIZE
#define PROTO_MAX_FRAME   (6 + PROTO_MAX_PAYLOAD)

/* ====== KIERUNEK ====== */
typedef enum
{
    DIR_STM_TO_PC = 0x00,
    DIR_PC_TO_STM = 0x01
} ProtoDir_t;

typedef enum
{
    ST_OK               = 0x00,

    ST_ERR_BAD_START    = 0x01,
    ST_ERR_BAD_END      = 0x02,
    ST_ERR_BAD_CRC      = 0x03,
    ST_ERR_BAD_DIR      = 0x04,
    ST_ERR_BAD_LEN      = 0x05,
    ST_ERR_UNKNOWN_CMD  = 0x06
} ProtoStatus_t;

/* ====== KOMENDY ====== */
typedef enum
{
    CMD_PING      = 0x01,
    CMD_PONG      = 0x81,

    CMD_PWM_READ  = 0x10,
    CMD_PWM_DATA  = 0x90,

    CMD_PWM_SET   = 0x11,
    CMD_PWM_ACK   = 0x91,

    CMD_STATUS    = 0xE0
} ProtoCmd_t;

typedef struct
{
    uint16_t motor_lb;
    uint16_t motor_lf;
    uint16_t motor_rf;
    uint16_t motor_rb;
} PwmPayload_t;

/* ====== Wynik parsera ====== */
typedef enum
{
    PROTO_INCOMPLETE = 0,
    PROTO_OK         = 1,
    PROTO_ERROR      = 2
} ProtoResult_t;

/* ====== Struktura zparsowanej ramki ======
   Uwaga: to NIE jest layout “on-wire”, tylko wygodna struktura po parsowaniu.
*/
typedef struct
{
    uint8_t start;
    uint8_t dir;
    uint8_t cmd;
    uint8_t len;
    uint8_t data[PROTO_MAX_PAYLOAD];
    uint8_t crc;
    uint8_t end;
} ProtoRxFrame_t;

/* ====== API ====== */
void Protocol_Init(void);
ProtoResult_t Protocol_PushByte(uint8_t byte);
const ProtoRxFrame_t* Protocol_GetFrame(void);
ProtoStatus_t Protocol_GetLastError(void);

/* Budowanie TX do bufora (zwraca długość TX w bajtach) */
uint8_t Protocol_BuildFrame(uint8_t dir, uint8_t cmd,
                            const uint8_t *payload, uint8_t len,
                            uint8_t *out, uint8_t out_max);

uint8_t Protocol_BuildStatusFrame(ProtoStatus_t status,
                                  uint8_t *out, uint8_t out_max);

#endif
