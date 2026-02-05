#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

/* ====== RAMKA ====== */
#define FRAME_START 0xAA
#define FRAME_END   0x55

#define PWM_MOTOR_COUNT   4
#define PWM_PAYLOAD_SIZE  (PWM_MOTOR_COUNT * 2)

/* IMU payload = 3x int16 = 6 bajtów */
typedef struct __attribute__((packed))
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} ImuPayload_t;

#define IMU_PAYLOAD_SIZE ((uint8_t)sizeof(ImuPayload_t))

/* Stab payload */
typedef struct
{
    uint16_t base_pwm;
} StabPayload_t;

#define STAB_PAYLOAD_SIZE ((uint8_t)sizeof(StabPayload_t))

/* ustaw max payload na zapas (żeby kolejne komendy nie wymagały grzebania) */
#define PROTO_MAX_PAYLOAD 32
#define PROTO_MAX_FRAME   (6 + PROTO_MAX_PAYLOAD)

/* ====== KIERUNEK ====== */
typedef enum
{
    DIR_STM_TO_PC = 0x00,
    DIR_PC_TO_STM = 0x01
} ProtoDir_t;

typedef enum
{
    ST_OK                 = 0x00,

    ST_ERR_BAD_START      = 0x01,
    ST_ERR_BAD_END        = 0x02,
    ST_ERR_BAD_CRC        = 0x03,
    ST_ERR_BAD_DIR        = 0x04,
    ST_ERR_BAD_LEN        = 0x05,
    ST_ERR_UNKNOWN_CMD    = 0x06,

    ST_ERR_IMU_NOT_READY  = 0x07
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

    CMD_STAB_SET  = 0x12,
    CMD_STAB_ACK  = 0x92,

    CMD_STAB_OFF  = 0x13,
    CMD_STAB_OFF_ACK = 0x93,

    CMD_NAV_SET   = 0x14,
    CMD_NAV_ACK   = 0x94,

    CMD_STATUS    = 0xE0,

    CMD_IMU_READ  = 0x20,
    CMD_IMU_DATA  = 0xA0,

	CMD_TELEM_READ = 0X21,
	CMD_TELEM_DATA = 0XA1,

    CMD_TELEM_LVL_READ = 0x22,
    CMD_TELEM_LVL_DATA = 0xA2
} ProtoCmd_t;

typedef struct __attribute__((packed))
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

/* ====== Struktura zparsowanej ramki ====== */
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

typedef enum
{
    NAV_STOP = 0,
    NAV_FORWARD = 1,
    NAV_RIGHT = 2,
    NAV_BACK = 3,
    NAV_LEFT = 4,
    NAV_YAW_RIGHT = 5,
    NAV_YAW_LEFT = 6
} NavAction_t;

typedef struct __attribute__((packed))
{
    uint16_t base_pwm;
    uint8_t  action;
} NavPayload_t;

#define NAV_PAYLOAD_SIZE 3u

typedef struct __attribute__((packed))
{
    ImuPayload_t imu;   // 6B
    PwmPayload_t pwm;   // 8B
} TelemetryPayload_t;

#define TELEM_PAYLOAD_SIZE ((uint8_t)sizeof(TelemetryPayload_t)) // 14

typedef struct __attribute__((packed))
{
    int16_t roll_raw;
    int16_t pitch_raw;
    int16_t yaw_raw;

    int16_t roll_lvl;
    int16_t pitch_lvl;

    int16_t roll_ctrl;
    int16_t pitch_ctrl;

    PwmPayload_t pwm;
} TelemetryLvlPayload_t;

#define TELEM_LVL_PAYLOAD_SIZE ((uint8_t)sizeof(TelemetryLvlPayload_t))


/* ====== API ====== */
void Protocol_Init(void);
ProtoResult_t Protocol_PushByte(uint8_t byte);
const ProtoRxFrame_t* Protocol_GetFrame(void);
ProtoStatus_t Protocol_GetLastError(void);

uint8_t Protocol_BuildFrame(uint8_t dir, uint8_t cmd,
                            const uint8_t *payload, uint8_t len,
                            uint8_t *out, uint8_t out_max);

uint8_t Protocol_BuildStatusFrame(ProtoStatus_t status,
                                  uint8_t *out, uint8_t out_max);

#endif
