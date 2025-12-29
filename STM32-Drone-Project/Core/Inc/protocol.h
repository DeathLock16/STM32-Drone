#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

/* ====== RAMKA ====== */

#define FRAME_START 0xAA
#define FRAME_END   0x55

/* ====== KIERUNEK ====== */
typedef enum
{
    DIR_STM_TO_PC = 0x00,
    DIR_PC_TO_STM = 0x01
} ProtoDir_t;

/* ====== KOMENDY ====== */
typedef enum
{
    CMD_PING = 0x01,
    CMD_PONG = 0x81
} ProtoCmd_t;

/* ====== STRUKTURA RAMKI ====== */
typedef struct
{
    uint8_t start;
    uint8_t dir;
    uint8_t cmd;
    uint8_t len;
    uint8_t crc;
    uint8_t end;
} ProtoFrame_t;

/* ====== API ====== */

/* inicjalizacja parsera */
void Protocol_Init(void);

/* podanie kolejnego bajtu z UART
 * zwraca 1 jeśli ramka kompletna
 */
int Protocol_PushByte(uint8_t byte);

/* pobranie ostatniej poprawnej ramki */
const ProtoFrame_t* Protocol_GetFrame(void);

/* budowa odpowiedzi (PING -> PONG itd.) */
int Protocol_BuildResponse(const ProtoFrame_t *rx,
                           ProtoFrame_t *tx);

#endif
