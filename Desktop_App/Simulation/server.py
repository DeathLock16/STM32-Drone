import socket
import struct
import time

HOST = "0.0.0.0"
PORT = 3333

FRAME_START = 0xAA
FRAME_END   = 0x55

DIR_PC_TO_STM = 0x01
DIR_STM_TO_PC = 0x00

CMD_PING = 0x01
CMD_PONG = 0x81


def calc_crc(dir_, cmd, length):
    return dir_ ^ cmd ^ length


def build_frame(dir_, cmd, payload=b""):
    length = len(payload)
    crc = calc_crc(dir_, cmd, length)
    return struct.pack(
        "BBBBBB",
        FRAME_START,
        dir_,
        cmd,
        length,
        crc,
        FRAME_END
    )


def parse_frame(buf: bytes):
    if len(buf) != 6:
        return None

    start, dir_, cmd, length, crc, end = struct.unpack("BBBBBB", buf)

    if start != FRAME_START or end != FRAME_END:
        return None

    if crc != calc_crc(dir_, cmd, length):
        return None

    return {
        "dir": dir_,
        "cmd": cmd,
        "len": length
    }


print("Starting TCP server...")
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
srv.bind((HOST, PORT))
srv.listen(1)

print("Waiting for ESP...")
conn, addr = srv.accept()
print("Connected from:", addr)

time.sleep(0.5)

while True:
    ping = build_frame(DIR_PC_TO_STM, CMD_PING)
    print("TX PING:", ping.hex(" "))
    conn.sendall(ping)

    buf = b""
    t0 = time.time()

    while time.time() - t0 < 1.0:
        buf += conn.recv(32)

        while len(buf) >= 6:
            if buf[0] != FRAME_START:
                buf = buf[1:]
                continue

            frame = buf[:6]
            buf = buf[6:]

            parsed = parse_frame(frame)
            if parsed:
                if parsed["cmd"] == CMD_PONG:
                    print("RX PONG ✓")
                else:
                    print("RX OTHER:", frame.hex(" "))
                break

        else:
            continue
        break

    time.sleep(0.05)
