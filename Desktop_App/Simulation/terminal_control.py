import socket
import struct
import time

HOST = "0.0.0.0"
PORT = 3333

FRAME_START = 0xAA
FRAME_END   = 0x55

DIR_PC_TO_STM = 0x01
DIR_STM_TO_PC = 0x00

CMD_PING     = 0x01
CMD_PONG     = 0x81

CMD_PWM_READ = 0x10
CMD_PWM_DATA = 0x90

CMD_PWM_SET  = 0x11
CMD_PWM_ACK  = 0x91

CMD_IMU_READ = 0x20
CMD_IMU_DATA = 0xA0

CMD_STATUS   = 0xE0

STATUS_STR = {
    0x01: "BAD_START",
    0x02: "BAD_END",
    0x03: "BAD_CRC",
    0x04: "BAD_DIR",
    0x05: "BAD_LEN",
    0x06: "UNKNOWN_CMD",
    0x07: "IMU_NOT_READY"
}


def calc_crc(*args):
    crc = 0
    for a in args:
        crc ^= a
    return crc


def build_frame(cmd, payload=b""):
    length = len(payload)
    crc = calc_crc(DIR_PC_TO_STM, cmd, length, *payload)
    return bytes([
        FRAME_START,
        DIR_PC_TO_STM,
        cmd,
        length,
        *payload,
        crc,
        FRAME_END
    ])


def parse_stream(buf):
    frames = []

    while len(buf) >= 6:
        if buf[0] != FRAME_START:
            buf = buf[1:]
            continue

        length = buf[3]
        frame_len = 6 + length

        if frame_len > 64:
            buf = buf[1:]
            continue

        if len(buf) < frame_len:
            break

        frame = buf[:frame_len]
        buf = buf[frame_len:]

        start, dir_, cmd, ln = frame[:4]
        payload = frame[4:4+ln]
        crc = frame[4+ln]
        end = frame[5+ln]

        if end != FRAME_END:
            continue

        if calc_crc(dir_, cmd, ln, *payload) != crc:
            continue

        frames.append((cmd, payload))

    return frames, buf


print("Starting TCP server...")
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
srv.bind((HOST, PORT))
srv.listen(1)

print("Waiting for ESP...")
conn, addr = srv.accept()
print("Connected from:", addr)

conn.settimeout(0.1)
rx_buf = b""

print("""
1 - PING
2 - PWM SET 5%
3 - PWM SET 100%
4 - PWM SET 0%
5 - PWM SET (custom)
6 - PWM READ
7 - IMU READ
q - quit
""")

value = 0

while True:
    cmd = input("> ").strip()

    if cmd == "q":
        break

    elif cmd == "1":
        conn.sendall(build_frame(CMD_PING))
        print("TX: PING")

    elif cmd == "2":
        pwm = struct.pack("<HHHH", 500, 500, 500, 500)
        conn.sendall(build_frame(CMD_PWM_SET, pwm))
        print("TX: PWM SET 5%")

    elif cmd == "3":
        pwm = struct.pack("<HHHH", 10000, 10000, 10000, 10000)
        conn.sendall(build_frame(CMD_PWM_SET, pwm))
        print("TX: PWM SET 100%")

    elif cmd == "4":
        while value <= 10000:
            print(f"TX: PWM SET {value}")
            pwm = struct.pack("<HHHH", value, value, value, value)
            conn.sendall(build_frame(CMD_PWM_SET, pwm))
            value += 100
            time.sleep(0.1)

    elif cmd == "5":
        s = input("Podaj wartość PWM (0..10000): ").strip()
        try:
            val = int(s)
        except ValueError:
            print("Nieprawidłowa liczba")
            continue

        val = max(0, min(10000, val))
        pwm = struct.pack("<HHHH", val, val, val, val)
        conn.sendall(build_frame(CMD_PWM_SET, pwm))
        print(f"TX: PWM SET custom = {val}")

    elif cmd == "6":
        conn.sendall(build_frame(CMD_PWM_READ))
        print("TX: PWM READ")

    elif cmd == "7":
        conn.sendall(build_frame(CMD_IMU_READ))
        print("TX: IMU READ")

    else:
        print("Unknown command")
        continue

    t0 = time.time()
    while time.time() - t0 < 1.0:
        try:
            rx_buf += conn.recv(64)
        except socket.timeout:
            pass

        frames, rx_buf = parse_stream(rx_buf)

        for fcmd, payload in frames:
            if fcmd == CMD_PONG:
                print("RX: PONG ✓")

            elif fcmd == CMD_PWM_ACK:
                print("RX: PWM ACK ✓")

            elif fcmd == CMD_PWM_DATA and len(payload) == 8:
                lb, lf, rf, rb = struct.unpack("<HHHH", payload)
                print(f"RX PWM: LB={lb} LF={lf} RF={rf} RB={rb}")

            elif fcmd == CMD_STATUS and len(payload) == 1:
                err = payload[0]
                print("RX STATUS:", STATUS_STR.get(err, f"0x{err:02X}"))

            elif fcmd == CMD_IMU_DATA and len(payload) == 6:
                roll, pitch, yaw = struct.unpack("<hhh", payload)
                print(
                    f"RX IMU: "
                    f"ROLL={roll/100:.2f}° "
                    f"PITCH={pitch/100:.2f}° "
                    f"YAW={yaw/100:.2f}°"
                )

        time.sleep(0.01)

print("Bye")
conn.close()
