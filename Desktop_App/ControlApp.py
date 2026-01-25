import socket
import struct
import threading
import time
import queue
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

HOST = "0.0.0.0"
PORT = 3333

FRAME_START = 0xAA
FRAME_END   = 0x55

DIR_PC_TO_STM = 0x01
DIR_STM_TO_PC = 0x00

CMD_PING     = 0x01
CMD_PONG     = 0x81

CMD_PWM_SET  = 0x11
CMD_PWM_ACK  = 0x91

CMD_IMU_DATA = 0xA0
CMD_STATUS   = 0xE0

CMD_NAV_SET  = 0x14
CMD_NAV_ACK  = 0x94

NAV_STOP      = 0
NAV_FORWARD   = 1
NAV_RIGHT     = 2
NAV_BACK      = 3
NAV_LEFT      = 4
NAV_YAW_RIGHT = 5
NAV_YAW_LEFT  = 6

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
    return crc & 0xFF

def build_frame(cmd, payload=b""):
    length = len(payload)
    crc = calc_crc(DIR_PC_TO_STM, cmd, length, *payload)
    return bytes([FRAME_START, DIR_PC_TO_STM, cmd, length]) + payload + bytes([crc, FRAME_END])

def parse_stream(buf):
    frames = []
    while len(buf) >= 6:
        if buf[0] != FRAME_START:
            buf = buf[1:]
            continue

        ln = buf[3]
        frame_len = 6 + ln
        if frame_len > 128:
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

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Control (TCP server)")
        self.log_q = queue.Queue()

        self.srv = None
        self.conn = None
        self.rx_thread = None
        self.running = False
        self.rx_buf = b""

        self.pwm_var = tk.IntVar(value=0)
        self.last_nav_action = NAV_STOP

        self._build_ui()
        self._ui_poll()

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="both", expand=True)

        left = ttk.Frame(top)
        left.pack(side="left", fill="y")

        right = ttk.Frame(top)
        right.pack(side="right", fill="both", expand=True)

        # Left: connection + slider
        conn_box = ttk.LabelFrame(left, text="Connection", padding=8)
        conn_box.pack(fill="x")

        self.btn_start = ttk.Button(conn_box, text="Start server", command=self.start_server)
        self.btn_start.pack(fill="x")

        self.btn_ping = ttk.Button(conn_box, text="Ping", command=self.send_ping, state="disabled")
        self.btn_ping.pack(fill="x", pady=(6,0))

        self.lbl_state = ttk.Label(conn_box, text="State: DISCONNECTED")
        self.lbl_state.pack(fill="x", pady=(6,0))

        slider_box = ttk.LabelFrame(left, text="Base PWM", padding=8)
        slider_box.pack(fill="y", pady=(10,0))

        self.lbl_pwm = ttk.Label(slider_box, text="0")
        self.lbl_pwm.pack()

        self.slider = ttk.Scale(
            slider_box,
            from_=10000, to=0,
            orient="vertical",
            command=self._on_slider
        )
        self.slider.set(0)
        self.slider.pack(fill="y", expand=True, pady=6)
        self.slider.bind("<ButtonRelease-1>", self._on_slider_release)

        self.btn_set_manual = ttk.Button(slider_box, text="Manual (PWM)", command=self.send_manual_pwm, state="disabled")
        self.btn_set_manual.pack(fill="x", pady=(6,0))

        # Right: controls + logs
        ctrl_box = ttk.LabelFrame(right, text="Controls", padding=8)
        ctrl_box.pack(fill="x")

        grid = ttk.Frame(ctrl_box)
        grid.pack()

        self.btn_yawl = ttk.Button(grid, text="⟲", width=6, command=lambda: self.send_nav(NAV_YAW_LEFT), state="disabled")
        self.btn_fwd  = ttk.Button(grid, text="↑",  width=6, command=lambda: self.send_nav(NAV_FORWARD), state="disabled")
        self.btn_yawr = ttk.Button(grid, text="⟳", width=6, command=lambda: self.send_nav(NAV_YAW_RIGHT), state="disabled")

        self.btn_left = ttk.Button(grid, text="←", width=6, command=lambda: self.send_nav(NAV_LEFT), state="disabled")
        self.btn_stop = ttk.Button(grid, text="STOP", width=6, command=lambda: self.send_nav(NAV_STOP), state="disabled")
        self.btn_right= ttk.Button(grid, text="→", width=6, command=lambda: self.send_nav(NAV_RIGHT), state="disabled")

        self.btn_back = ttk.Button(grid, text="↓", width=6, command=lambda: self.send_nav(NAV_BACK), state="disabled")

        self.btn_yawl.grid(row=0, column=0, padx=4, pady=4)
        self.btn_fwd.grid(row=0, column=1, padx=4, pady=4)
        self.btn_yawr.grid(row=0, column=2, padx=4, pady=4)

        self.btn_left.grid(row=1, column=0, padx=4, pady=4)
        self.btn_stop.grid(row=1, column=1, padx=4, pady=4)
        self.btn_right.grid(row=1, column=2, padx=4, pady=4)

        self.btn_back.grid(row=2, column=1, padx=4, pady=4)

        log_box = ttk.LabelFrame(right, text="Logs", padding=8)
        log_box.pack(fill="both", expand=True, pady=(10,0))

        self.log = ScrolledText(log_box, height=12, wrap="word", state="disabled")
        self.log.pack(fill="both", expand=True)

    def _on_slider(self, _=None):
        val = int(float(self.slider.get()) / 100) * 100
        self.pwm_var.set(val)
        self.lbl_pwm.config(text=str(val))

    def _on_slider_release(self, _event=None):
        if not self.conn:
            return

        base = int(self.pwm_var.get())
        action = int(self.last_nav_action)

        payload = struct.pack("<HB", base, action)
        self.send(build_frame(CMD_NAV_SET, payload))

        name = {
            NAV_STOP:"STOP", NAV_FORWARD:"FWD", NAV_RIGHT:"RIGHT", NAV_BACK:"BACK",
            NAV_LEFT:"LEFT", NAV_YAW_RIGHT:"YAW_R", NAV_YAW_LEFT:"YAW_L"
        }.get(action, str(action))

        self.log_put(f"TX: NAV (slider release) {name} base={base}")

    def log_put(self, msg):
        self.log_q.put(msg)

    def _ui_poll(self):
        try:
            while True:
                msg = self.log_q.get_nowait()
                self.log.configure(state="normal")
                self.log.insert("end", msg + "\n")
                self.log.see("end")
                self.log.configure(state="disabled")
        except queue.Empty:
            pass
        self.root.after(50, self._ui_poll)

    def set_connected(self, connected: bool):
        self.lbl_state.config(text="State: CONNECTED" if connected else "State: DISCONNECTED")
        state = "normal" if connected else "disabled"
        for b in [self.btn_ping, self.btn_set_manual, self.btn_yawl, self.btn_fwd, self.btn_yawr,
                  self.btn_left, self.btn_stop, self.btn_right, self.btn_back]:
            b.config(state=state)

    def start_server(self):
        if self.running:
            return

        self.running = True
        self.btn_start.config(state="disabled")
        self.log_put(f"Starting TCP server on {HOST}:{PORT} ...")

        def worker():
            try:
                self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.srv.bind((HOST, PORT))
                self.srv.listen(1)
                self.log_put("Waiting for ESP connection...")

                self.conn, addr = self.srv.accept()
                self.conn.settimeout(0.05)
                self.log_put(f"Connected from {addr}")

                self.root.after(0, lambda: self.set_connected(True))

                self.rx_loop()

            except Exception as e:
                self.log_put(f"Server error: {e}")
            finally:
                self.running = False
                try:
                    if self.conn:
                        self.conn.close()
                except:
                    pass
                try:
                    if self.srv:
                        self.srv.close()
                except:
                    pass
                self.conn = None
                self.srv = None
                self.root.after(0, lambda: self.set_connected(False))
                self.root.after(0, lambda: self.btn_start.config(state="normal"))
                self.log_put("Server stopped.")

        self.rx_thread = threading.Thread(target=worker, daemon=True)
        self.rx_thread.start()

    def send(self, data: bytes):
        if not self.conn:
            return
        try:
            self.conn.sendall(data)
        except Exception as e:
            self.log_put(f"Send failed: {e}")

    def send_ping(self):
        self.send(build_frame(CMD_PING))
        self.log_put("TX: PING")

    def send_manual_pwm(self):
        val = int(self.pwm_var.get())
        payload = struct.pack("<HHHH", val, val, val, val)
        self.send(build_frame(CMD_PWM_SET, payload))
        self.log_put(f"TX: MANUAL PWM={val}")

    def send_nav(self, action):
        base = int(self.pwm_var.get())
        self.last_nav_action = action
        payload = struct.pack("<HB", base, action)
        self.send(build_frame(CMD_NAV_SET, payload))
        name = {
            NAV_STOP:"STOP", NAV_FORWARD:"FWD", NAV_RIGHT:"RIGHT", NAV_BACK:"BACK",
            NAV_LEFT:"LEFT", NAV_YAW_RIGHT:"YAW_R", NAV_YAW_LEFT:"YAW_L"
        }.get(action, str(action))
        self.log_put(f"TX: NAV {name} base={base}")

    def rx_loop(self):
        while self.running and self.conn:
            try:
                data = self.conn.recv(256)
                if not data:
                    self.log_put("Connection closed by peer.")
                    break
                self.rx_buf += data
            except socket.timeout:
                pass
            except Exception as e:
                self.log_put(f"RX error: {e}")
                break

            frames, self.rx_buf = parse_stream(self.rx_buf)
            for cmd, payload in frames:
                if cmd == CMD_PONG:
                    self.log_put("RX: PONG ✓")
                elif cmd == CMD_PWM_ACK:
                    self.log_put("RX: PWM ACK ✓")
                elif cmd == CMD_NAV_ACK:
                    if len(payload) == 3:
                        base, act = struct.unpack("<HB", payload)
                        self.log_put(f"RX: NAV ACK ✓ base={base} act={act}")
                    else:
                        self.log_put("RX: NAV ACK ✓")
                elif cmd == CMD_STATUS and len(payload) == 1:
                    err = payload[0]
                    self.log_put("RX STATUS: " + STATUS_STR.get(err, f"0x{err:02X}"))
                elif cmd == CMD_IMU_DATA and len(payload) == 6:
                    roll, pitch, yaw = struct.unpack("<hhh", payload)
                    self.log_put(f"RX IMU: R={roll/100:.1f} P={pitch/100:.1f} Y={yaw/100:.1f}")

            time.sleep(0.01)

def main():
    root = tk.Tk()
    app = App(root)
    root.mainloop()

if __name__ == "__main__":
    main()
