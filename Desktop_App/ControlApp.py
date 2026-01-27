import socket
import struct
import threading
import time
import queue
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText
from tkinter import filedialog, messagebox
from collections import deque
import csv

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

HOST = "192.168.8.100"
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

CMD_TELEM_READ = 0x21
CMD_TELEM_DATA = 0xA1

CMD_STATUS   = 0xE0

CMD_NAV_SET  = 0x14
CMD_NAV_ACK  = 0x94

CMD_STAB_OFF     = 0x13
CMD_STAB_OFF_ACK = 0x93

NAV_STOP      = 0
NAV_FORWARD   = 1
NAV_RIGHT     = 2
NAV_BACK      = 3
NAV_LEFT      = 4
NAV_YAW_RIGHT = 5
NAV_YAW_LEFT  = 6

NAV_STR = {
    NAV_STOP: "STOP",
    NAV_FORWARD: "FORWARD",
    NAV_RIGHT: "RIGHT",
    NAV_BACK: "BACK",
    NAV_LEFT: "LEFT",
    NAV_YAW_RIGHT: "YAW_RIGHT",
    NAV_YAW_LEFT: "YAW_LEFT"
}

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

        _, dir_, cmd, ln = frame[:4]
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
        self.running = False
        self.rx_buf = b""

        self.rx_thread = None
        self.tx_thread = None

        self.tx_q = queue.Queue(maxsize=2000)
        self.tx_stop = threading.Event()

        self.pwm_var = tk.IntVar(value=0)

        self.desired_action = NAV_STOP
        self.current_base_sent = 0

        self.takeoff_on = False
        self.takeoff_stop = threading.Event()
        self.takeoff_thread = None

        self.pwm_poll_on = True
        self.pwm_poll_interval_s = 0.05  # 20 Hz
        self.pwm_poll_thread = None
        self.pwm_poll_stop = threading.Event()

        self.t0 = time.monotonic()
        self.hist_len = 300
        self.t_hist = deque(maxlen=self.hist_len)
        self.pwm_hist = {
            "LF": deque(maxlen=self.hist_len),
            "RF": deque(maxlen=self.hist_len),
            "LB": deque(maxlen=self.hist_len),
            "RB": deque(maxlen=self.hist_len),
        }
        self.last_pwm = {"LF": 0, "RF": 0, "LB": 0, "RB": 0}

        self.roll_var = tk.StringVar(value="ROLL: --.-°")
        self.pitch_var = tk.StringVar(value="PITCH: --.-°")
        self.yaw_var = tk.StringVar(value="YAW: --.-°")

        # ---- DATA LOGGING ----
        self.data_lock = threading.Lock()
        self.data_rows = []
        self.last_imu = {"roll": None, "pitch": None, "yaw": None}

        self._build_ui()
        self._ui_poll()
        self._plot_update()

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="both", expand=True)

        left = ttk.Frame(top)
        left.pack(side="left", fill="y")

        right = ttk.Frame(top)
        right.pack(side="right", fill="both", expand=True)

        conn_box = ttk.LabelFrame(left, text="Connection", padding=8)
        conn_box.pack(fill="x")

        self.btn_start = ttk.Button(conn_box, text="Start server", command=self.start_server)
        self.btn_start.pack(fill="x")

        self.btn_ping = ttk.Button(conn_box, text="Ping", command=self.send_ping, state="disabled")
        self.btn_ping.pack(fill="x", pady=(6,0))

        self.lbl_state = ttk.Label(conn_box, text="State: DISCONNECTED")
        self.lbl_state.pack(fill="x", pady=(6,0))

        slider_box = ttk.LabelFrame(left, text="Base PWM", padding=8)
        slider_box.pack(fill="y", pady=(10,0), expand=True)

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

        self.btn_takeoff = ttk.Button(slider_box, text="TAKEOFF (smooth)", command=self.toggle_takeoff, state="disabled")
        self.btn_takeoff.pack(fill="x", pady=(6,0))

        self.btn_export = ttk.Button(slider_box, text="Export CSV", command=self.export_csv, state="disabled")
        self.btn_export.pack(fill="x", pady=(12,0))

        top_right = ttk.Frame(right)
        top_right.pack(fill="both", expand=True)

        controls = ttk.Frame(top_right)
        controls.pack(side="left", anchor="n")

        self.btn_yawl = ttk.Button(controls, text="⟲", width=6, command=lambda: self.send_nav(NAV_YAW_LEFT), state="disabled")
        self.btn_fwd  = ttk.Button(controls, text="↑",  width=6, command=lambda: self.send_nav(NAV_FORWARD), state="disabled")
        self.btn_yawr = ttk.Button(controls, text="⟳", width=6, command=lambda: self.send_nav(NAV_YAW_RIGHT), state="disabled")

        self.btn_left = ttk.Button(controls, text="←", width=6, command=lambda: self.send_nav(NAV_LEFT), state="disabled")
        self.btn_stop = ttk.Button(controls, text="STOP", width=6, command=lambda: self.send_nav(NAV_STOP), state="disabled")
        self.btn_right= ttk.Button(controls, text="→", width=6, command=lambda: self.send_nav(NAV_RIGHT), state="disabled")

        self.btn_back = ttk.Button(controls, text="↓", width=6, command=lambda: self.send_nav(NAV_BACK), state="disabled")

        self.btn_yawl.grid(row=0, column=0, padx=4, pady=4)
        self.btn_fwd.grid(row=0, column=1, padx=4, pady=4)
        self.btn_yawr.grid(row=0, column=2, padx=4, pady=4)

        self.btn_left.grid(row=1, column=0, padx=4, pady=4)
        self.btn_stop.grid(row=1, column=1, padx=4, pady=4)
        self.btn_right.grid(row=1, column=2, padx=4, pady=4)

        self.btn_back.grid(row=2, column=1, padx=4, pady=4)

        imu_box = ttk.Frame(controls)
        imu_box.grid(row=3, column=0, columnspan=3, sticky="w", padx=4, pady=(8, 0))

        ttk.Label(imu_box, textvariable=self.roll_var).pack(anchor="w")
        ttk.Label(imu_box, textvariable=self.pitch_var).pack(anchor="w")
        ttk.Label(imu_box, textvariable=self.yaw_var).pack(anchor="w")

        plot_box = ttk.LabelFrame(top_right, text="PWM (live)", padding=6)
        plot_box.pack(side="left", fill="both", expand=True, padx=(10,0))

        fig = Figure(figsize=(6.0, 2.8), dpi=100)
        self.ax = fig.add_subplot(111)
        self.ax.set_xlabel("time (s)")
        self.ax.set_ylabel("PWM")

        self.line_lf, = self.ax.plot([], [], label="LF")
        self.line_rf, = self.ax.plot([], [], label="RF")
        self.line_lb, = self.ax.plot([], [], label="LB")
        self.line_rb, = self.ax.plot([], [], label="RB")

        self.ax.legend(loc="upper right")

        self.canvas = FigureCanvasTkAgg(fig, master=plot_box)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

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
        if self.takeoff_on:
            return

        base = int(self.pwm_var.get())
        payload = struct.pack("<HB", base, int(self.desired_action))
        self.send_frame(CMD_NAV_SET, payload)

        self.log_put(f"TX: NAV (slider release) act={self.desired_action} base={base}")

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

    def _plot_update(self):
        try:
            if len(self.t_hist) >= 2:
                t = list(self.t_hist)
                t0 = t[-1]
                x = [ti - t0 for ti in t]

                y_lf = list(self.pwm_hist["LF"])
                y_rf = list(self.pwm_hist["RF"])
                y_lb = list(self.pwm_hist["LB"])
                y_rb = list(self.pwm_hist["RB"])

                self.line_lf.set_data(x, y_lf)
                self.line_rf.set_data(x, y_rf)
                self.line_lb.set_data(x, y_lb)
                self.line_rb.set_data(x, y_rb)

                self.ax.relim()
                self.ax.autoscale_view()

                self.canvas.draw_idle()
        except Exception:
            pass

        self.root.after(100, self._plot_update)

    def set_connected(self, connected: bool):
        self.lbl_state.config(text="State: CONNECTED" if connected else "State: DISCONNECTED")
        state = "normal" if connected else "disabled"
        for b in [self.btn_ping, self.btn_set_manual, self.btn_takeoff,
                  self.btn_yawl, self.btn_fwd, self.btn_yawr,
                  self.btn_left, self.btn_stop, self.btn_right, self.btn_back,
                  self.btn_export]:
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

                self.tx_stop.clear()
                self.tx_thread = threading.Thread(target=self.tx_loop, daemon=True)
                self.tx_thread.start()

                self.pwm_poll_stop.clear()
                self.pwm_poll_thread = threading.Thread(target=self.pwm_poll_loop, daemon=True)
                self.pwm_poll_thread.start()

                self.root.after(0, lambda: self.set_connected(True))

                self.rx_loop()

            except Exception as e:
                self.log_put(f"Server error: {e}")
            finally:
                self.running = False

                self.takeoff_stop.set()
                self.takeoff_on = False

                self.tx_stop.set()
                self.pwm_poll_stop.set()

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

    def send_frame(self, cmd, payload=b""):
        frame = build_frame(cmd, payload)
        if not self.conn:
            return
        try:
            self.tx_q.put_nowait(frame)
        except queue.Full:
            self.log_put("TX queue full: dropping frame")

    def tx_loop(self):
        while not self.tx_stop.is_set():
            try:
                data = self.tx_q.get(timeout=0.1)
            except queue.Empty:
                continue

            if not self.conn:
                continue

            try:
                self.conn.sendall(data)
            except Exception as e:
                self.log_put(f"Send failed: {e}")
                break

    def pwm_poll_loop(self):
        while not self.pwm_poll_stop.is_set():
            if self.conn and self.pwm_poll_on:
                self.send_frame(CMD_TELEM_READ)
            time.sleep(self.pwm_poll_interval_s)

    def send_ping(self):
        self.send_frame(CMD_PING)
        self.log_put("TX: PING")

    def send_manual_pwm(self):
        if self.takeoff_on:
            return
        val = int(self.pwm_var.get())
        payload = struct.pack("<HHHH", val, val, val, val)
        self.send_frame(CMD_PWM_SET, payload)
        self.log_put(f"TX: MANUAL PWM={val}")

    def send_nav(self, action):
        if not self.conn:
            return

        self.desired_action = int(action)

        base = self.current_base_sent if self.takeoff_on else int(self.pwm_var.get())
        payload = struct.pack("<HB", base, int(self.desired_action))
        self.send_frame(CMD_NAV_SET, payload)

        self.log_put(f"TX: NAV act={self.desired_action} base={base}")

    def toggle_takeoff(self):
        if not self.conn:
            return

        if not self.takeoff_on:
            self.takeoff_on = True
            self.takeoff_stop.clear()
            self.desired_action = NAV_STOP
            self.btn_takeoff.config(text="DISARM (STAB_OFF)")
            self.log_put("TAKEOFF: start smooth ramp (action stays changeable)")

            def ramp_worker():
                base = self.current_base_sent
                step = 80
                interval = 0.03

                while not self.takeoff_stop.is_set() and self.conn:
                    target = int(self.pwm_var.get())

                    if base < target:
                        base = min(base + step, target)
                    elif base > target:
                        base = max(base - step, target)

                    self.current_base_sent = base
                    payload = struct.pack("<HB", base, int(self.desired_action))
                    self.send_frame(CMD_NAV_SET, payload)

                    time.sleep(interval)

                self.log_put("TAKEOFF: ramp stopped")

            self.takeoff_thread = threading.Thread(target=ramp_worker, daemon=True)
            self.takeoff_thread.start()

        else:
            self.takeoff_on = False
            self.takeoff_stop.set()

            self.send_frame(CMD_STAB_OFF)
            self.log_put("TX: STAB_OFF (PWM_SetSafe)")

            self.btn_takeoff.config(text="TAKEOFF (smooth)")

    def _append_telem_row(self, t_s, roll, pitch, yaw, lf, rf, lb, rb):
        with self.data_lock:
            self.data_rows.append({
                "t_s": t_s,
                "base": int(self.current_base_sent),
                "action": int(self.desired_action),
                "action_name": NAV_STR.get(int(self.desired_action), str(int(self.desired_action))),
                "takeoff_on": int(self.takeoff_on),

                "roll_deg": float(roll),
                "pitch_deg": float(pitch),
                "yaw_deg": float(yaw),

                "pwm_lf": int(lf),
                "pwm_rf": int(rf),
                "pwm_lb": int(lb),
                "pwm_rb": int(rb),
            })

    def export_csv(self):
        with self.data_lock:
            rows = list(self.data_rows)

        if not rows:
            messagebox.showinfo("Export CSV", "Brak danych do eksportu (jeszcze nic nie przyszło z telemetrii).")
            return

        default_name = time.strftime("drone_log_%Y-%m-%d_%H-%M-%S.csv")
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not path:
            return

        fieldnames = [
            "t_s",
            "base",
            "action",
            "action_name",
            "takeoff_on",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "pwm_lf",
            "pwm_rf",
            "pwm_lb",
            "pwm_rb",
        ]

        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=fieldnames)
                w.writeheader()
                for r in rows:
                    w.writerow(r)

            self.log_put(f"EXPORT: zapisano {len(rows)} wierszy do: {path}")
        except Exception as e:
            messagebox.showerror("Export CSV", f"Nie udało się zapisać CSV:\n{e}")

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
                        self.current_base_sent = base
                        self.log_put(f"RX: NAV ACK ✓ base={base} act={act}")
                    else:
                        self.log_put("RX: NAV ACK ✓")

                elif cmd == CMD_PWM_DATA:
                    if len(payload) == 8:
                        lb, lf, rf, rb = struct.unpack("<HHHH", payload)
                        self.last_pwm = {"LF": lf, "RF": rf, "LB": lb, "RB": rb}

                        t = time.monotonic() - self.t0
                        self.t_hist.append(t)
                        self.pwm_hist["LF"].append(lf)
                        self.pwm_hist["RF"].append(rf)
                        self.pwm_hist["LB"].append(lb)
                        self.pwm_hist["RB"].append(rb)
                    else:
                        self.log_put(f"RX: PWM DATA bad len={len(payload)}")

                elif cmd == CMD_STAB_OFF_ACK:
                    self.log_put("RX: STAB_OFF ACK ✓")

                elif cmd == CMD_STATUS and len(payload) == 1:
                    err = payload[0]
                    self.log_put("RX STATUS: " + STATUS_STR.get(err, f"0x{err:02X}"))

                elif cmd == CMD_IMU_DATA and len(payload) == 6:
                    roll, pitch, yaw = struct.unpack("<hhh", payload)
                    r = roll / 100.0
                    p = pitch / 100.0
                    y = yaw / 100.0
                    self.last_imu = {"roll": r, "pitch": p, "yaw": y}
                    self.log_put(f"RX IMU: R={r:.1f} P={p:.1f} Y={y:.1f}")

                elif cmd == CMD_TELEM_DATA:
                    if len(payload) == 14:
                        roll, pitch, yaw, lb, lf, rf, rb = struct.unpack("<hhhHHHH", payload)

                        r = roll / 100.0
                        p = pitch / 100.0
                        y = yaw / 100.0
                        self.last_imu = {"roll": r, "pitch": p, "yaw": y}

                        self.root.after(0, lambda rr=r, pp=p, yy=y: (
                            self.roll_var.set(f"ROLL: {rr:+.1f}°"),
                            self.pitch_var.set(f"PITCH: {pp:+.1f}°"),
                            self.yaw_var.set(f"YAW: {yy:+.1f}°")
                        ))

                        self.last_pwm = {"LF": lf, "RF": rf, "LB": lb, "RB": rb}

                        t = time.monotonic() - self.t0
                        self.t_hist.append(t)
                        self.pwm_hist["LF"].append(lf)
                        self.pwm_hist["RF"].append(rf)
                        self.pwm_hist["LB"].append(lb)
                        self.pwm_hist["RB"].append(rb)

                        self._append_telem_row(t, r, p, y, lf, rf, lb, rb)
                    else:
                        self.log_put(f"RX: TELEM bad len={len(payload)}")

            time.sleep(0.005)

def main():
    root = tk.Tk()
    app = App(root)
    root.mainloop()

if __name__ == "__main__":
    main()
