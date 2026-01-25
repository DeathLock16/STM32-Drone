import socket
import struct
import threading
import time
import queue
import tkinter as tk
from tkinter import ttk, filedialog
from tkinter.scrolledtext import ScrolledText
import os

import numpy as np
import trimesh

from pyopengltk import OpenGLFrame
from OpenGL.GL import *
from OpenGL.GLU import *


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

CMD_IMU_READ = 0x20
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
        if frame_len > 256:
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


class StlViewer(OpenGLFrame):
    def __init__(self, master, **kw):
        super().__init__(master, **kw)

        self._have_mesh = False
        self._verts = None
        self._faces = None
        self._normals = None

        self._rot_roll = 0.0
        self._rot_pitch = 0.0
        self._rot_yaw = 0.0

        self._scale = 1.0
        self._center = np.zeros(3, dtype=np.float32)

        self._mouse_down = False
        self._last_mouse = None
        self._drag_yaw = 0.0
        self._drag_pitch = 0.0

        self._loop_running = False
        self._fps_ms = 33  # 30 FPS; daj 16 jeśli chcesz 60 FPS

        self.bind("<ButtonPress-1>", self._on_mouse_down)
        self.bind("<ButtonRelease-1>", self._on_mouse_up)
        self.bind("<B1-Motion>", self._on_mouse_drag)

    def initgl(self):
        glClearColor(0.12, 0.12, 0.12, 1.0)
        glEnable(GL_DEPTH_TEST)

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

        glLightfv(GL_LIGHT0, GL_POSITION, (0.3, 0.7, 1.0, 0.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (1.0, 1.0, 1.0, 1.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.2, 0.2, 0.2, 1.0))

        glShadeModel(GL_SMOOTH)

        if not self._loop_running:
            self._loop_running = True
            self.after(self._fps_ms, self._render_loop)

    def _render_loop(self):
        if not self.winfo_exists():
            return
        try:
            self.tkRedraw()
        except Exception:
            return
        self.after(self._fps_ms, self._render_loop)

    def _on_mouse_down(self, e):
        self._mouse_down = True
        self._last_mouse = (e.x, e.y)

    def _on_mouse_up(self, _e):
        self._mouse_down = False
        self._last_mouse = None

    def _on_mouse_drag(self, e):
        if not self._mouse_down or self._last_mouse is None:
            return
        lx, ly = self._last_mouse
        dx = e.x - lx
        dy = e.y - ly
        self._last_mouse = (e.x, e.y)

        self._drag_yaw += dx * 0.4
        self._drag_pitch += dy * 0.4

    def load_stl(self, path: str):
        mesh = trimesh.load(path, force="mesh")
        if mesh is None or mesh.is_empty:
            self._have_mesh = False
            self._verts = None
            self._faces = None
            self._normals = None
            return

        verts = np.array(mesh.vertices, dtype=np.float32)
        faces = np.array(mesh.faces, dtype=np.uint32).reshape(-1)

        if mesh.vertex_normals is not None and len(mesh.vertex_normals) == len(mesh.vertices):
            normals = np.array(mesh.vertex_normals, dtype=np.float32)
        else:
            mesh.rezero()
            mesh.compute_vertex_normals()
            normals = np.array(mesh.vertex_normals, dtype=np.float32)

        mins = verts.min(axis=0)
        maxs = verts.max(axis=0)
        center = (mins + maxs) * 0.5
        size = (maxs - mins)
        max_dim = float(np.max(size)) if float(np.max(size)) > 1e-6 else 1.0
        scale = 1.0 / max_dim

        self._verts = verts
        self._faces = faces
        self._normals = normals
        self._center = center
        self._scale = scale

        self._have_mesh = True
        self._drag_yaw = 0.0
        self._drag_pitch = 0.0

    def _request_redraw(self):
        if hasattr(self, "tkRedraw"):
            try:
                self.tkRedraw()
                return
            except Exception:
                pass
        try:
            self.after(1, lambda: None)
        except Exception:
            pass

    def set_rotation(self, roll_deg: float, pitch_deg: float, yaw_deg: float):
        self._rot_roll = float(roll_deg)
        self._rot_pitch = float(pitch_deg)
        self._rot_yaw = float(yaw_deg)
        self._request_redraw()


    def redraw(self):
        w = max(1, int(self.winfo_width()))
        h = max(1, int(self.winfo_height()))

        glViewport(0, 0, w, h)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, float(w) / float(h), 0.01, 100.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(0.0, 0.0, 2.2,  0.0, 0.0, 0.0,  0.0, 1.0, 0.0)

        glRotatef(self._drag_pitch, 1.0, 0.0, 0.0)
        glRotatef(self._drag_yaw,   0.0, 1.0, 0.0)

        # Mapowanie jak w Blenderze:
        # obj.rotation_euler = (pitch, -roll, yaw)
        # Blender: obj.rotation_euler = (pitch, -roll, yaw)
        # => kolejność w macierzy: R = Rz(yaw) * Ry(-roll) * Rx(pitch)
        glRotatef(self._rot_yaw,    0.0, 0.0, 1.0)   # yaw  -> Z
        glRotatef(-self._rot_roll,  0.0, 1.0, 0.0)   # -roll -> Y
        glRotatef(self._rot_pitch,  1.0, 0.0, 0.0)   # pitch -> X


        if not self._have_mesh:
            return

        glScalef(self._scale, self._scale, self._scale)
        glTranslatef(-self._center[0], -self._center[1], -self._center[2])

        glColor3f(0.85, 0.85, 0.9)

        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_NORMAL_ARRAY)

        glVertexPointerf(self._verts)
        glNormalPointerf(self._normals)

        glDrawElements(GL_TRIANGLES, self._faces.size, GL_UNSIGNED_INT, self._faces)

        glDisableClientState(GL_NORMAL_ARRAY)
        glDisableClientState(GL_VERTEX_ARRAY)

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Control (TCP server)")
        self.root.geometry("1200x700")

        self.log_q = queue.Queue()

        self.srv = None
        self.conn = None
        self.running = False
        self.rx_buf = b""

        self.tx_q = queue.Queue()
        self.tx_thread = None
        self.rx_thread = None
        self.send_lock = threading.Lock()

        self.imu_polling = False
        self.imu_period_ms = 50  # 20 Hz

        self.pwm_var = tk.IntVar(value=0)
        self.last_nav_action = NAV_STOP

        self.viewer = None
        self.lbl_stl = None

        self._build_ui()
        self._ui_poll()

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="both", expand=True)

        top.columnconfigure(0, weight=0)
        top.columnconfigure(1, weight=0)
        top.columnconfigure(2, weight=1)
        top.rowconfigure(0, weight=0)
        top.rowconfigure(1, weight=1)

        left = ttk.Frame(top)
        left.grid(row=0, column=0, sticky="nsw", padx=(0, 10))

        controls_col = ttk.Frame(top)
        controls_col.grid(row=0, column=1, sticky="nw", padx=(0, 10))

        viewer_col = ttk.Frame(top)
        viewer_col.grid(row=0, column=2, sticky="nsew")

        conn_box = ttk.LabelFrame(left, text="Connection", padding=8)
        conn_box.pack(fill="x")

        self.btn_start = ttk.Button(conn_box, text="Start server", command=self.start_server)
        self.btn_start.pack(fill="x")

        self.btn_ping = ttk.Button(conn_box, text="Ping", command=self.send_ping, state="disabled")
        self.btn_ping.pack(fill="x", pady=(6, 0))

        self.lbl_state = ttk.Label(conn_box, text="State: DISCONNECTED")
        self.lbl_state.pack(fill="x", pady=(6, 0))

        slider_box = ttk.LabelFrame(left, text="Base PWM", padding=8)
        slider_box.pack(fill="y", pady=(10, 0))

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
        self.btn_set_manual.pack(fill="x", pady=(6, 0))

        ctrl_box = ttk.LabelFrame(controls_col, text="Controls", padding=8)
        ctrl_box.pack(anchor="nw")

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

        viewer_box = ttk.LabelFrame(viewer_col, text="3D Model (STL) + IMU rotation", padding=6)
        viewer_box.pack(fill="both", expand=True)

        viewer_top = ttk.Frame(viewer_box)
        viewer_top.pack(fill="x")

        self.btn_load_stl = ttk.Button(viewer_top, text="Load STL...", command=self._pick_and_load_stl)
        self.btn_load_stl.pack(side="left")

        self.lbl_stl = ttk.Label(viewer_top, text="(no model)")
        self.lbl_stl.pack(side="left", padx=(8, 0))

        self.viewer = StlViewer(viewer_box, width=800, height=450)
        self.viewer.pack(fill="both", expand=True, pady=(6, 0))

        self.viewer.animate = 1

        log_box = ttk.LabelFrame(top, text="Logs", padding=8)
        log_box.grid(row=1, column=0, columnspan=3, sticky="nsew", pady=(10, 0))

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
            NAV_STOP: "STOP", NAV_FORWARD: "FWD", NAV_RIGHT: "RIGHT", NAV_BACK: "BACK",
            NAV_LEFT: "LEFT", NAV_YAW_RIGHT: "YAW_R", NAV_YAW_LEFT: "YAW_L"
        }.get(action, str(action))
        self.log_put(f"TX: NAV (slider release) {name} base={base}")

    def log_put(self, msg: str):
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

    def _pick_and_load_stl(self):
        path = filedialog.askopenfilename(
            title="Select STL",
            filetypes=[("STL files", "*.stl"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            self.viewer.load_stl(path)
            self.lbl_stl.config(text=os.path.basename(path))
            self.log_put(f"Loaded STL: {path}")
        except Exception as e:
            self.log_put(f"STL load failed: {e}")

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

                self._start_tx_thread()
                self._start_imu_polling()

                self.rx_loop()

            except Exception as e:
                self.log_put(f"Server error: {e}")
            finally:
                self._stop_imu_polling()

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

    def _start_tx_thread(self):
        if self.tx_thread and self.tx_thread.is_alive():
            return

        def tx_worker():
            while self.running and self.conn:
                try:
                    data = self.tx_q.get(timeout=0.2)
                except queue.Empty:
                    continue

                try:
                    with self.send_lock:
                        self.conn.sendall(data)
                except Exception as e:
                    self.log_put(f"Send failed: {e}")
                    break

        self.tx_thread = threading.Thread(target=tx_worker, daemon=True)
        self.tx_thread.start()

    def send(self, data: bytes):
        if not self.conn:
            return
        self.tx_q.put(data)

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
            NAV_STOP: "STOP", NAV_FORWARD: "FWD", NAV_RIGHT: "RIGHT", NAV_BACK: "BACK",
            NAV_LEFT: "LEFT", NAV_YAW_RIGHT: "YAW_R", NAV_YAW_LEFT: "YAW_L"
        }.get(action, str(action))
        self.log_put(f"TX: NAV {name} base={base}")

    def _start_imu_polling(self):
        if self.imu_polling:
            return
        self.imu_polling = True
        self.log_put("IMU polling: START")

        def tick():
            if not self.imu_polling or not self.conn:
                return
            self.send(build_frame(CMD_IMU_READ))
            self.root.after(self.imu_period_ms, tick)

        self.root.after(0, tick)

    def _stop_imu_polling(self):
        if self.imu_polling:
            self.imu_polling = False
            self.log_put("IMU polling: STOP")

    def rx_loop(self):
        while self.running and self.conn:
            try:
                data = self.conn.recv(512)
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
                    roll_i, pitch_i, yaw_i = struct.unpack("<hhh", payload)
                    r = roll_i / 100.0
                    p = pitch_i / 100.0
                    y = yaw_i / 100.0
                    self.root.after(0, lambda rr=r, pp=p, yy=y: self.viewer.set_rotation(rr, pp, yy))


            time.sleep(0.005)


def main():
    root = tk.Tk()
    app = App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
