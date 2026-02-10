import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import pandas as pd

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure


REQUIRED_COLUMNS = [
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

OPTIONAL_COLUMNS = [
    "roll_cal_deg",
    "pitch_cal_deg",
    "err_roll_deg",
    "err_pitch_deg",
    "u_roll",
    "u_pitch",
    "u_yaw",
    "k",
    "sat",
    "i_enabled",
]


class LogViewerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Drone Log Viewer (BASE/PWM + angles + debug)")
        self.geometry("1280x780")
        self.minsize(980, 640)

        self.df = None
        self.filepath = None

        self._build_ui()

    def _build_ui(self):
        menubar = tk.Menu(self)
        file_menu = tk.Menu(menubar, tearoff=False)
        file_menu.add_command(label="Otwórz log…", command=self.open_file, accelerator="Ctrl+O")
        file_menu.add_separator()
        file_menu.add_command(label="Zamknij", command=self.destroy, accelerator="Ctrl+Q")
        menubar.add_cascade(label="Plik", menu=file_menu)
        self.config(menu=menubar)

        self.bind_all("<Control-o>", lambda e: self.open_file())
        self.bind_all("<Control-q>", lambda e: self.destroy())

        top = ttk.Frame(self, padding=(10, 8, 10, 6))
        top.pack(side=tk.TOP, fill=tk.X)

        self.file_label = ttk.Label(top, text="Brak pliku. Otwórz log CSV.", font=("Segoe UI", 10))
        self.file_label.pack(side=tk.LEFT, fill=tk.X, expand=True)

        ttk.Button(top, text="Otwórz log…", command=self.open_file).pack(side=tk.RIGHT)

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

        self.tab_plots = ttk.Frame(self.notebook)
        self.tab_angles = ttk.Frame(self.notebook)
        self.tab_debug = ttk.Frame(self.notebook)
        self.tab_actions = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_plots, text="Wykresy")
        self.notebook.add(self.tab_angles, text="Kąty")
        self.notebook.add(self.tab_debug, text="Debug (PID/mixer)")
        self.notebook.add(self.tab_actions, text="Akcje")

        self._build_plots_tab()
        self._build_angles_tab()
        self._build_debug_tab()
        self._build_actions_tab()

        self.status = ttk.Label(self, text="Gotowe.", anchor="w")
        self.status.pack(side=tk.BOTTOM, fill=tk.X)

    def set_status(self, text):
        self.status.config(text=text)
        self.update_idletasks()

    def open_file(self):
        path = filedialog.askopenfilename(
            title="Wybierz plik logu",
            filetypes=[("CSV", "*.csv"), ("Wszystkie pliki", "*.*")],
        )
        if not path:
            return
        self.load_file(path)

    def load_file(self, path):
        self.set_status("Wczytuję plik…")
        try:
            df = pd.read_csv(path)
        except Exception as e:
            messagebox.showerror("Błąd", f"Nie udało się wczytać CSV:\n{e}")
            self.set_status("Błąd wczytywania.")
            return

        missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
        if missing:
            messagebox.showerror(
                "Niepoprawny format",
                "Brak wymaganych kolumn:\n" + "\n".join(missing) + "\n\nDostępne kolumny:\n" + ", ".join(df.columns),
            )
            self.set_status("Niepoprawny format.")
            return

        df = df.copy()

        for c in OPTIONAL_COLUMNS:
            if c not in df.columns:
                df[c] = pd.NA

        df["t_s"] = pd.to_numeric(df["t_s"], errors="coerce")
        df = df.dropna(subset=["t_s"]).sort_values("t_s").reset_index(drop=True)

        for c in ["base", "action", "takeoff_on", "pwm_lf", "pwm_rf", "pwm_lb", "pwm_rb", "sat", "i_enabled"]:
            df[c] = pd.to_numeric(df[c], errors="coerce")

        for c in ["roll_deg", "pitch_deg", "yaw_deg", "roll_cal_deg", "pitch_cal_deg",
                  "err_roll_deg", "err_pitch_deg", "k", "u_roll", "u_pitch", "u_yaw"]:
            df[c] = pd.to_numeric(df[c], errors="coerce")

        self.df = df
        self.filepath = path
        self.file_label.config(text=f"Plik: {path}")
        self.set_status(f"Wczytano {len(df)} rekordów.")
        self.refresh_all()

    def _build_plots_tab(self):
        container = ttk.Frame(self.tab_plots)
        container.pack(fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(8, 7), dpi=100)
        self.ax_base = self.fig.add_subplot(311)
        self.ax_pwm = self.fig.add_subplot(312, sharex=self.ax_base)
        self.ax_dbg = self.fig.add_subplot(313, sharex=self.ax_base)

        self.canvas = FigureCanvasTkAgg(self.fig, master=container)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        toolbar = NavigationToolbar2Tk(self.canvas, container)
        toolbar.update()
        toolbar.pack(side=tk.TOP, fill=tk.X)

    def _build_angles_tab(self):
        outer = ttk.Frame(self.tab_angles)
        outer.pack(fill=tk.BOTH, expand=True)

        self.angles_tree = self._make_treeview(
            outer,
            columns=[
                ("t_s", "t_s"),
                ("roll_deg", "roll_deg"),
                ("pitch_deg", "pitch_deg"),
                ("yaw_deg", "yaw_deg"),
                ("roll_cal_deg", "roll_cal_deg"),
                ("pitch_cal_deg", "pitch_cal_deg"),
            ],
        )

    def _build_debug_tab(self):
        outer = ttk.Frame(self.tab_debug)
        outer.pack(fill=tk.BOTH, expand=True)

        self.debug_tree = self._make_treeview(
            outer,
            columns=[
                ("t_s", "t_s"),
                ("err_roll_deg", "err_roll_deg"),
                ("err_pitch_deg", "err_pitch_deg"),
                ("u_roll", "u_roll"),
                ("u_pitch", "u_pitch"),
                ("u_yaw", "u_yaw"),
                ("k", "k"),
                ("sat", "sat"),
                ("i_enabled", "i_enabled"),
            ],
        )

    def _build_actions_tab(self):
        outer = ttk.Frame(self.tab_actions)
        outer.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(outer, padding=(0, 0, 0, 6))
        top.pack(side=tk.TOP, fill=tk.X)

        self.only_changes = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            top,
            text="Pokaż tylko zmiany (segmenty akcji)",
            variable=self.only_changes,
            command=self.refresh_actions_table,
        ).pack(side=tk.LEFT)

        self.actions_tree = self._make_treeview(
            outer,
            columns=[
                ("start_t_s", "start_t_s"),
                ("end_t_s", "end_t_s"),
                ("action", "action"),
                ("action_name", "action_name"),
                ("takeoff_on", "takeoff_on"),
            ],
        )

    def _make_treeview(self, parent, columns):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.BOTH, expand=True)

        tree = ttk.Treeview(frame, columns=[c[0] for c in columns], show="headings", height=18)
        vsb = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
        hsb = ttk.Scrollbar(frame, orient="horizontal", command=tree.xview)
        tree.configure(yscroll=vsb.set, xscroll=hsb.set)

        tree.grid(row=0, column=0, sticky="nsew")
        vsb.grid(row=0, column=1, sticky="ns")
        hsb.grid(row=1, column=0, sticky="ew")

        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        for col_id, col_title in columns:
            tree.heading(col_id, text=col_title)
            tree.column(col_id, width=130, anchor="center", stretch=True)

        return tree

    def _clear_tree(self, tree):
        for iid in tree.get_children():
            tree.delete(iid)

    def refresh_all(self):
        if self.df is None or self.df.empty:
            return
        self.refresh_plots()
        self.refresh_angles_table()
        self.refresh_debug_table()
        self.refresh_actions_table()

    def refresh_plots(self):
        df = self.df
        t = df["t_s"].values

        self.ax_base.clear()
        self.ax_pwm.clear()
        self.ax_dbg.clear()

        self.ax_base.set_title("BASE / takeoff_on")
        self.ax_base.set_ylabel("base")
        self.ax_base.plot(t, df["base"].values, label="base")
        if df["takeoff_on"].notna().any():
            self.ax_base.plot(t, df["takeoff_on"].values, label="takeoff_on")
        self.ax_base.grid(True, alpha=0.25)
        self.ax_base.legend(loc="upper right")

        self.ax_pwm.set_title("PWM")
        self.ax_pwm.set_ylabel("pwm")
        self.ax_pwm.plot(t, df["pwm_lf"].values, label="pwm_lf")
        self.ax_pwm.plot(t, df["pwm_rf"].values, label="pwm_rf")
        self.ax_pwm.plot(t, df["pwm_lb"].values, label="pwm_lb")
        self.ax_pwm.plot(t, df["pwm_rb"].values, label="pwm_rb")
        self.ax_pwm.grid(True, alpha=0.25)
        self.ax_pwm.legend(loc="upper right")

        self.ax_dbg.set_title("Debug: err/u/k/sat")
        self.ax_dbg.set_xlabel("t_s [s]")

        any_dbg = False
        if df["err_roll_deg"].notna().any():
            self.ax_dbg.plot(t, df["err_roll_deg"].values, label="err_roll_deg")
            any_dbg = True
        if df["err_pitch_deg"].notna().any():
            self.ax_dbg.plot(t, df["err_pitch_deg"].values, label="err_pitch_deg")
            any_dbg = True
        if df["u_roll"].notna().any():
            self.ax_dbg.plot(t, df["u_roll"].values, label="u_roll")
            any_dbg = True
        if df["u_pitch"].notna().any():
            self.ax_dbg.plot(t, df["u_pitch"].values, label="u_pitch")
            any_dbg = True
        if df["k"].notna().any():
            self.ax_dbg.plot(t, df["k"].values, label="k")
            any_dbg = True
        if df["sat"].notna().any():
            self.ax_dbg.plot(t, df["sat"].values, label="sat")
            any_dbg = True

        if any_dbg:
            self.ax_dbg.grid(True, alpha=0.25)
            self.ax_dbg.legend(loc="upper right")
        else:
            self.ax_dbg.text(0.02, 0.5, "Brak kolumn debug w tym logu.", transform=self.ax_dbg.transAxes)

        self.fig.tight_layout()
        self.canvas.draw_idle()

    def refresh_angles_table(self):
        df = self.df
        tree = self.angles_tree
        self._clear_tree(tree)

        cols = ["t_s", "roll_deg", "pitch_deg", "yaw_deg", "roll_cal_deg", "pitch_cal_deg"]
        view = df[cols].copy()

        for _, row in view.iterrows():
            def fmt(x):
                return "" if pd.isna(x) else f"{float(x):.3f}"
            tree.insert(
                "",
                "end",
                values=[
                    fmt(row["t_s"]),
                    fmt(row["roll_deg"]),
                    fmt(row["pitch_deg"]),
                    fmt(row["yaw_deg"]),
                    fmt(row["roll_cal_deg"]),
                    fmt(row["pitch_cal_deg"]),
                ],
            )

    def refresh_debug_table(self):
        df = self.df
        tree = self.debug_tree
        self._clear_tree(tree)

        cols = ["t_s", "err_roll_deg", "err_pitch_deg", "u_roll", "u_pitch", "u_yaw", "k", "sat", "i_enabled"]
        view = df[cols].copy()

        for _, row in view.iterrows():
            def fmt(x):
                return "" if pd.isna(x) else str(int(x)) if isinstance(x, (int,)) else f"{float(x):.3f}"
            def fmt_int(x):
                return "" if pd.isna(x) else str(int(float(x)))
            tree.insert(
                "",
                "end",
                values=[
                    "" if pd.isna(row["t_s"]) else f"{float(row['t_s']):.3f}",
                    "" if pd.isna(row["err_roll_deg"]) else f"{float(row['err_roll_deg']):.3f}",
                    "" if pd.isna(row["err_pitch_deg"]) else f"{float(row['err_pitch_deg']):.3f}",
                    "" if pd.isna(row["u_roll"]) else f"{int(float(row['u_roll'])):+d}",
                    "" if pd.isna(row["u_pitch"]) else f"{int(float(row['u_pitch'])):+d}",
                    "" if pd.isna(row["u_yaw"]) else f"{int(float(row['u_yaw'])):+d}",
                    "" if pd.isna(row["k"]) else f"{float(row['k']):.3f}",
                    fmt_int(row["sat"]),
                    fmt_int(row["i_enabled"]),
                ],
            )

    def build_action_segments(self):
        df = self.df
        cols = ["t_s", "action", "action_name", "takeoff_on"]
        d = df[cols].copy()
        d["action_name"] = d["action_name"].astype(str)

        change = (
            (d["action"].shift(1) != d["action"]) |
            (d["action_name"].shift(1) != d["action_name"]) |
            (d["takeoff_on"].shift(1) != d["takeoff_on"])
        )
        change.iloc[0] = True
        starts = d[change].copy()
        starts["start_t_s"] = starts["t_s"]

        ends = starts["start_t_s"].shift(-1)
        starts["end_t_s"] = ends.fillna(d["t_s"].iloc[-1])

        segments = starts[["start_t_s", "end_t_s", "action", "action_name", "takeoff_on"]].reset_index(drop=True)
        return segments

    def refresh_actions_table(self):
        if self.df is None:
            return

        tree = self.actions_tree
        self._clear_tree(tree)

        if self.only_changes.get():
            seg = self.build_action_segments()
            for _, row in seg.iterrows():
                tree.insert(
                    "",
                    "end",
                    values=[
                        f"{row['start_t_s']:.3f}",
                        f"{row['end_t_s']:.3f}",
                        "" if pd.isna(row["action"]) else str(int(float(row["action"]))),
                        str(row["action_name"]),
                        "" if pd.isna(row["takeoff_on"]) else str(int(float(row["takeoff_on"]))),
                    ],
                )
        else:
            df = self.df
            cols = ["t_s", "action", "action_name", "takeoff_on"]
            for _, row in df[cols].iterrows():
                tree.insert(
                    "",
                    "end",
                    values=[
                        f"{row['t_s']:.3f}",
                        "",
                        "" if pd.isna(row["action"]) else str(int(float(row["action"]))),
                        str(row["action_name"]),
                        "" if pd.isna(row["takeoff_on"]) else str(int(float(row["takeoff_on"]))),
                    ],
                )


def main():
    app = LogViewerApp()
    app.mainloop()


if __name__ == "__main__":
    main()
