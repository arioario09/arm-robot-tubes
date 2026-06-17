import tkinter as tk
from tkinter import ttk, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import serial
import serial.tools.list_ports
import threading
import time

class RobotArmController:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Arm Controller")
        self.root.geometry("1300x820")

        self.theta1 = tk.DoubleVar(value=0)
        self.theta2 = tk.DoubleVar(value=0)
        self.theta3 = tk.DoubleVar(value=0)

        self.L1 = 2.0
        self.L2 = 2.5
        self.L3 = 2.0

        self.STEPS_PER_REV = 3200
        self.DEG_PER_REV   = 360
        self.gear_ratio = {1: 1.0, 2: 1.0, 3: 1.0}

        # ⚙️ ── KALIBRASI VISUALISASI PLOT (SESUAIKAN DENGAN ROBOT FISIK ANDA) ── ⚙️
        # 1. INVERT: Ubah ke True jika arah gerakan di plot TERBALIK dengan robot asli Anda.
        self.INVERT_PLOT_AXIS = {
            1: False,  # True = membalik arah putar Base di plot
            2: True,  # True = membalik arah putar Shoulder di plot
            3: True   # True = membalik arah putar Elbow di plot
        }
        
        # 2. OFFSET: Tambah/kurang derajat jika posisi awal (0°) di plot berbeda dengan robot asli.
        # Contoh: Jika posisi awal robot asli tegak lurus ke atas (90°), Anda bisa isi 90.0 atau -90.0 disini.
        self.OFFSET_PLOT_AXIS = {
            1: 0.0,    # Offset sudut Base (derajat)
            2: 90.0,    # Offset sudut Shoulder (derajat)
            3: 0.0     # Offset sudut Elbow (derajat)
        }
        # ────────────────────────────────────────────────────────────────────────

        self.serial_port = None
        self.is_connected = False

        self.waypoints = []   # list of (t1, t2, t3, gripper_cmd)
        self.is_playing = False
        self.gripper_pos_est = 0   # estimasi posisi gripper (langkah)

        self.setup_gui()
        self.update_plot()

    def deg_to_steps(self, deg, axis):
        steps = int((deg / self.DEG_PER_REV) * self.STEPS_PER_REV * self.gear_ratio[axis])
        if axis == 2:
            return -steps  # Sesuai permintaan sebelumnya: Axis 2 minus untuk maju
        return steps

    # ── GUI ───────────────────────────────────────────────────
    def setup_gui(self):
        self.root.columnconfigure(0, weight=0)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        # ── Left panel ────────────────────────────────────────
        left = ttk.Frame(self.root, padding=10)
        left.grid(row=0, column=0, sticky="nsew", padx=(8,4), pady=8)
        left.columnconfigure(0, weight=1)

        # Serial
        sf = ttk.LabelFrame(left, text="Serial Connection", padding=8)
        sf.grid(row=0, column=0, sticky="ew", pady=(0,6))
        sf.columnconfigure(1, weight=1)

        ttk.Label(sf, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(sf, width=14)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(4,0))
        self.refresh_ports()

        ttk.Label(sf, text="Baud Rate:").grid(row=1, column=0, sticky="w", pady=(4,0))
        self.baud_combo = ttk.Combobox(sf, width=14, values=[9600,19200,38400,57600,115200])
        self.baud_combo.set(115200)
        self.baud_combo.grid(row=1, column=1, sticky="ew", padx=(4,0), pady=(4,0))

        br = ttk.Frame(sf)
        br.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6,0))
        ttk.Button(br, text="Refresh", command=self.refresh_ports).pack(side="left", padx=(0,4))
        self.btn_connect = ttk.Button(br, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side="left")

        self.conn_label = ttk.Label(sf, text="● Disconnected", foreground="red")
        self.conn_label.grid(row=3, column=0, columnspan=2, pady=(4,0))

        # Sliders axis 1-3
        for i, (label, var) in enumerate([
            ("θ1 - Base Rotation", self.theta1),
            ("θ2 - Shoulder (Minus = Maju)",  self.theta2),
            ("θ3 - Elbow",         self.theta3),
        ], start=1):
            self._make_slider(left, label, var, i)

        # ── GRIPPER CONTROL ───────────────────────────────────
        gf = ttk.LabelFrame(left, text="Gripper (28BYJ-48)", padding=8)
        gf.grid(row=4, column=0, sticky="ew", pady=(0,6))
        gf.columnconfigure(1, weight=1)

        qr = ttk.Frame(gf)
        qr.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0,6))
        ttk.Button(qr, text="OPEN (-180°)",  command=self.grip_open,  width=14).pack(side="left", padx=(0,4))
        ttk.Button(qr, text="CLOSE (+180°)", command=self.grip_close, width=14).pack(side="left")

        ttk.Label(gf, text="Gerak (°):").grid(row=1, column=0, sticky="w")
        self.grip_deg_var = tk.StringVar(value="90")
        ttk.Entry(gf, textvariable=self.grip_deg_var, width=8).grid(row=1, column=1, sticky="ew", padx=4)
        dr = ttk.Frame(gf)
        dr.grid(row=1, column=2)
        ttk.Button(dr, text="+", width=3, command=lambda: self.grip_deg(+1)).pack(side="left", padx=1)
        ttk.Button(dr, text="−", width=3, command=lambda: self.grip_deg(-1)).pack(side="left")

        ttk.Label(gf, text="Langkah:").grid(row=2, column=0, sticky="w", pady=(4,0))
        self.grip_step_var = tk.StringVar(value="512")
        ttk.Entry(gf, textvariable=self.grip_step_var, width=8).grid(row=2, column=1, sticky="ew", padx=4, pady=(4,0))
        sr = ttk.Frame(gf)
        sr.grid(row=2, column=2, pady=(4,0))
        ttk.Button(sr, text="+", width=3, command=lambda: self.grip_steps(+1)).pack(side="left", padx=1)
        ttk.Button(sr, text="−", width=3, command=lambda: self.grip_steps(-1)).pack(side="left")

        kr = ttk.Frame(gf)
        kr.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(6,0))
        ttk.Label(kr, text="Speed (RPM):").pack(side="left")
        self.grip_spd_var = tk.IntVar(value=10)
        ttk.Spinbox(kr, from_=1, to=15, textvariable=self.grip_spd_var, width=4).pack(side="left", padx=(2,8))
        ttk.Button(kr, text="Set Speed", command=self.grip_set_speed).pack(side="left", padx=(0,4))
        ttk.Button(kr, text="Reset Pos", command=self.grip_reset).pack(side="left")

        self.grip_pos_label = ttk.Label(gf, text="Posisi est: 0 langkah", foreground="gray")
        self.grip_pos_label.grid(row=4, column=0, columnspan=3, sticky="w", pady=(4,0))

        # Command preview
        pf = ttk.LabelFrame(left, text="Command Preview", padding=6)
        pf.grid(row=5, column=0, sticky="ew", pady=(0,6))
        self.steps_label = ttk.Label(pf, text="G  0   0   0", font=("Courier", 10, "bold"))
        self.steps_label.pack(anchor="w")

        # Action buttons
        af = ttk.LabelFrame(left, text="Actions", padding=8)
        af.grid(row=6, column=0, sticky="ew", pady=(0,6))
        ar = ttk.Frame(af)
        ar.pack(fill="x")
        ttk.Button(ar, text="Send to Robot", command=self.send_to_robot).pack(side="left", padx=(0,4))
        ttk.Button(ar, text="Homing All",    command=self.send_homing).pack(side="left", padx=(0,4))
        ttk.Button(ar, text="STOP",          command=self.send_stop).pack(side="left")

        # Serial log
        lf = ttk.LabelFrame(left, text="Serial Log", padding=6)
        lf.grid(row=7, column=0, sticky="ew", pady=(0,6))
        self.log_text = tk.Text(lf, height=4, font=("Courier", 8), state="disabled", relief="flat")
        self.log_text.pack(fill="x")

        # ── Right panel ───────────────────────────────────────
        right = ttk.Frame(self.root, padding=8)
        right.grid(row=0, column=1, sticky="nsew", padx=(4,8), pady=8)
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)
        right.rowconfigure(1, weight=0)
        right.rowconfigure(2, weight=0)

        # 3D plot
        pf2 = ttk.LabelFrame(right, text="Robot Visualization", padding=4)
        pf2.grid(row=0, column=0, sticky="nsew", pady=(0,6))
        self.fig = Figure(figsize=(6,4.5), dpi=100)
        self.ax_3d = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=pf2)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Side views
        sf2 = ttk.LabelFrame(right, text="Top / Side / Front", padding=4)
        sf2.grid(row=1, column=0, sticky="ew", pady=(0,6))
        self.fig2 = Figure(figsize=(6,1.8), dpi=100)
        self.ax_xy = self.fig2.add_subplot(131)
        self.ax_xz = self.fig2.add_subplot(132)
        self.ax_yz = self.fig2.add_subplot(133)
        self.fig2.tight_layout(pad=1.2)
        self.canvas2 = FigureCanvasTkAgg(self.fig2, master=sf2)
        self.canvas2.get_tk_widget().pack(fill="x")

        # Waypoints
        wf = ttk.LabelFrame(right, text="Waypoints  (axis 1-3 + gripper command per titik)", padding=8)
        wf.grid(row=2, column=0, sticky="ew")
        wf.columnconfigure(0, weight=1)

        lb_wrap = ttk.Frame(wf)
        lb_wrap.grid(row=0, column=0, sticky="ew", pady=(0,6))
        lb_wrap.columnconfigure(0, weight=1)
        self.wp_listbox = tk.Listbox(lb_wrap, height=5, font=("Courier", 9), selectmode="single", relief="sunken")
        self.wp_listbox.grid(row=0, column=0, sticky="ew")
        sb = ttk.Scrollbar(lb_wrap, orient="vertical", command=self.wp_listbox.yview)
        sb.grid(row=0, column=1, sticky="ns")
        self.wp_listbox.configure(yscrollcommand=sb.set)

        gc_row = ttk.Frame(wf)
        gc_row.grid(row=1, column=0, sticky="ew", pady=(0,4))
        ttk.Label(gc_row, text="Gripper cmd saat waypoint ini:").pack(side="left")
        self.wp_grip_var = tk.StringVar(value="none")
        grip_opts = ttk.Combobox(gc_row, textvariable=self.wp_grip_var, width=14, values=["none","GOPEN","GCLOSE","GD 90","GD -90","GD 45","GD -45"])
        grip_opts.pack(side="left", padx=4)

        wb1 = ttk.Frame(wf)
        wb1.grid(row=2, column=0, sticky="ew", pady=(0,4))
        for txt, cmd in [("+ Add", self.wp_add), ("Update", self.wp_update), ("Delete", self.wp_delete), ("↑", self.wp_up), ("↓", self.wp_down), ("Go To", self.wp_goto)]:
            ttk.Button(wb1, text=txt, command=cmd).pack(side="left", padx=2)

        wb2 = ttk.Frame(wf)
        wb2.grid(row=3, column=0, sticky="ew")
        ttk.Label(wb2, text="Delay (s):").pack(side="left")
        self.delay_var = tk.DoubleVar(value=1.5)
        ttk.Spinbox(wb2, from_=0.1, to=30.0, increment=0.5, textvariable=self.delay_var, width=5).pack(side="left", padx=(2,8))
        ttk.Label(wb2, text="Repeat:").pack(side="left")
        self.repeat_var = tk.IntVar(value=1)
        ttk.Spinbox(wb2, from_=1, to=99, textvariable=self.repeat_var, width=4).pack(side="left", padx=(2,8))
        self.btn_play = ttk.Button(wb2, text="▶ Play", command=self.wp_play)
        self.btn_play.pack(side="left", padx=(0,4))
        ttk.Button(wb2, text="■ Stop", command=self.wp_stop).pack(side="left")

    def _make_slider(self, parent, label, var, row):
        f = ttk.LabelFrame(parent, text=label, padding=8)
        f.grid(row=row, column=0, sticky="ew", pady=(0,4))
        f.columnconfigure(0, weight=1)
        top = ttk.Frame(f); top.grid(row=0, column=0, sticky="ew"); top.columnconfigure(0, weight=1)
        sl = ttk.Scale(top, from_=0, to=540, variable=var, orient="horizontal", command=lambda e: self.on_slider_change(e))
        sl.grid(row=0, column=0, sticky="ew")
        lbl = ttk.Label(top, text=f"{var.get():.1f}°", width=7, anchor="e")
        lbl.grid(row=0, column=1, padx=(6,0))
        var._lbl = lbl
        bf = ttk.Frame(f); bf.grid(row=1, column=0, sticky="w", pady=(4,0))
        for d in (-10,-5,-1,1,5,10):
            ttk.Button(bf, text=f"{'+' if d>0 else ''}{d}°", width=4, command=lambda dv=d, v=var: self._adj(v, dv)).pack(side="left", padx=1)

    def _adj(self, var, delta):
        var.set(max(0, min(540, var.get() + delta)))
        self.on_slider_change(None)

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports: self.port_combo.current(0)

    def toggle_connection(self):
        if not self.is_connected:
            try:
                self.serial_port = serial.Serial(self.port_combo.get(), int(self.baud_combo.get()), timeout=1)
                self.is_connected = True
                self.btn_connect.config(text="Disconnect")
                self.conn_label.config(text="● Connected", foreground="green")
                self.log(f"Connected to {self.port_combo.get()}")
                threading.Thread(target=self.read_serial_loop, daemon=True).start()
            except Exception as e: messagebox.showerror("Error", str(e))
        else:
            if self.serial_port: self.serial_port.close()
            self.is_connected = False
            self.btn_connect.config(text="Connect")
            self.conn_label.config(text="● Disconnected", foreground="red")
            self.log("Disconnected.")

    def read_serial_loop(self):
        while self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode(errors='replace').strip()
                    if line: self.root.after(0, self.log, f"← {line}")
            except: break
            time.sleep(0.05)

    def send_raw(self, cmd):
        if not self.is_connected:
            messagebox.showwarning("Warning", "Belum terhubung.")
            return False
        try:
            self.serial_port.write((cmd + "\n").encode())
            self.log(f"→ {cmd}")
            return True
        except Exception as e:
            messagebox.showerror("Error", str(e))
            return False

    def log(self, msg):
        self.log_text.config(state="normal"); self.log_text.insert("end", msg + "\n"); self.log_text.see("end"); self.log_text.config(state="disabled")

    def _grip_update_label(self):
        self.grip_pos_label.config(text=f"Posisi est: {self.gripper_pos_est} langkah")

    def grip_open(self):
        self.send_raw("GOPEN"); self.gripper_pos_est -= int((180/360)*2048); self._grip_update_label()

    def grip_close(self):
        self.send_raw("GCLOSE"); self.gripper_pos_est += int((180/360)*2048); self._grip_update_label()

    def grip_deg(self, sign):
        try:
            deg = float(self.grip_deg_var.get()) * sign
            self.send_raw(f"GD {deg:.1f}"); self.gripper_pos_est += int((deg/360)*2048); self._grip_update_label()
        except ValueError: messagebox.showwarning("Input", "Masukkan angka derajat yang valid.")

    def grip_steps(self, sign):
        try:
            steps = int(self.grip_step_var.get()) * sign
            self.send_raw(f"GR {steps}"); self.gripper_pos_est += steps; self._grip_update_label()
        except ValueError: messagebox.showwarning("Input", "Masukkan angka langkah yang valid.")

    def grip_set_speed(self): self.send_raw(f"GSPD {self.grip_spd_var.get()}")
    def grip_reset(self): self.send_raw("GRESET"); self.gripper_pos_est = 0; self._grip_update_label()

    def _send_grip_cmd(self, cmd):
        if not cmd or cmd.lower() == "none": return
        self.send_raw(cmd); cmd_up = cmd.upper().strip()
        if cmd_up == "GOPEN": self.gripper_pos_est -= int((180/360)*2048)
        elif cmd_up == "GCLOSE": self.gripper_pos_est += int((180/360)*2048)
        elif cmd_up.startswith("GD "):
            try: self.gripper_pos_est += int((float(cmd_up[3:])/360)*2048)
            except: pass
        elif cmd_up.startswith("GR "):
            try: self.gripper_pos_est += int(cmd_up[3:])
            except: pass
        self.root.after(0, self._grip_update_label)

    def send_to_robot(self):
        s1 = self.deg_to_steps(self.theta1.get(), 1)
        s2 = self.deg_to_steps(self.theta2.get(), 2)
        s3 = self.deg_to_steps(self.theta3.get(), 3)
        self.send_raw(f"G {s1} {s2} {s3}")

    def send_homing(self):
        if messagebox.askyesno("Homing", "Jalankan homing semua axis?"): self.send_raw("HALL")
    def send_stop(self): self.send_raw("STOP")

    # ── Waypoints ─────────────────────────────────────────────
    def _wp_label(self, idx, t1, t2, t3, gcmd):
        gc = gcmd if gcmd and gcmd.lower() != "none" else "-"
        return f"  #{idx+1:02d}  θ1={t1:5.1f}° θ2={t2:5.1f}° θ3={t3:5.1f}°  grip:{gc}"

    def _refresh_listbox(self):
        self.wp_listbox.delete(0, "end")
        for i, (t1,t2,t3,gc) in enumerate(self.waypoints): self.wp_listbox.insert("end", self._wp_label(i,t1,t2,t3,gc))

    def wp_add(self):
        t1,t2,t3 = self.theta1.get(), self.theta2.get(), self.theta3.get(); gc = self.wp_grip_var.get().strip()
        self.waypoints.append((t1,t2,t3,gc)); idx = len(self.waypoints)-1
        self.wp_listbox.insert("end", self._wp_label(idx,t1,t2,t3,gc))
        self.wp_listbox.selection_clear(0,"end"); self.wp_listbox.selection_set(idx)

    def wp_update(self):
        sel = self.wp_listbox.curselection()
        if not sel: return
        i = sel[0]; t1,t2,t3 = self.theta1.get(), self.theta2.get(), self.theta3.get(); gc = self.wp_grip_var.get().strip()
        self.waypoints[i] = (t1,t2,t3,gc); self.wp_listbox.delete(i); self.wp_listbox.insert(i, self._wp_label(i,t1,t2,t3,gc)); self.wp_listbox.selection_set(i)

    def wp_delete(self):
        sel = self.wp_listbox.curselection()
        if not sel: return
        self.waypoints.pop(sel[0]); self._refresh_listbox()

    def wp_up(self):
        sel = self.wp_listbox.curselection()
        if not sel or sel[0]==0: return
        i = sel[0]; self.waypoints[i-1], self.waypoints[i] = self.waypoints[i], self.waypoints[i-1]
        self._refresh_listbox(); self.wp_listbox.selection_set(i-1)

    def wp_down(self):
        sel = self.wp_listbox.curselection()
        if not sel or sel[0]>=len(self.waypoints)-1: return
        i = sel[0]; self.waypoints[i], self.waypoints[i+1] = self.waypoints[i+1], self.waypoints[i]
        self._refresh_listbox(); self.wp_listbox.selection_set(i+1)

    def wp_goto(self):
        sel = self.wp_listbox.curselection()
        if not sel: return
        t1,t2,t3,gc = self.waypoints[sel[0]]
        self.theta1.set(t1); self.theta2.set(t2); self.theta3.set(t3); self.wp_grip_var.set(gc)
        self.on_slider_change(None); self.send_to_robot()
        if gc and gc.lower() != "none": time.sleep(0.1); self._send_grip_cmd(gc)

    def wp_play(self):
        if not self.waypoints: messagebox.showwarning("Waypoints", "Belum ada waypoint."); return
        if self.is_playing: return
        self.is_playing = True; self.btn_play.config(text="Playing...")
        threading.Thread(target=self._play_loop, daemon=True).start()

    def _play_loop(self):
        delay = self.delay_var.get(); repeat = self.repeat_var.get()
        for _ in range(repeat):
            if not self.is_playing: break
            for i, (t1,t2,t3,gc) in enumerate(self.waypoints):
                if not self.is_playing: break
                self.root.after(0, self.theta1.set, t1)
                self.root.after(0, self.theta2.set, t2)
                self.root.after(0, self.theta3.set, t3)
                self.root.after(0, self.on_slider_change, None)
                self.root.after(0, self.wp_listbox.selection_clear, 0, "end")
                self.root.after(0, self.wp_listbox.selection_set, i)
                
                s1 = self.deg_to_steps(t1,1); s2 = self.deg_to_steps(t2,2); s3 = self.deg_to_steps(t3,3)
                self.root.after(0, self.send_raw, f"G {s1} {s2} {s3}")
                
                time.sleep(delay * 0.6)
                if gc and gc.lower() != "none":
                    self._send_grip_cmd(gc)
                    try:
                        deg_val = 180.0 if gc.upper() in ("GOPEN","GCLOSE") else abs(float(gc.split()[-1]))
                        time.sleep(max(0.5, deg_val / 90.0 * 1.0))
                    except: time.sleep(1.0)
                else: time.sleep(delay * 0.4)
        self.is_playing = False; self.root.after(0, self.btn_play.config, {"text": "▶ Play"})

    def wp_stop(self): self.is_playing = False; self.send_raw("STOP"); self.btn_play.config(text="▶ Play")

    def on_slider_change(self, event):
        for var in (self.theta1, self.theta2, self.theta3):
            if hasattr(var, '_lbl'): var._lbl.config(text=f"{var.get():.1f}°")
        s1 = self.deg_to_steps(self.theta1.get(), 1)
        s2 = self.deg_to_steps(self.theta2.get(), 2)
        s3 = self.deg_to_steps(self.theta3.get(), 3)
        self.steps_label.config(text=f"G  {s1}   {s2}   {s3}"); self.update_plot()

    # ── LOGIKA FORWARD KINEMATICS DENGAN PENYESUAIAN VISUAL ──
    def fk(self, t1d, t2d, t3d):
        # BERLAKUKAN INVERSI JIKA KALIBRASI DISET 'True'
        val_t1 = -t1d if self.INVERT_PLOT_AXIS[1] else t1d
        val_t2 = -t2d if self.INVERT_PLOT_AXIS[2] else t2d
        val_t3 = -t3d if self.INVERT_PLOT_AXIS[3] else t3d

        # BERLAKUKAN OFFSET JIKA POSISI AWAL 0° BERBEDA
        val_t1 += self.OFFSET_PLOT_AXIS[1]
        val_t2 += self.OFFSET_PLOT_AXIS[2]
        val_t3 += self.OFFSET_PLOT_AXIS[3]

        t1, t2, t3 = np.radians(val_t1), np.radians(val_t2), np.radians(val_t3)
        
        p0 = np.array([0,0,0])
        p1 = np.array([0,0,self.L1])
        p2 = p1 + np.array([self.L2*np.cos(t2)*np.cos(t1),
                             self.L2*np.cos(t2)*np.sin(t1),
                             self.L2*np.sin(t2)])
        p3 = p2 + np.array([self.L3*np.cos(t2+t3)*np.cos(t1),
                             self.L3*np.cos(t2+t3)*np.sin(t1),
                             self.L3*np.sin(t2+t3)])
        return [p0,p1,p2,p3]

    def update_plot(self):
        pts = self.fk(self.theta1.get(), self.theta2.get(), self.theta3.get())
        xs, ys, zs = [p[0] for p in pts], [p[1] for p in pts], [p[2] for p in pts]

        ax = self.ax_3d; ax.clear()
        if len(self.waypoints) > 1:
            wx,wy,wz = [],[],[]
            for (t1,t2,t3,_) in self.waypoints:
                ep = self.fk(t1,t2,t3)[-1]
                wx.append(ep[0]); wy.append(ep[1]); wz.append(ep[2])
            ax.plot(wx,wy,wz,'--',color='steelblue',alpha=0.4,linewidth=1)
            ax.scatter(wx,wy,wz,c='steelblue',s=15,alpha=0.5)

        ax.plot(xs,ys,zs,'o-', color='royalblue', linewidth=3, markersize=8, markerfacecolor='white', markeredgecolor='royalblue')
        ax.scatter([xs[0]],[ys[0]],[zs[0]], c='red', s=80, zorder=5, label='Base')
        ax.scatter([xs[-1]],[ys[-1]],[zs[-1]], c='green', s=80, zorder=5, label='End Effector')

        limit = (self.L1+self.L2+self.L3)*0.65
        ax.set_xlim([-limit,limit]); ax.set_ylim([-limit,limit]); ax.set_zlim([0, self.L1+self.L2+self.L3])
        ax.set_xlabel('X',fontsize=8); ax.set_ylabel('Y',fontsize=8); ax.set_zlabel('Z',fontsize=8)
        ax.set_title(f"θ1={self.theta1.get():.0f}°  θ2={self.theta2.get():.0f}°  θ3={self.theta3.get():.0f}°", fontsize=8)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
        self.canvas.draw()

        for ax2, hh, vv, title, xl, yl in [
            (self.ax_xy, xs, ys, "Top (X-Y)",  "X","Y"),
            (self.ax_xz, xs, zs, "Side (X-Z)", "X","Z"),
            (self.ax_yz, ys, zs, "Front(Y-Z)", "Y","Z"),
        ]:
            ax2.clear()
            ax2.plot(hh, vv, 'o-', color='royalblue', linewidth=2, markersize=5, markerfacecolor='white', markeredgecolor='royalblue')
            ax2.scatter([hh[-1]],[vv[-1]], c='green', s=40, zorder=5)
            ax2.set_title(title, fontsize=7); ax2.set_xlabel(xl, fontsize=7); ax2.set_ylabel(yl, fontsize=7)
            ax2.tick_params(labelsize=6); ax2.grid(True, alpha=0.3)
        self.fig2.tight_layout(pad=1.2); self.canvas2.draw()

def main():
    root = tk.Tk()
    app = RobotArmController(root)
    root.mainloop()

if __name__ == "__main__":
    main()