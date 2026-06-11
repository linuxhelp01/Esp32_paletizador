import queue
import re
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


BAUD = 115200
AXES = ("X", "Y", "Y1", "Y2", "Z")
MOTOR_AXES = ("X", "Y", "Z")
ROBOT_AXES = ("X", "Y", "Z")
STATUS_RE = re.compile(
    r"^\s*(?P<axis>X|Y1|Y2|Z)\s+id=0x(?P<id>[0-9A-Fa-f]+)\s+"
    r"online=(?P<online>[01])\s+angularEnc=(?P<enc>-?\d+|\?)\s+"
    r"linearMm=(?P<mm>-?\d+(?:\.\d+)?|\?)\s+"
    r"rpm=(?P<rpm>-?\d+|\?)\s+"
    r"acc=(?P<acc>\d+)\s+moveStatus=(?P<move>\d+)\s+"
    r"homeStatus=(?P<home>\d+)\s+last=(?P<last>\d+)"
)


class SerialWorker:
    def __init__(self, inbox):
        self.inbox = inbox
        self.port = None
        self.thread = None
        self.running = False

    def connect(self, port_name):
        if serial is None:
            raise RuntimeError("Falta pyserial. Instala con: python -m pip install pyserial")
        self.disconnect()
        self.port = serial.Serial(port_name, BAUD, timeout=0.1)
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        self.thread = None
        if self.port:
            self.port.close()
        self.port = None

    def send(self, line):
        if not self.port or not self.port.is_open:
            raise RuntimeError("Puerto serial no conectado")
        self.port.write((line.strip() + "\n").encode("utf-8"))

    def _read_loop(self):
        while self.running and self.port and self.port.is_open:
            try:
                line = self.port.readline().decode("utf-8", errors="replace").strip()
                if line:
                    self.inbox.put(line)
            except serial.SerialException as exc:
                self.inbox.put(f"SERIAL ERROR: {exc}")
                self.running = False
            time.sleep(0.01)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Paletizador CAN")
        self.geometry("920x560")
        self.minsize(840, 500)

        self.inbox = queue.Queue()
        self.serial_worker = SerialWorker(self.inbox)

        self.port_var = tk.StringVar()
        self.motor_axis_var = tk.StringVar(value="X")
        self.motor_angular_position_var = tk.StringVar(value="0")
        self.motor_velocity_rpm_var = tk.StringVar(value="500")
        self.motor_accel_rpm_s_var = tk.StringVar(value="1000")
        self.motor_jog_rpm_var = tk.StringVar(value="0")
        self.robot_axis_var = tk.StringVar(value="X")
        self.robot_linear_position_var = tk.StringVar(value="0.000")
        self.robot_linear_velocity_var = tk.StringVar(value="40.0")
        self.robot_linear_accel_var = tk.StringVar(value="200.0")
        self.robot_x_var = tk.StringVar(value="0.000")
        self.robot_y_var = tk.StringVar(value="0.000")
        self.robot_z_var = tk.StringVar(value="0.000")
        self.xyz_linear_velocity_var = tk.StringVar(value="40.0")
        self.xyz_linear_accel_var = tk.StringVar(value="200.0")
        self.auto_status_var = tk.BooleanVar(value=True)
        self.link_status_var = tk.StringVar(value="No conexion")
        self.link_ok = False
        self.link_deadline = 0.0

        self._build()
        self.refresh_ports()
        self.after(50, self.process_inbox)
        self.after(500, self.poll_status)

    def _build(self):
        root = ttk.Frame(self, padding=12)
        root.pack(fill="both", expand=True)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(3, weight=1)
        root.rowconfigure(4, weight=2)

        connection = ttk.Frame(root)
        connection.grid(row=0, column=0, sticky="ew")
        connection.columnconfigure(1, weight=1)

        ttk.Label(connection, text="Puerto").grid(row=0, column=0, padx=(0, 6))
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(0, 6))
        ttk.Button(connection, text="Actualizar", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 6))
        ttk.Button(connection, text="Conectar", command=self.connect).grid(row=0, column=3, padx=(0, 6))
        ttk.Button(connection, text="Desconectar", command=self.disconnect).grid(row=0, column=4, padx=(0, 10))
        ttk.Label(connection, textvariable=self.link_status_var).grid(row=0, column=5, sticky="e")

        motion_tabs = ttk.Notebook(root)
        motion_tabs.grid(row=1, column=0, sticky="ew", pady=(12, 8))

        motor_tab = ttk.Frame(motion_tabs, padding=10)
        axis_tab = ttk.Frame(motion_tabs, padding=10)
        xyz_tab = ttk.Frame(motion_tabs, padding=10)
        motion_tabs.add(motor_tab, text="Prueba 1: angular motor")
        motion_tabs.add(axis_tab, text="Prueba 2: lineal eje")
        motion_tabs.add(xyz_tab, text="Prueba 3: lineal XYZ")

        for col in range(10):
            motor_tab.columnconfigure(col, weight=1)
        ttk.Label(motor_tab, text="Eje/motor").grid(row=0, column=0, sticky="w")
        ttk.Combobox(motor_tab, textvariable=self.motor_axis_var, values=MOTOR_AXES, width=6, state="readonly").grid(row=1, column=0, sticky="ew", padx=(0, 8))
        ttk.Label(motor_tab, text="Angular motor (encoder)").grid(row=0, column=1, sticky="w")
        ttk.Entry(motor_tab, textvariable=self.motor_angular_position_var).grid(row=1, column=1, sticky="ew", padx=(0, 8))
        ttk.Label(motor_tab, text="Vel angular (RPM)").grid(row=0, column=2, sticky="w")
        ttk.Entry(motor_tab, textvariable=self.motor_velocity_rpm_var).grid(row=1, column=2, sticky="ew", padx=(0, 8))
        ttk.Label(motor_tab, text="Acc angular (RPM/s)").grid(row=0, column=3, sticky="w")
        ttk.Entry(motor_tab, textvariable=self.motor_accel_rpm_s_var).grid(row=1, column=3, sticky="ew", padx=(0, 8))
        ttk.Button(motor_tab, text="Enviar angular", command=self.send_motor_angular_position).grid(row=1, column=4, sticky="ew", padx=(0, 8))
        ttk.Label(motor_tab, text="Vel modo giro (RPM)").grid(row=0, column=5, sticky="w")
        ttk.Entry(motor_tab, textvariable=self.motor_jog_rpm_var).grid(row=1, column=5, sticky="ew", padx=(0, 8))
        ttk.Button(motor_tab, text="Enviar VEL", command=self.send_motor_velocity).grid(row=1, column=6, sticky="ew", padx=(0, 8))
        ttk.Button(motor_tab, text="HOME motor", command=self.send_motor_home).grid(row=1, column=7, sticky="ew", padx=(0, 8))
        ttk.Button(motor_tab, text="ZERO motor", command=self.send_motor_zero).grid(row=1, column=8, sticky="ew", padx=(0, 8))
        ttk.Button(motor_tab, text="STOP", command=lambda: self.send("STOP")).grid(row=1, column=9, sticky="ew")

        for col in range(7):
            axis_tab.columnconfigure(col, weight=1)
        ttk.Label(axis_tab, text="Eje robot").grid(row=0, column=0, sticky="w")
        ttk.Combobox(axis_tab, textvariable=self.robot_axis_var, values=ROBOT_AXES, width=6, state="readonly").grid(row=1, column=0, sticky="ew", padx=(0, 8))
        ttk.Label(axis_tab, text="Pos lineal eje (mm)").grid(row=0, column=1, sticky="w")
        ttk.Entry(axis_tab, textvariable=self.robot_linear_position_var).grid(row=1, column=1, sticky="ew", padx=(0, 8))
        ttk.Label(axis_tab, text="Vel lineal eje (mm/s)").grid(row=0, column=2, sticky="w")
        ttk.Entry(axis_tab, textvariable=self.robot_linear_velocity_var).grid(row=1, column=2, sticky="ew", padx=(0, 8))
        ttk.Label(axis_tab, text="Acc lineal eje (mm/s2)").grid(row=0, column=3, sticky="w")
        ttk.Entry(axis_tab, textvariable=self.robot_linear_accel_var).grid(row=1, column=3, sticky="ew", padx=(0, 8))
        ttk.Button(axis_tab, text="Enviar lineal eje", command=self.send_robot_linear_position).grid(row=1, column=4, sticky="ew", padx=(0, 8))
        ttk.Button(axis_tab, text="HOME eje", command=self.send_robot_home).grid(row=1, column=5, sticky="ew", padx=(0, 8))
        ttk.Button(axis_tab, text="ZERO eje", command=self.send_robot_zero).grid(row=1, column=6, sticky="ew")

        for col in range(6):
            xyz_tab.columnconfigure(col, weight=1)
        ttk.Label(xyz_tab, text="X extremo (mm)").grid(row=0, column=0, sticky="w")
        ttk.Entry(xyz_tab, textvariable=self.robot_x_var).grid(row=1, column=0, sticky="ew", padx=(0, 8))
        ttk.Label(xyz_tab, text="Y extremo (mm)").grid(row=0, column=1, sticky="w")
        ttk.Entry(xyz_tab, textvariable=self.robot_y_var).grid(row=1, column=1, sticky="ew", padx=(0, 8))
        ttk.Label(xyz_tab, text="Z extremo (mm)").grid(row=0, column=2, sticky="w")
        ttk.Entry(xyz_tab, textvariable=self.robot_z_var).grid(row=1, column=2, sticky="ew", padx=(0, 8))
        ttk.Label(xyz_tab, text="Vel lineal XYZ (mm/s)").grid(row=0, column=3, sticky="w")
        ttk.Entry(xyz_tab, textvariable=self.xyz_linear_velocity_var).grid(row=1, column=3, sticky="ew", padx=(0, 8))
        ttk.Label(xyz_tab, text="Acc lineal XYZ (mm/s2)").grid(row=0, column=4, sticky="w")
        ttk.Entry(xyz_tab, textvariable=self.xyz_linear_accel_var).grid(row=1, column=4, sticky="ew", padx=(0, 8))
        ttk.Button(xyz_tab, text="Enviar XYZ coordinado", command=self.send_robot_xyz).grid(row=1, column=5, sticky="ew")

        quick = ttk.Frame(root)
        quick.grid(row=2, column=0, sticky="ew", pady=(0, 8))
        ttk.Button(quick, text="STATUS", command=lambda: self.send("STATUS")).pack(side="left")
        ttk.Button(quick, text="PING", command=lambda: self.send("PING")).pack(side="left", padx=(8, 0))
        ttk.Button(quick, text="HELP", command=lambda: self.send("HELP")).pack(side="left", padx=(8, 0))
        ttk.Button(quick, text="STOP", command=lambda: self.send("STOP")).pack(side="left", padx=(8, 0))
        ttk.Checkbutton(quick, text="Actualizar estado", variable=self.auto_status_var).pack(side="left", padx=(16, 0))

        columns = ("axis", "id", "online", "enc", "mm", "rpm", "acc", "move", "home", "last")
        self.status_table = ttk.Treeview(root, columns=columns, show="headings", height=5)
        self.status_table.grid(row=3, column=0, sticky="nsew", pady=(0, 8))

        headings = {
            "axis": "Eje",
            "id": "CAN ID",
            "online": "Conectado",
            "enc": "Angular enc",
            "mm": "Lineal mm",
            "rpm": "Velocidad",
            "acc": "Aceleracion",
            "move": "Mov.",
            "home": "Home",
            "last": "Ultima ms",
        }
        widths = {
            "axis": 60,
            "id": 80,
            "online": 90,
            "enc": 140,
            "mm": 100,
            "rpm": 100,
            "acc": 100,
            "move": 70,
            "home": 70,
            "last": 100,
        }
        for column in columns:
            self.status_table.heading(column, text=headings[column])
            self.status_table.column(column, width=widths[column], anchor="center")

        for axis in ("X", "Y1", "Y2", "Z"):
            self.status_table.insert("", "end", iid=axis, values=(axis, "-", "NO", "-", "-", "-", "-", "-", "-", "-"))

        self.log = tk.Text(root, height=18, wrap="word")
        self.log.grid(row=4, column=0, sticky="nsew")
        self.log.configure(state="disabled")

    def refresh_ports(self):
        if list_ports is None:
            self.port_combo["values"] = []
            return
        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def connect(self):
        try:
            self.serial_worker.connect(self.port_var.get())
            self.link_ok = False
            self.link_deadline = time.monotonic() + 3.0
            self.link_status_var.set("Verificando enlace...")
            self.append_log(f"Conectado a {self.port_var.get()} @ {BAUD}")
            self.send("HELLO", log=False)
        except Exception as exc:
            self.link_ok = False
            self.link_status_var.set("No conexion")
            messagebox.showerror("Conexion serial", str(exc))

    def disconnect(self):
        self.serial_worker.disconnect()
        self.link_ok = False
        self.link_status_var.set("No conexion")
        self.append_log("Desconectado")

    def send_motor_angular_position(self):
        self.send(
            f"POSANG {self.motor_axis_var.get()} {self.motor_angular_position_var.get()} "
            f"{self.motor_velocity_rpm_var.get()} {self.motor_accel_rpm_s_var.get()}"
        )

    def send_robot_linear_position(self):
        self.send(
            f"POSLINE {self.robot_axis_var.get()} {self.robot_linear_position_var.get()} "
            f"{self.robot_linear_velocity_var.get()} {self.robot_linear_accel_var.get()}"
        )

    def send_motor_velocity(self):
        self.send(
            f"VELANG {self.motor_axis_var.get()} {self.motor_jog_rpm_var.get()} "
            f"{self.motor_accel_rpm_s_var.get()}"
        )

    def send_robot_xyz(self):
        self.send(
            f"POSXYZ {self.robot_x_var.get()} {self.robot_y_var.get()} {self.robot_z_var.get()} "
            f"{self.xyz_linear_velocity_var.get()} {self.xyz_linear_accel_var.get()}"
        )

    def send_robot_home(self):
        self.send(f"HOME {self.robot_axis_var.get()}")

    def send_motor_home(self):
        self.send(f"HOME {self.motor_axis_var.get()}")

    def send_motor_zero(self):
        self.send(f"ZERO {self.motor_axis_var.get()}")

    def send_robot_zero(self):
        self.send(f"ZERO {self.robot_axis_var.get()}")

    def send(self, line, log=True):
        try:
            self.serial_worker.send(line)
            if log:
                self.append_log(f"> {line}")
        except Exception as exc:
            if log:
                messagebox.showwarning("Envio serial", str(exc))

    def process_inbox(self):
        while True:
            try:
                line = self.inbox.get_nowait()
            except queue.Empty:
                break
            if line == "PALLETIZER_LINK_OK":
                self.link_ok = True
                self.link_status_var.set("Enlace exitoso")
                self.send("STATUS", log=False)
                continue
            if not self.update_status_table(line):
                if line != "STATUS":
                    self.append_log(line)
        if (
            self.serial_worker.port
            and self.serial_worker.port.is_open
            and not self.link_ok
            and self.link_deadline
            and time.monotonic() > self.link_deadline
        ):
            self.link_status_var.set("No conexion")
            self.link_deadline = 0.0
        self.after(50, self.process_inbox)

    def poll_status(self):
        if self.auto_status_var.get() and self.link_ok and self.serial_worker.port and self.serial_worker.port.is_open:
            self.send("STATUS", log=False)
        self.after(500, self.poll_status)

    def update_status_table(self, line):
        match = STATUS_RE.match(line)
        if not match:
            return False

        axis = match.group("axis")
        values = (
            axis,
            f"0x{match.group('id').upper()}",
            "SI" if match.group("online") == "1" else "NO",
            match.group("enc"),
            match.group("mm"),
            match.group("rpm"),
            match.group("acc"),
            match.group("move"),
            match.group("home"),
            match.group("last"),
        )
        self.status_table.item(axis, values=values)
        return True

    def append_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def destroy(self):
        self.serial_worker.disconnect()
        super().destroy()


if __name__ == "__main__":
    App().mainloop()
