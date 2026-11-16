import math
import threading
import time
import tkinter as tk
from tkinter import messagebox

from dronekit import connect, VehicleMode

try:
    import tkintermapview
except ImportError:
    tkintermapview = None

MODES = ["GUIDED", "STABILIZE", "LOITER", "LAND", "RTL"]

# RC override settings (ArduCopter: 1=roll, 2=pitch, 3=throttle, 4=yaw)
RC_CENTER = 1500
RC_DELTA = 300          # how far a key pushes the "stick" (1200..1800)
RC_RATE_MS = 100        # send RC values 10 times per second
BOX = 120               # size of each stick box on the canvas

# Map settings
SITL_LAT, SITL_LON = -35.363261, 149.165230   # default SITL start location
HEADING_LINE_M = 30     # length of the heading line on the map, in meters
MAP_UPDATE_MS = 500


class GCS:
    def __init__(self, root):
        self.root = root
        self.root.title("My GCS (DroneKit)")
        self.vehicle = None

        self.keys = set()           # keys currently held down
        self._release_jobs = {}     # pending key releases (fixes Linux key auto-repeat)
        self.rc = {1: RC_CENTER, 2: RC_CENTER, 3: RC_CENTER, 4: RC_CENTER}

        # Layout: controls on the left, map on the right
        main = tk.Frame(root)
        main.pack(fill="both", expand=True)
        left = tk.Frame(main)
        left.pack(side="left", fill="y")
        right = tk.Frame(main)
        right.pack(side="left", fill="both", expand=True, padx=(0, 10), pady=10)

        # ---------- Connection row ----------
        top = tk.Frame(left)
        top.pack(padx=10, pady=10, fill="x")
        tk.Label(top, text="Connection string:").pack(side="left")
        self.conn_entry = tk.Entry(top, width=25)
        self.conn_entry.insert(0, "127.0.0.1:14550")
        self.conn_entry.pack(side="left", padx=5)
        self.connect_btn = tk.Button(top, text="Connect", command=self.connect)
        self.connect_btn.pack(side="left")
        tk.Button(top, text="Close Connection", command=self.close).pack(side="left", padx=5)

        # ---------- Status panel ----------
        st = tk.LabelFrame(left, text="Status")
        st.pack(padx=10, pady=5, fill="x")
        self.lbl_conn = tk.Label(st, text="Connection: NOT CONNECTED", fg="red", anchor="w")
        self.lbl_mode = tk.Label(st, text="Mode: -", anchor="w")
        self.lbl_armed = tk.Label(st, text="Armed: -", anchor="w")
        self.lbl_alt = tk.Label(st, text="Altitude: - m", anchor="w")
        for lbl in (self.lbl_conn, self.lbl_mode, self.lbl_armed, self.lbl_alt):
            lbl.pack(fill="x", padx=5)

        # ---------- Flight modes ----------
        modes = tk.LabelFrame(left, text="Flight mode")
        modes.pack(padx=10, pady=5, fill="x")
        for name in MODES:
            label = "RTL" if name == "RTL" else name.capitalize()
            tk.Button(modes, text=label, width=10,
                      command=lambda n=name: self.set_mode(n)).pack(side="left", padx=3, pady=5)

        # ---------- Arm / Takeoff ----------
        act = tk.LabelFrame(left, text="Actions")
        act.pack(padx=10, pady=5, fill="x")
        tk.Button(act, text="Arm", width=10, command=self.arm).pack(side="left", padx=3, pady=5)
        tk.Label(act, text="Altitude (m):").pack(side="left", padx=(15, 2))
        self.alt_entry = tk.Entry(act, width=6)
        self.alt_entry.insert(0, "10")
        self.alt_entry.pack(side="left")
        tk.Button(act, text="Takeoff", width=10, command=self.takeoff).pack(side="left", padx=5)

        # ---------- Keyboard control + RC display ----------
        rcf = tk.LabelFrame(left, text="RC control (keyboard)")
        rcf.pack(padx=10, pady=5, fill="x")
        self.kb_var = tk.BooleanVar(value=False)
        tk.Checkbutton(
            rcf, text="Enable keyboard control (use Loiter mode)",
            variable=self.kb_var, command=self.toggle_keyboard
        ).pack(anchor="w", padx=5)
        tk.Label(
            rcf, anchor="w", justify="left",
            text="W/S = throttle   A/D = yaw   Left/Right = roll   Up/Down = pitch"
        ).pack(anchor="w", padx=5)

        self.canvas = tk.Canvas(rcf, width=2 * BOX + 60, height=BOX + 50)
        self.canvas.pack(pady=5)
        self.lbl_rc = tk.Label(rcf, text="")
        self.lbl_rc.pack()

        self.status = tk.Label(left, text="Not connected", anchor="w")
        self.status.pack(fill="x", padx=10, pady=(0, 10))

        # ---------- Map ----------
        self.map_widget = None
        self.marker = None
        self.heading_line = None
        self.trail = []
        self.trail_path = None

        if tkintermapview is None:
            tk.Label(
                right, justify="left",
                text="Map unavailable.\nInstall it inside the container with:\n\n"
                     "pip3 install tkintermapview\n\nthen restart gcs.py"
            ).pack(padx=10, pady=10)
        else:
            maptop = tk.Frame(right)
            maptop.pack(fill="x")
            self.follow_var = tk.BooleanVar(value=True)
            tk.Checkbutton(maptop, text="Follow drone", variable=self.follow_var).pack(side="left")
            tk.Button(maptop, text="Clear trail", command=self.clear_trail).pack(side="left", padx=5)
            self.lbl_pos = tk.Label(maptop, text="Position: -", anchor="w")
            self.lbl_pos.pack(side="left", padx=10)

            self.map_widget = tkintermapview.TkinterMapView(
                right, width=520, height=560, corner_radius=0
            )
            self.map_widget.pack(fill="both", expand=True)
            self.map_widget.set_position(SITL_LAT, SITL_LON)
            self.map_widget.set_zoom(17)

        # Keyboard bindings (ignored while typing in a text field)
        root.bind_all("<KeyPress>", self.on_key_press)
        root.bind_all("<KeyRelease>", self.on_key_release)

        self.draw_rc()
        self.update_status()
        self.send_rc()
        self.update_map()

    # ================= helpers =================
    def _require_connection(self):
        if self.vehicle is None:
            messagebox.showwarning("Not connected", "Connect to the vehicle first.")
            return False
        return True

    def set_status(self, text):
        self.status.config(text=text)

    # ================= connection =================
    def connect(self):
        conn_str = self.conn_entry.get().strip()
        if not conn_str:
            messagebox.showerror("Error", "Enter a connection string.")
            return
        self.set_status("Connecting...")
        self.connect_btn.config(state="disabled")
        threading.Thread(target=self._connect_worker, args=(conn_str,), daemon=True).start()

    def _connect_worker(self, conn_str):
        try:
            vehicle = connect(conn_str, wait_ready=["mode", "armed"],
                              timeout=120, heartbeat_timeout=60)
            self.root.after(0, self._on_connected, vehicle)
        except Exception as e:
            self.root.after(0, self._on_connect_failed, str(e))

    def _on_connected(self, vehicle):
        self.vehicle = vehicle
        self.connect_btn.config(state="normal")
        self.set_status("Connected")

    def _on_connect_failed(self, error):
        self.vehicle = None
        self.connect_btn.config(state="normal")
        self.set_status("Not connected")
        messagebox.showerror("Connection failed", error)

    def close(self):
        if self.vehicle is not None:
            try:
                self.vehicle.channels.overrides = {}
            except Exception:
                pass
            self.vehicle.close()
            self.vehicle = None
        self.kb_var.set(False)
        self.clear_map()
        self.set_status("Connection closed")

    # ================= status panel =================
    def update_status(self):
        v = self.vehicle
        if v is None:
            self.lbl_conn.config(text="Connection: NOT CONNECTED", fg="red")
            self.lbl_mode.config(text="Mode: -")
            self.lbl_armed.config(text="Armed: -")
            self.lbl_alt.config(text="Altitude: - m")
        else:
            try:
                hb = v.last_heartbeat
                alive = hb is not None and hb < 3
                self.lbl_conn.config(
                    text="Connection: CONNECTED" if alive else "Connection: NO HEARTBEAT",
                    fg="green" if alive else "orange",
                )
                self.lbl_mode.config(text=f"Mode: {v.mode.name}")
                self.lbl_armed.config(text="Armed: YES" if v.armed else "Armed: NO")
                alt = v.location.global_relative_frame.alt
                self.lbl_alt.config(
                    text="Altitude: - m" if alt is None else f"Altitude: {alt:.1f} m"
                )
            except Exception:
                pass
        self.root.after(200, self.update_status)

    # ================= modes / arm =================
    def set_mode(self, name):
        if not self._require_connection():
            return
        self.vehicle.mode = VehicleMode(name)
        self.set_status(f"Mode command sent: {name}")

    def arm(self):
        if not self._require_connection():
            return
        self.vehicle.armed = True
        self.set_status("Arm command sent")

    # ================= takeoff =================
    def takeoff(self):
        if not self._require_connection():
            return
        try:
            alt = float(self.alt_entry.get())
            if alt <= 0 or alt > 100:
                raise ValueError
        except ValueError:
            messagebox.showerror("Error", "Enter an altitude between 1 and 100 meters.")
            return
        self.kb_var.set(False)
        self.toggle_keyboard()
        self.root.focus_set()
        self.set_status(f"Takeoff to {alt:.0f} m started...")
        threading.Thread(target=self._takeoff_worker, args=(alt,), daemon=True).start()

    def _takeoff_worker(self, alt):
        v = self.vehicle
        try:
            v.mode = VehicleMode("GUIDED")
            t0 = time.time()
            while v.mode.name != "GUIDED" and time.time() - t0 < 10:
                time.sleep(0.5)
            t0 = time.time()
            while not v.is_armable and time.time() - t0 < 60:
                self.root.after(0, self.set_status, "Waiting for vehicle to be ready...")
                time.sleep(1)
            v.armed = True
            t0 = time.time()
            while not v.armed and time.time() - t0 < 15:
                time.sleep(0.5)
            if not v.armed:
                self.root.after(0, self.set_status, "Takeoff failed: could not arm")
                return
            v.simple_takeoff(alt)
            self.root.after(0, self.set_status, f"Taking off to {alt:.0f} m")
        except Exception as e:
            self.root.after(0, self.set_status, f"Takeoff error: {e}")

    # ================= keyboard / RC =================
    def toggle_keyboard(self):
        if not self.kb_var.get():
            self.keys.clear()
            if self.vehicle is not None:
                try:
                    self.vehicle.channels.overrides = {}   # give control back
                except Exception:
                    pass
            self.rc = {1: RC_CENTER, 2: RC_CENTER, 3: RC_CENTER, 4: RC_CENTER}
            self.draw_rc()
        else:
            self.root.focus_set()

    def on_key_press(self, event):
        if isinstance(event.widget, tk.Entry):
            return
        key = event.keysym.lower()
        job = self._release_jobs.pop(key, None)
        if job:
            self.root.after_cancel(job)       # auto-repeat: cancel the fake release
        self.keys.add(key)

    def on_key_release(self, event):
        key = event.keysym.lower()
        # Wait a moment: on Linux, holding a key sends release+press repeatedly
        self._release_jobs[key] = self.root.after(30, self._really_release, key)

    def _really_release(self, key):
        self._release_jobs.pop(key, None)
        self.keys.discard(key)

    def compute_rc(self):
        k = self.keys
        roll = RC_CENTER + RC_DELTA * (("right" in k) - ("left" in k))
        # Pitch: stick forward = LOWER value in ArduPilot, so Up arrow = forward
        pitch = RC_CENTER + RC_DELTA * (("down" in k) - ("up" in k))
        throttle = RC_CENTER + RC_DELTA * (("w" in k) - ("s" in k))
        yaw = RC_CENTER + RC_DELTA * (("d" in k) - ("a" in k))
        return {1: roll, 2: pitch, 3: throttle, 4: yaw}

    def send_rc(self):
        if self.kb_var.get() and self.vehicle is not None:
            self.rc = self.compute_rc()
            try:
                for ch, val in self.rc.items():
                    self.vehicle.channels.overrides[str(ch)] = val
            except Exception as e:
                self.set_status(f"RC error: {e}")
            self.draw_rc()
        self.root.after(RC_RATE_MS, self.send_rc)

    # ================= RC stick display =================
    def draw_rc(self):
        c = self.canvas
        c.delete("all")
        pad = 20
        # Left box: yaw (x) and throttle (y).  Right box: roll (x) and pitch (y).
        boxes = [
            (pad, "Throttle / Yaw", self.rc[4], self.rc[3], False),
            (2 * pad + BOX, "Pitch / Roll", self.rc[1], self.rc[2], True),
        ]
        for x0, title, xv, yv, invert_y in boxes:
            y0 = 20
            c.create_rectangle(x0, y0, x0 + BOX, y0 + BOX, outline="gray40", width=2)
            c.create_line(x0 + BOX / 2, y0, x0 + BOX / 2, y0 + BOX, fill="gray70")
            c.create_line(x0, y0 + BOX / 2, x0 + BOX, y0 + BOX / 2, fill="gray70")
            c.create_text(x0 + BOX / 2, 10, text=title)
            fx = (xv - 1000) / 1000.0
            fy = (yv - 1000) / 1000.0
            px = x0 + fx * BOX
            # throttle: higher PWM = up. pitch: lower PWM = forward = up on screen
            py = y0 + (fy if invert_y else (1 - fy)) * BOX
            c.create_oval(px - 6, py - 6, px + 6, py + 6, fill="red", outline="black")
        self.lbl_rc.config(
            text=f"CH1 roll: {self.rc[1]}   CH2 pitch: {self.rc[2]}   "
                 f"CH3 thr: {self.rc[3]}   CH4 yaw: {self.rc[4]}"
        )

    # ================= map =================
    @staticmethod
    def point_ahead(lat, lon, heading_deg, dist_m):
        """Point dist_m meters away from (lat, lon) in the heading direction (0 = north)."""
        h = math.radians(heading_deg)
        dlat = dist_m * math.cos(h) / 111320.0
        dlon = dist_m * math.sin(h) / (111320.0 * max(math.cos(math.radians(lat)), 0.01))
        return (lat + dlat, lon + dlon)

    @staticmethod
    def distance_m(lat1, lon1, lat2, lon2):
        dlat = (lat2 - lat1) * 111320.0
        dlon = (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1))
        return math.hypot(dlat, dlon)

    def update_map(self):
        v = self.vehicle
        if self.map_widget is not None and v is not None:
            try:
                loc = v.location.global_frame
                if loc is not None and loc.lat is not None and loc.lon is not None \
                        and not (loc.lat == 0 and loc.lon == 0):
                    self.draw_drone(loc.lat, loc.lon, v.heading)
            except Exception:
                pass
        self.root.after(MAP_UPDATE_MS, self.update_map)

    def draw_drone(self, lat, lon, heading):
        label = "Drone" if heading is None else f"Drone  {heading}\N{DEGREE SIGN}"
        if self.marker is None:
            self.marker = self.map_widget.set_marker(lat, lon, text=label)
        else:
            self.marker.set_position(lat, lon)
            self.marker.set_text(label)

        # Heading (yaw) direction: red line pointing the way the drone faces
        if heading is not None:
            ahead = self.point_ahead(lat, lon, heading, HEADING_LINE_M)
            if self.heading_line is None:
                self.heading_line = self.map_widget.set_path(
                    [(lat, lon), ahead], color="red", width=4
                )
            else:
                self.heading_line.set_position_list([(lat, lon), ahead])

        # Trail: blue line of where the drone has been
        if not self.trail or self.distance_m(*self.trail[-1], lat, lon) > 1.5:
            self.trail.append((lat, lon))
            if len(self.trail) == 2:
                self.trail_path = self.map_widget.set_path(self.trail, color="blue", width=2)
            elif len(self.trail) > 2 and self.trail_path is not None:
                self.trail_path.add_position(lat, lon)

        if self.follow_var.get():
            self.map_widget.set_position(lat, lon)
        self.lbl_pos.config(text=f"Position: {lat:.6f}, {lon:.6f}")

    def clear_trail(self):
        if self.trail_path is not None:
            self.trail_path.delete()
            self.trail_path = None
        self.trail = []

    def clear_map(self):
        if self.map_widget is None:
            return
        self.clear_trail()
        if self.marker is not None:
            self.marker.delete()
            self.marker = None
        if self.heading_line is not None:
            self.heading_line.delete()
            self.heading_line = None
        self.lbl_pos.config(text="Position: -")


if __name__ == "__main__":
    root = tk.Tk()
    GCS(root)
    root.mainloop()
