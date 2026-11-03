import threading
import tkinter as tk
from tkinter import messagebox

from dronekit import connect, VehicleMode

MODES = ["GUIDED", "STABILIZE", "LOITER", "LAND", "RTL"]


class GCS:
    def __init__(self, root):
        self.root = root
        self.root.title("My GCS (DroneKit)")
        self.vehicle = None

        # Connection row
        top = tk.Frame(root)
        top.pack(padx=10, pady=10, fill="x")

        tk.Label(top, text="Connection string:").pack(side="left")
        self.conn_entry = tk.Entry(top, width=30)
        self.conn_entry.insert(0, "127.0.0.1:14550")
        self.conn_entry.pack(side="left", padx=5)

        self.connect_btn = tk.Button(top, text="Connect", command=self.connect)
        self.connect_btn.pack(side="left")

        # Mode buttons
        modes = tk.LabelFrame(root, text="Flight mode")
        modes.pack(padx=10, pady=5, fill="x")
        for name in MODES:
            tk.Button(
                modes, text=name.capitalize(), width=10,
                command=lambda n=name: self.set_mode(n)
            ).pack(side="left", padx=3, pady=5)

        # Arm / Close
        bottom = tk.Frame(root)
        bottom.pack(padx=10, pady=10, fill="x")
        tk.Button(bottom, text="Arm", width=12, command=self.arm).pack(side="left", padx=3)
        tk.Button(bottom, text="Close Connection", command=self.close).pack(side="left", padx=3)

        self.status = tk.Label(root, text="Not connected", anchor="w")
        self.status.pack(fill="x", padx=10, pady=(0, 10))

    def _require_connection(self):
        if self.vehicle is None:
            messagebox.showwarning("Not connected", "Connect to the vehicle first.")
            return False
        return True

    def connect(self):
        conn_str = self.conn_entry.get().strip()
        if not conn_str:
            messagebox.showerror("Error", "Enter a connection string.")
            return
        self.status.config(text="Connecting...")
        self.connect_btn.config(state="disabled")
        # connect() blocks, so run it in a thread to keep the window responsive
        threading.Thread(target=self._connect_worker, args=(conn_str,), daemon=True).start()

    def _connect_worker(self, conn_str):
        try:
            vehicle = connect(conn_str, wait_ready=False, timeout=30)
            self.root.after(0, self._on_connected, vehicle)
        except Exception as e:
            self.root.after(0, self._on_connect_failed, str(e))

    def _on_connected(self, vehicle):
        self.vehicle = vehicle
        self.connect_btn.config(state="normal")
        self.status.config(text=f"Connected (mode: {vehicle.mode.name})")

    def _on_connect_failed(self, error):
        self.vehicle = None
        self.connect_btn.config(state="normal")
        self.status.config(text="Not connected")
        messagebox.showerror("Connection failed", error)

    def set_mode(self, name):
        if not self._require_connection():
            return
        self.vehicle.mode = VehicleMode(name)
        self.status.config(text=f"Mode command sent: {name}")

    def arm(self):
        if not self._require_connection():
            return
        self.vehicle.armed = True
        self.status.config(text="Arm command sent")

    def close(self):
        if self.vehicle is not None:
            self.vehicle.close()
            self.vehicle = None
        self.status.config(text="Connection closed")


if __name__ == "__main__":
    root = tk.Tk()
    GCS(root)
    root.mainloop()
