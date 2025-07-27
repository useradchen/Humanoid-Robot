import tkinter as tk
from tkinter import messagebox
import serial
import time

class RobotControlUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control UI")
        self.serial_port = None
        self.baud_rate = 115200

        # 設置較大的字體
        self.font = ("Arial", 14)

        # 序列埠選擇
        tk.Label(root, text="Serial Port (e.g., COM3 or /dev/ttyUSB0):", font=self.font).grid(row=0, column=0, padx=5, pady=5)
        self.port_entry = tk.Entry(root, font=self.font)
        self.port_entry.grid(row=0, column=1, padx=5, pady=5)
        self.port_entry.insert(0, "COM3")  # 預設值，可根據您的系統修改
        tk.Button(root, text="Connect", command=self.connect_serial, font=self.font).grid(row=0, column=2, padx=5, pady=5)

        # 功能 1：設定初始 YZ 和速度
        tk.Label(root, text="Initial Y Pos:", font=self.font).grid(row=1, column=0, padx=5, pady=5)
        self.initial_y_entry = tk.Entry(root, font=self.font)
        self.initial_y_entry.grid(row=1, column=1, padx=5, pady=5)
        self.initial_y_entry.insert(0, "0.0")
        self.initial_y_entry.bind("<MouseWheel>", lambda e: self._adjust_value(e, self.initial_y_entry, 0.1))

        tk.Label(root, text="Initial Z Pos:", font=self.font).grid(row=2, column=0, padx=5, pady=5)
        self.initial_z_entry = tk.Entry(root, font=self.font)
        self.initial_z_entry.grid(row=2, column=1, padx=5, pady=5)
        self.initial_z_entry.insert(0, "50.0")
        self.initial_z_entry.bind("<MouseWheel>", lambda e: self._adjust_value(e, self.initial_z_entry, 0.1))

        tk.Label(root, text="Steps:", font=self.font).grid(row=3, column=0, padx=5, pady=5)
        self.steps_entry = tk.Entry(root, font=self.font)
        self.steps_entry.grid(row=3, column=1, padx=5, pady=5)
        self.steps_entry.insert(0, "300")
        self.steps_entry.bind("<MouseWheel>", lambda e: self._adjust_value(e, self.steps_entry, 1))

        tk.Button(root, text="Set Initial Parameters", command=self.set_initial_params, font=self.font).grid(row=4, column=0, columnspan=3, pady=10)

        # 功能 2：傳送 XYZ 和 Theta
        tk.Label(root, text="X:", font=self.font).grid(row=5, column=0, padx=5, pady=5)
        self.x_entry = tk.Entry(root, font=self.font)
        self.x_entry.grid(row=5, column=1, padx=5, pady=5)
        self.x_entry.insert(0, "0.0")
        self.x_entry.bind("<MouseWheel>", lambda e: self._adjust_value(e, self.x_entry, 0.1))

        tk.Label(root, text="Y:", font=self.font).grid(row=6, column=0, padx=5, pady=5)
        self.y_entry = tk.Entry(root, font=self.font)
        self.y_entry.grid(row=6, column=1, padx=5, pady=5)
        self.y_entry.insert(0, "0.0")
        self.y_entry.bind("<MouseWheel>", lambda e: self._adjust_value(e, self.y_entry, 0.1))

        tk.Label(root, text="Z:", font=self.font).grid(row=7, column=0, padx=5, pady=5)
        self.z_entry = tk.Entry(root, font=self.font)
        self.z_entry.grid(row=7, column=1, padx=5, pady=5)
        self.z_entry.insert(0, "0.0")
        self.z_entry.bind("<MouseWheel>", lambda e: self._adjust_value(e, self.z_entry, 0.1))

        tk.Label(root, text="Theta:", font=self.font).grid(row=8, column=0, padx=5, pady=5)
        self.theta_entry = tk.Entry(root, font=self.font)
        self.theta_entry.grid(row=8, column=1, padx=5, pady=5)
        self.theta_entry.insert(0, "0.0")
        self.theta_entry.bind("<MouseWheel>", lambda e: self._adjust_value(e, self.theta_entry, 0.1))

        tk.Button(root, text="Send XYZ and Theta", command=self.send_xyz_theta, font=self.font).grid(row=9, column=0, columnspan=3, pady=10)

        # 功能 3：停止按鈕
        tk.Button(root, text="Stop Robot", command=self.stop_robot, bg="red", fg="white", font=self.font).grid(row=10, column=0, columnspan=3, pady=10)

    def _adjust_value(self, event, entry, step):
        """處理滑鼠滾輪事件，調整輸入欄位值"""
        value = float(entry.get()) if entry.get() else 0.0
        if event.delta > 0:  # 滾輪向上，增加值
            value += step
        else:  # 滾輪向下，減少值
            value -= step
        entry.delete(0, tk.END)
        entry.insert(0, round(value, 2))  # 保留兩位小數

    def connect_serial(self):
        port = self.port_entry.get()
        try:
            self.serial_port = serial.Serial(port, self.baud_rate, timeout=1)
            messagebox.showinfo("Success", f"Connected to {port}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to connect to {port}: {str(e)}")

    def set_initial_params(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showerror("Error", "Serial port is not connected.")
            return
        try:
            initial_y = float(self.initial_y_entry.get())
            initial_z = float(self.initial_z_entry.get())
            steps = int(self.steps_entry.get())
            command = f"set_yz:{initial_y},{initial_z},{steps}\n"
            self.serial_port.write(command.encode())
            messagebox.showinfo("Success", "Initial parameters sent successfully.")
        except ValueError:
            messagebox.showerror("Error", "Invalid input for initial parameters.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send parameters: {str(e)}")

    def send_xyz_theta(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showerror("Error", "Serial port is not connected.")
            return
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            z = float(self.z_entry.get())
            theta = float(self.theta_entry.get())
            command = f"{x},{y},{z},{theta}\n"
            self.serial_port.write(command.encode())
            messagebox.showinfo("Success", "XYZ and Theta sent successfully.")
        except ValueError:
            messagebox.showerror("Error", "Invalid input for XYZ or Theta.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send XYZ and Theta: {str(e)}")

    def stop_robot(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showerror("Error", "Serial port is not connected.")
            return
        try:
            self.serial_port.write(b"q\n")
            messagebox.showinfo("Success", "Stop command sent successfully.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send stop command: {str(e)}")

    def close(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControlUI(root)
    root.protocol("WM_DELETE_WINDOW", lambda: [app.close(), root.destroy()])
    root.mainloop()