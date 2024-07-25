import customtkinter as ctk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from scipy.spatial.transform import Rotation as R
from collections import deque
import time
import serial
import threading
import serial.tools.list_ports

class RealTimeDataApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Telemetry Data GUI")
        self.geometry("1200x800")
        self.iconbitmap("icon.ico")

        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # Create a sidebar for controls
        self.sidebar_frame = ctk.CTkFrame(self, width=200, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, rowspan=4, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(4, weight=1)
        self.logo_label = ctk.CTkLabel(self.sidebar_frame, text="TELEMTERY\nDATA", 
                                       font=ctk.CTkFont(size=25, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))

        # Add the COM port selection
        self.COMport_label = ctk.CTkLabel(self.sidebar_frame, text="COM Port:")
        self.COMport_label.grid(row=1, column=0, padx=10, pady=(10,0))

        self.COM_ports = self.get_com_ports()
        self.COM_var = ctk.StringVar(value=self.COM_ports[0] if self.COM_ports else "No COM Ports")
        self.COM_port_optionemenu = ctk.CTkOptionMenu(self.sidebar_frame, variable=self.COM_var, values=self.COM_ports, command=self.change_port)
        self.COM_port_optionemenu.grid(row=2, column=0, pady=(5, 30))

        self.serial_port = None
        self.reading_thread = None
        self.reading_active = threading.Event()
        self.selected_port = self.COM_var.get()
        
        # Initialize deque for data
        self.data_queue = deque(maxlen=100)
        self.updating = False
        
        # Add a button to start/stop reading data
        self.start_button = ctk.CTkButton(self.sidebar_frame, text="Start", command=self.toggle_update)
        self.start_button.grid(row=3, column=0, padx=20, pady=10)

        # Create a canvas for the plot
        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.figure, master=self)
        self.canvas.get_tk_widget().grid(row=0, column=1, sticky="nsew")
        
        # Set plot limits and labels
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_xticklabels([])
        self.ax.set_yticklabels([])
        self.ax.set_zticklabels([])

        self.connect_serial()
        
    def get_com_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def change_port(self, selection):
        self.selected_port = selection

    def fetch_data(self):
        while self.reading_active.is_set():
            if self.serial_port:
                line = self.serial_port.readline().decode('utf-8').strip()
                data = line.split(',')
                if len(data) == 4:  # Assuming data format is correct
                    q0, q1, q2, q3 = map(float, data)
                    #q0 = data[0]
                    #q1 = data[1]
                    #q2 = data[2]
                    #q3 = data[3]
                    self.data_queue.append((q0, q1, q2, q3))
                    print(data)
                    self.update_plot()  # Update plot with new data
            #time.sleep(0.2)  # Adjust the sleep time based on data rate

    def update_plot(self):
        if not self.data_queue:
            return

        q0, q1, q2, q3 = self.data_queue[-1]
        self.ax.clear()

        quaternion = np.array([q0, q1, q2, q3])
        quaternion = quaternion / np.linalg.norm(quaternion)
        rotation_matrix = R.from_quat(quaternion).as_matrix()
        axes = np.eye(3)
        rotated_axes = axes @ rotation_matrix.T

        self.ax.quiver(0, 0, 0, rotated_axes[0, 0], rotated_axes[0, 1], rotated_axes[0, 2], color='r', length=1, linewidth=5, arrow_length_ratio=0.1, label='X-axis')
        self.ax.quiver(0, 0, 0, rotated_axes[1, 0], rotated_axes[1, 1], rotated_axes[1, 2], color='g', length=1, linewidth=5, arrow_length_ratio=0.1, label='Y-axis')
        self.ax.quiver(0, 0, 0, rotated_axes[2, 0], rotated_axes[2, 1], rotated_axes[2, 2], color='b', length=1, linewidth=5, arrow_length_ratio=0.1, label='Z-axis')
        self.ax.quiver(0, 0, 0, 1, 0, 0, color='c', length=0.5, linewidth=5, arrow_length_ratio=0.1, label='North')
        self.ax.quiver(0, 0, 0, 0, 1, 0, color='y', length=0.5, linewidth=5, arrow_length_ratio=0.1, label='East')
        self.ax.quiver(0, 0, 0, 0, 0, 1, color='m', length=0.5, linewidth=5, arrow_length_ratio=0.1, label='Down')

        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.legend()
        self.ax.set_xticklabels([])
        self.ax.set_yticklabels([])
        self.ax.set_zticklabels([])

        self.canvas.draw()

    def connect_serial(self):
        try:
            self.serial_port = serial.Serial(self.selected_port, 57600, timeout=1)
        except serial.SerialException as e:
            print(f"Error: {e}")

    def start_reading(self):
        self.reading_active.set()
        self.reading_thread = threading.Thread(target=self.fetch_data)
        self.reading_thread.start()

    def stop_reading(self):
        self.reading_active.clear()
        if self.reading_thread:
            self.reading_thread.join()

    def toggle_update(self):
        self.updating = not self.updating
        if self.updating:
            self.start_button.configure(text="Stop")
            self.start_reading()
        else:
            self.start_button.configure(text="Start")
            self.stop_reading()

if __name__ == "__main__":
    ctk.set_appearance_mode("System")
    ctk.set_default_color_theme("dark-blue")

    app = RealTimeDataApp()
    app.mainloop()
