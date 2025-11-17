"""
PID Tuner GUI - Separate Process
Provides a GUI to adjust PID parameters and communicates via file.
"""

import tkinter as tk
from tkinter import ttk
import struct
import time
import sys
import os

# PID gains data structure:
# 8 doubles: kp_x, ki_x, kd_x, pf_x, kp_y, ki_y, kd_y, pf_y
STRUCT_FORMAT = '8d'
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)

class PIDTunerGUI:
    def __init__(self, data_file='pid_gains.dat'):
        self.data_file = data_file
        
        # Create main window
        self.root = tk.Tk()
        self.root.title("PID Tuner")
        self.root.geometry("400x500")
        
        # Default PID values
        self.gains = {
            'kp_x': 0.055,
            'ki_x': 0.01,
            'kd_x': 0.03,
            'pf_x': 0.0,
            'kp_y': 0.055,
            'ki_y': 0.01,
            'kd_y': 0.03,
            'pf_y': 0.0
        }
        
        # Read initial values from file if available
        self._read_initial_gains()
        
        # Store initial gains for reset functionality
        self.initial_gains = self.gains.copy()
        
        # Store slider references for reset functionality
        self.sliders = {}
        self.value_vars = {}
        
        # Create UI
        self._create_ui()
        
        # Don't write gains here - let controller's initial values persist
        # Only write when user adjusts sliders
        
        # Auto-update
        self.root.after(100, self._update_loop)
    
    def _create_ui(self):
        """Create the GUI layout."""
        
        # Title
        title = tk.Label(self.root, text="PID Parameter Tuner", font=("Arial", 16, "bold"))
        title.pack(pady=10)
        
        # Info label
        info = tk.Label(self.root, text="Adjust PID gains in real-time", font=("Arial", 10))
        info.pack()
        
        # Frame for controls
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.pack(fill=tk.BOTH, expand=True)
        
        # X-axis section
        x_label = tk.Label(control_frame, text="X-Axis (Roll)", font=("Arial", 12, "bold"))
        x_label.grid(row=0, column=0, columnspan=3, pady=5, sticky=tk.W)
        
        self._create_slider(control_frame, "Kp", 'kp_x', 0.0, 0.2, 0.001, row=1)
        self._create_slider(control_frame, "Ki", 'ki_x', 0.0, 0.05, 0.001, row=2)
        self._create_slider(control_frame, "Kd", 'kd_x', 0.0, 0.1, 0.001, row=3)
        self._create_slider(control_frame, "Pf", 'pf_x', 0.0, 0.5, 0.01, row=4)
        
        # Separator
        sep = ttk.Separator(control_frame, orient='horizontal')
        sep.grid(row=5, column=0, columnspan=3, sticky='ew', pady=10)
        
        # Y-axis section
        y_label = tk.Label(control_frame, text="Y-Axis (Pitch)", font=("Arial", 12, "bold"))
        y_label.grid(row=6, column=0, columnspan=3, pady=5, sticky=tk.W)
        
        self._create_slider(control_frame, "Kp", 'kp_y', 0.0, 0.2, 0.001, row=7)
        self._create_slider(control_frame, "Ki", 'ki_y', 0.0, 0.05, 0.001, row=8)
        self._create_slider(control_frame, "Kd", 'kd_y', 0.0, 0.1, 0.001, row=9)
        self._create_slider(control_frame, "Pf", 'pf_y', 0.0, 0.5, 0.01, row=10)
        
        # Buttons frame
        button_frame = ttk.Frame(self.root, padding="10")
        button_frame.pack()
        
        # Reset button
        reset_btn = tk.Button(button_frame, text="Reset to Defaults", command=self._reset_gains,
                             bg="#ff6b6b", fg="white", font=("Arial", 10, "bold"))
        reset_btn.pack(side=tk.LEFT, padx=5)
        
        # Copy X to Y button
        copy_btn = tk.Button(button_frame, text="Copy X → Y", command=self._copy_x_to_y,
                           bg="#4ecdc4", fg="white", font=("Arial", 10, "bold"))
        copy_btn.pack(side=tk.LEFT, padx=5)
        
        # Status label
        self.status_label = tk.Label(self.root, text=f"Writing to: {self.data_file}", 
                                     font=("Arial", 9), fg="gray")
        self.status_label.pack(pady=5)
    
    def _read_initial_gains(self):
        """Read initial PID gains from file if available."""
        try:
            if os.path.exists(self.data_file):
                with open(self.data_file, 'rb') as f:
                    data = f.read(STRUCT_SIZE)
                    if len(data) == STRUCT_SIZE:
                        gains = struct.unpack(STRUCT_FORMAT, data)
                        self.gains['kp_x'], self.gains['ki_x'], self.gains['kd_x'], self.gains['pf_x'], \
                        self.gains['kp_y'], self.gains['ki_y'], self.gains['kd_y'], self.gains['pf_y'] = gains
                        print(f"Loaded initial gains from file")
        except Exception as e:
            print(f"Could not read initial gains: {e}")
    
    def _create_slider(self, parent, label, key, min_val, max_val, resolution, row):
        """Create a labeled slider."""
        # Label
        lbl = tk.Label(parent, text=f"{label}:", font=("Arial", 10))
        lbl.grid(row=row, column=0, sticky=tk.W, padx=5, pady=2)
        
        # Value display
        value_var = tk.StringVar(value=f"{self.gains[key]:.3f}")
        value_label = tk.Label(parent, textvariable=value_var, font=("Arial", 10, "bold"),
                              width=6, anchor=tk.E)
        value_label.grid(row=row, column=2, padx=5)
        
        # Slider
        slider = tk.Scale(parent, from_=min_val, to=max_val, resolution=resolution,
                         orient=tk.HORIZONTAL, showvalue=0, length=200,
                         command=lambda val, k=key, v=value_var: self._on_slider_change(k, val, v))
        slider.set(self.gains[key])
        slider.grid(row=row, column=1, padx=5, pady=2)
        
        # Store references for reset functionality
        self.sliders[key] = slider
        self.value_vars[key] = value_var
        
        return slider
    
    def _on_slider_change(self, key, value, value_var):
        """Handle slider value change."""
        self.gains[key] = float(value)
        value_var.set(f"{float(value):.3f}")
        self._write_gains()
    
    def _write_gains(self):
        """Write current gains to file."""
        data = struct.pack(STRUCT_FORMAT,
                          self.gains['kp_x'], self.gains['ki_x'], 
                          self.gains['kd_x'], self.gains['pf_x'],
                          self.gains['kp_y'], self.gains['ki_y'],
                          self.gains['kd_y'], self.gains['pf_y'])
        try:
            with open(self.data_file, 'wb') as f:
                f.write(data)
        except Exception as e:
            print(f"Error writing gains: {e}")
    
    def _reset_gains(self):
        """Reset all gains to initial values from file."""
        for key, value in self.initial_gains.items():
            self.gains[key] = value
            # Update slider widget
            if key in self.sliders:
                self.sliders[key].set(value)
            # Update value display
            if key in self.value_vars:
                self.value_vars[key].set(f"{value:.3f}")
        
        self._write_gains()
        print("Reset to initial gains")
    
    def _copy_x_to_y(self):
        """Copy X-axis gains to Y-axis."""
        y_keys = ['kp_y', 'ki_y', 'kd_y', 'pf_y']
        x_keys = ['kp_x', 'ki_x', 'kd_x', 'pf_x']
        
        for x_key, y_key in zip(x_keys, y_keys):
            self.gains[y_key] = self.gains[x_key]
            # Update slider widget
            if y_key in self.sliders:
                self.sliders[y_key].set(self.gains[y_key])
            # Update value display
            if y_key in self.value_vars:
                self.value_vars[y_key].set(f"{self.gains[y_key]:.3f}")
        
        self._write_gains()
        print(f"Copied X gains to Y: Kp={self.gains['kp_y']:.3f} Ki={self.gains['ki_y']:.3f} Kd={self.gains['kd_y']:.3f} Pf={self.gains['pf_y']:.3f}")
    
    def _update_loop(self):
        """Periodic update (can be used for reading feedback, etc)."""
        # Currently just keeps the GUI responsive
        self.root.after(100, self._update_loop)
    
    def run(self):
        """Start the GUI."""
        print(f"PID Tuner GUI started")
        print(f"Writing gains to: {self.data_file}")
        self.root.mainloop()
        
        # Cleanup on exit
        try:
            if os.path.exists(self.data_file):
                os.remove(self.data_file)
        except:
            pass


if __name__ == "__main__":
    data_file = sys.argv[1] if len(sys.argv) > 1 else 'pid_gains.dat'
    tuner = PIDTunerGUI(data_file)
    tuner.run()
