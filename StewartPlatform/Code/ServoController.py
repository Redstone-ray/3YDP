"""ServoController

This module provides a ServoController class that sends angle commands for
three servos to an Arduino over a serial port. Each servo angle is sent as
one byte (0-255). The Arduino should expect three bytes per command.

Behavior:
- Importing this module creates a module-level `controller` instance with
  default offsets (zeros). This allows other modules to `from ServoController
  import controller` and call `controller.send_angles([...])`.
- Running the file as a script starts an interactive prompt that asks for
  three servo angles and relays them to the controller. The interactive mode
  initializes with fixed offsets (see __main__ section).

Notes / assumptions:
- "3 hexadecimal values (2*3 bytes)" is interpreted as three bytes, each
  represented by two hex characters when printed. We send raw bytes over
  serial and also print their hex representation for debugging.
- Angles are specified in degrees and clamped to 0-90 before being sent.
- Auto-kills any conflicting Python processes holding the serial port.

Requires pyserial for serial communication.
"""

from __future__ import annotations

import sys
import struct
import subprocess
from typing import Iterable, List, Optional
import numpy as np

try:
	import serial
	from serial.serialutil import SerialException
except ImportError:
	raise ImportError("pyserial is required. Install with: pip install pyserial")


def _kill_processes_using_port(port: str) -> None:
	"""Kill any Python processes that might be holding the serial port."""
	try:
		import psutil
		current_pid = psutil.Process().pid
		killed = []
		
		for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
			try:
				if proc.info['pid'] == current_pid:
					continue
				if 'python' in proc.info['name'].lower():
					cmdline = ' '.join(proc.info['cmdline'] or [])
					if 'ServoController.py' in cmdline or 'servo_controller' in cmdline.lower():
						proc.kill()
						killed.append(proc.info['pid'])
			except (psutil.NoSuchProcess, psutil.AccessDenied):
				pass
		
		if killed:
			print(f"Killed conflicting processes: {killed}")
			import time
			time.sleep(0.5)  # Give OS time to release the port
	except ImportError:
		# psutil not available, try basic Windows taskkill approach
		if sys.platform == 'win32':
			try:
				# List processes and kill any Python ServoController instances
				result = subprocess.run(['tasklist', '/FI', 'IMAGENAME eq python.exe', '/FO', 'CSV'],
				                       capture_output=True, text=True, timeout=2)
				# This is a basic check - won't work perfectly but better than nothing
			except:
				pass


def _clamp_angle(a: float, min_angle: float = 0.0, max_angle: float = 90.0) -> int:
	"""Clamp an angle in degrees to an integer in the specified range."""
	try:
		v = int(round(a))
	except Exception:
		raise ValueError(f"Invalid angle value: {a}")
	min_int = int(round(min_angle))
	max_int = int(round(max_angle))
	if v < min_int:
		return min_int
	if v > max_int:
		return max_int
	return v


class ServoController:
	"""Controller for three servos over serial.

	Contract:
	- Inputs: three servo angles in degrees (iterable of length 3).
	- Output: writes three raw bytes to the serial port.
	- Each byte is the clamped angle (0-90).
	- Errors: raises ValueError for bad inputs or serial connection failures.
	"""

	def __init__(self, port: str = "COM3", baudrate: int = 115200, timeout: float = 1.0, 
	             visualize: bool = False, flip_servo: Optional[Iterable[bool]] = None,
	             neutral_angles: Optional[Iterable[float]] = None,
	             min_servo_angle: float = 0.0, max_servo_angle: float = 25.0,
	             led_rotation_offset: float = 0.0):
		"""Create a controller.

		port: serial port name (e.g., 'COM3' on Windows)
		baudrate: serial speed
		timeout: serial timeout in seconds
		visualize: if True, use SPV4 for 3D visualization
		flip_servo: iterable of three booleans to flip servo direction (e.g., [True, False, True])
		            If True for a servo, angle is inverted around its neutral position
		neutral_angles: iterable of three angles (degrees) defining the horizontal/neutral position
		                for each servo. Default is [0, 0, 0].
		                Without flip: commanded_angle is added to neutral_angle
		                With flip: real_angle = neutral_angle - commanded_angle
		                Example: neutral_angles=[15, 20, 15], flip_servo=[True, False, True]
		                  - Command 0° → sends [15, 0, 15] (horizontal)
		                  - Command 5° → sends [10, 5, 10] (servos 1&3: 15-5, servo 2: 0+5)
		min_servo_angle: minimum REAL raw servo angle in degrees (default 0)
		max_servo_angle: maximum REAL raw servo angle in degrees (default 25)
		led_rotation_offset: rotation offset in degrees to align LED reference frame with global frame (default 0)
		"""
		flips = list(flip_servo) if flip_servo is not None else [False, False, False]
		if len(flips) != 3:
			raise ValueError("flip_servo must be an iterable of three booleans")
		self.flip_servo: List[bool] = [bool(x) for x in flips]
		
		neutrals = list(neutral_angles) if neutral_angles is not None else [0.0, 0.0, 0.0]
		if len(neutrals) != 3:
			raise ValueError("neutral_angles must be an iterable of three numbers")
		self.neutral_angles: List[float] = [float(x) for x in neutrals]
		
		self.min_servo_angle: float = float(min_servo_angle)
		self.max_servo_angle: float = float(max_servo_angle)
		
		# LED configuration
		self.led_rotation_offset: float = float(led_rotation_offset)
		self.num_leds: int = 60
		self.first_led_index: int = 3
		self.active_leds: int = 57  # 60 - 3
		
		# LED command protocol constants
		self.LED_COMMAND_BYTE: int = 250
		self.LED_MODE_IDLE: int = 0
		self.LED_MODE_CALIBRATING: int = 1
		self.LED_MODE_RUNNING_BALANCED: int = 2
		self.LED_MODE_RUNNING_BALL: int = 3
		
		self.port = port
		self.baudrate = int(baudrate)
		self.timeout = float(timeout)
		self.visualize = visualize
		self._ser = None
		self.platform = None

		# Import SPV4 only if visualization is enabled
		if self.visualize:
			try:
				from SPV4 import StewartPlatform
				self.platform = StewartPlatform(l=25.9, l_base=10, l_link1=8, l_link2=10)
				print("Visualization enabled")
			except ImportError as e:
				print(f"Warning: Could not import SPV4 for visualization: {e}")
				self.visualize = False

		# Kill any conflicting processes using this port
		_kill_processes_using_port(self.port)

		# Open the serial port
		try:
			self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout, write_timeout=0.5)
			# Clear any stale data in buffers
			self._ser.reset_input_buffer()
			self._ser.reset_output_buffer()
			print(f"Successfully opened {self.port}")
		except SerialException as e:
			raise RuntimeError(f"Could not open serial port {self.port}: {e}") from e

	def send_angles(self, angles: Iterable[float]) -> bytes:
		"""Send three servo angles (degrees) to the Arduino.

		Returns the raw bytes that were sent.
		"""
		angs = list(angles)
		if len(angs) != 3:
			raise ValueError("angles must contain exactly three values")

		# Apply neutral angles and flips, then clamp
		# Store both the servo angles (what gets sent) and the clipped commanded angles
		final = []
		clipped_commanded_angles = []
		was_clamped = False
		
		for i, (a, flip, neutral) in enumerate(zip(angs, self.flip_servo, self.neutral_angles)):
			commanded = float(a)
			
			# Apply neutral angle and flip
			# Without flip: real_angle = neutral + commanded
			# With flip:    real_angle = neutral - commanded
			if flip:
				real_angle = neutral - commanded
			else:
				real_angle = neutral + commanded
			
			# Clamp the servo angle to physical limits
			val = _clamp_angle(real_angle, self.min_servo_angle, self.max_servo_angle)
			final.append(val)
			
			# Check if clamping occurred
			if abs(real_angle - val) > 0.5:  # tolerance for rounding
				was_clamped = True
				print(f"\033[1;31mWarning: Servo {i+1} angle clamped from {real_angle:.2f}° to {val}° (limits: {self.min_servo_angle:.0f}°-{self.max_servo_angle:.0f}°)\033[0m")
			
			# Back-calculate what commanded angle actually corresponds to this clamped servo angle
			if flip:
				clipped_commanded = neutral - val
			else:
				clipped_commanded = val - neutral
			clipped_commanded_angles.append(clipped_commanded)

		# Pack into three bytes
		data = struct.pack("BBB", final[0], final[1], final[2])

		# Send to Arduino
		if self._ser is None:
			raise RuntimeError("Serial port is not open")
		
		try:
			# Clear input buffer to prevent buildup (Arduino doesn't send much data)
			if self._ser.in_waiting > 0:
				self._ser.reset_input_buffer()
			self._ser.write(data)
			self._ser.flush()
		except SerialException as e:
			raise RuntimeError(f"Failed to write to serial port: {e}") from e
		
		# Visualize if enabled (use clipped commanded angles)
		if self.visualize and self.platform is not None:
			try:
				# Use forward kinematics with the 3 primary angles (clipped)
				# Estimate platform center z from average angle
				avg_angle = np.mean(clipped_commanded_angles)
				estimated_z = self.platform.l_base + self.platform.l_link1 * np.sin(avg_angle * np.pi / 180)
				
				# FK calculates secondary angles automatically using platform geometry constraint
				fk = self.platform.forward_kinematics(clipped_commanded_angles[0], clipped_commanded_angles[1], clipped_commanded_angles[2],
				                                      center_position=[0, 0, estimated_z])
				
				# Visualize using the FK result (which now has all 6 angles calculated)
				self.platform.visualize(fk['x1'], fk['x2'], fk['x3'],
				                       fk['theta_11'], fk['theta_12'],
				                       fk['theta_21'], fk['theta_22'],
				                       fk['theta_31'], fk['theta_32'])
			except Exception as e:
				print(f"Visualization error: {e}")
		
		# Store clipped angles for return info
		self._last_clipped_angles = clipped_commanded_angles
		
		return data

	def get_commanded_angle_limits(self, servo_index: int) -> tuple[float, float]:
		"""Get the commanded angle limits (in global reference frame) for a specific servo.
		
		Args:
			servo_index: 0, 1, or 2 for servos 1, 2, 3
			
		Returns:
			(min_commanded, max_commanded) in degrees in the global reference frame
		"""
		if servo_index < 0 or servo_index > 2:
			raise ValueError("servo_index must be 0, 1, or 2")
		
		flip = self.flip_servo[servo_index]
		neutral = self.neutral_angles[servo_index]
		
		# Back-calculate commanded angles from servo limits
		# Without flip: commanded = servo_angle - neutral
		# With flip: commanded = neutral - servo_angle
		if flip:
			# When flipped: higher commanded angle → lower servo angle
			# So min_servo → max_commanded, max_servo → min_commanded
			min_commanded = neutral - self.max_servo_angle
			max_commanded = neutral - self.min_servo_angle
		else:
			# Without flip: higher commanded angle → higher servo angle
			min_commanded = self.min_servo_angle - neutral
			max_commanded = self.max_servo_angle - neutral
		
		return (min_commanded, max_commanded)
	
	def print_commanded_angle_limits(self) -> None:
		"""Print the commanded angle limits for all servos in the global reference frame."""
		print("\nCommanded Angle Limits (global reference frame):")
		for i in range(3):
			min_cmd, max_cmd = self.get_commanded_angle_limits(i)
			print(f"  Servo {i+1}: [{min_cmd:+.1f}°, {max_cmd:+.1f}°]")
		print()
	
	def _send_led_command(self, mode: int, data: Optional[int] = None) -> None:
		"""Send an LED command to the Arduino.
		
		Args:
			mode: LED mode (IDLE, CALIBRATING, RUNNING_BALANCED, RUNNING_BALL)
			data: Optional data byte (e.g., LED position for RUNNING_BALL mode)
		"""
		if self._ser is None:
			return  # Silently fail if serial port is not open
		
		try:
			# Build command: [LED_COMMAND_BYTE, mode, optional data]
			if data is not None:
				cmd = struct.pack("BBB", self.LED_COMMAND_BYTE, mode, data)
			else:
				cmd = struct.pack("BB", self.LED_COMMAND_BYTE, mode)
			
			self._ser.write(cmd)
			self._ser.flush()
		except SerialException:
			# Don't let LED commands interfere with control loop
			pass
	
	def set_led_idle(self) -> None:
		"""Set LEDs to idle mode (blue circling animation)."""
		self._send_led_command(self.LED_MODE_IDLE)
	
	def set_led_calibrating(self) -> None:
		"""Set LEDs to calibration mode (blinking red every 4th LED)."""
		self._send_led_command(self.LED_MODE_CALIBRATING)
	
	def set_led_running_balanced(self) -> None:
		"""Set LEDs to running balanced mode (steady green every 4th LED)."""
		self._send_led_command(self.LED_MODE_RUNNING_BALANCED)
	
	def set_led_running_ball(self, x: float, y: float) -> None:
		"""Set LEDs to running ball mode (orange LED at ball position).
		
		Args:
			x: Ball X position in mm (global reference frame)
			y: Ball Y position in mm (global reference frame)
		"""
		# Convert ball position (x, y) to angle
		angle_rad = np.arctan2(y, x)
		angle_deg = angle_rad * 180.0 / np.pi
		
		# Apply rotation offset to convert from global to LED reference frame
		angle_deg += self.led_rotation_offset
		
		# Normalize to 0-360 range
		angle_deg = angle_deg % 360.0
		
		# Convert angle to LED index (0-56)
		# 360 degrees / 57 LEDs = 6.316 degrees per LED
		led_index = int(round((angle_deg / 360.0) * self.active_leds)) % self.active_leds
		
		self._send_led_command(self.LED_MODE_RUNNING_BALL, led_index)

	def close(self) -> None:
		"""Close the serial port if open."""
		if self._ser is not None:
			try:
				self._ser.close()
			except Exception:
				pass
			self._ser = None


# Module-level controller is NOT created on import to avoid blocking the serial port.
# To use from another module: from ServoController import ServoController
# then create your own instance: controller = ServoController(neutral_angles=[...], flip_servo=[...], port="COM3")
controller = None


if __name__ == "__main__":
	# Neutral angles: the real servo angle when commanded angle is 0° (horizontal position)
	# Example: [15, 20, 15] means servo 1 is horizontal at 15°, servo 2 at 20°, servo 3 at 15°
	# Without flip: real_angle = neutral + commanded (e.g., neutral=0, command 5° → send 5°)
	# With flip:    real_angle = neutral - commanded (e.g., neutral=15, command 5° → send 10°)
	NEUTRAL_ANGLES = [128, 128, 128]
	
	# Flip servo direction: if True, servo angle is inverted around its neutral position
	# Use this when servo rotation is backwards relative to the global reference frame
	# Example: [True, False, True] flips servos 1 and 3
	FLIP_SERVOS = [True, True, True]

	print("ServoController interactive mode")
	print("Enter three angles in degrees separated by spaces (or 'q' to quit)")
	print()
	
	# List available COM ports
	try:
		from serial.tools import list_ports
		ports = list(list_ports.comports())
		if ports:
			print("Available COM ports:")
			for p in ports:
				print(f"  - {p.device}: {p.description}")
		else:
			print("No COM ports detected.")
	except Exception as e:
		print(f"Could not list COM ports: {e}")
	
	print()
	port = input("Enter COM port [default COM8]: ").strip() or "COM8"
	visualize = input("Enable visualization? (y/n) [default y]: ").strip().lower() != 'n'
	print()
	
	# Create controller for interactive session
	interactive = ServoController(port=port, visualize=visualize, 
	                              flip_servo=FLIP_SERVOS, neutral_angles=NEUTRAL_ANGLES,
	                              min_servo_angle=85.0, max_servo_angle=144.0)
	
	# Display commanded angle limits
	interactive.print_commanded_angle_limits()

	try:
		while True:
			raw = input("[a,b,c]: ").strip()
			if not raw:
				continue
			if raw.lower() in ("q", "quit", "exit"):
				break
			parts = raw.replace(",", " ").split()
			if len(parts) != 3:
				print("Please enter exactly three numbers, e.g. '90 45 120'")
				continue
			try:
				angles = [float(p) for p in parts]
			except ValueError:
				print("Could not parse angles; please enter numeric values.")
				continue
			sent = interactive.send_angles(angles)
			clipped = interactive._last_clipped_angles
			print(f"Sent: {sent.hex()} (hex), actual angles sent: {[f'{a:.2f}°' for a in clipped]}")
	finally:
		interactive.close()
		print("Exiting interactive mode")

