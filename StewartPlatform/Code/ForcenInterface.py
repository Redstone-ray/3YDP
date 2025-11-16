"""
Forcen Interface Class
Provides interface to communicate with Forcen ASY-00035-001 3-DOF sensing system
"""

import serial
import time


class ForcenInterface:
    """
    Interface class for Forcen 3-DOF sensing system.
    Handles communication, initialization, and calibration of the sensor.
    """
    
    # Response codes from the manual
    RESPONSE_SUCCESS = "a0x1"
    RESPONSE_SAVE_SUCCESS = "a0x2"
    RESPONSE_SAVE_FAIL = "e0x7"
    
    # Sample rate commands
    SAMPLE_RATES = {
        10: "<SDR10>",
        100: "<SDR100>",
        1000: "<SDR1000>"
    }
    
    # Device mode commands
    CMD_RUNNING_MODE = "<SDM2>"  # Continuous data streaming
    CMD_CALIBRATION_MODE = "<SDM5>"  # Taring/calibration mode
    CMD_SAVE = "<CS>"  # Save to EEPROM
    
    def __init__(self, com_port, baud_rate=115200, sample_rate=100, 
                 arm_cog_x=0.0, arm_cog_y=0.0, arm_cog_z=0.0, 
                 arm_weight=0.0, arm_length_z=0.0, arm_weight_confidence=0.0,
                 rotation_angle=0.0, scale_x=1.0, scale_y=1.0, flip_x=False, flip_y=False):
        """
        Initialize the Forcen Interface.
        
        Parameters:
        -----------
        com_port : str
            Serial port name (e.g., 'COM3')
        baud_rate : int
            Serial communication baud rate (default: 115200)
        sample_rate : int
            Sensor sampling rate in Hz. Valid values: 10, 100, 1000 (default: 100)
        arm_cog_x : float
            ARM COG X - Center of gravity X-coordinate in meters (Register U0)
        arm_cog_y : float
            ARM COG Y - Center of gravity Y-coordinate in meters (Register U1)
        arm_cog_z : float
            ARM COG Z - Center of gravity Z-coordinate in meters (Register U2)
        arm_weight : float
            ARM WEIGHT - Total weight of connected arm in kg (Register U3)
        arm_length_z : float
            ARM LENGTH Z - Distance to loading point in meters (Register U4)
        arm_weight_confidence : float
            ARM WEIGHT CONFIDENCE - Confidence ratio 0-1 (Register U5, default: 0.0)
        rotation_angle : float
            Angle in degrees to rotate the coordinate frame (counter-clockwise, default: 0.0)
            Used to align sensor coordinate frame with real-world coordinate frame
        scale_x : float
            Scale factor for X-axis radial distance correction (default: 1.0)
        scale_y : float
            Scale factor for Y-axis radial distance correction (default: 1.0)
        flip_x : bool
            Flip X-axis after rotation (default: False)
        flip_y : bool
            Flip Y-axis after rotation (default: False)
        """
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.sample_rate = sample_rate
        self.serial_connection = None
        
        # Position transformation parameters
        self.offset_x = 0.0  # Calibrated during initialization
        self.offset_y = 0.0  # Calibrated during initialization
        self.offset_weight = 0.0  # Weight offset in grams
        self.offsets_calibrated = False
        self.rotation_angle = rotation_angle  # Degrees
        self.rotation_rad = rotation_angle * 3.14159265359 / 180.0  # Convert to radians
        self.scale_x = scale_x  # Radial scale factor for X
        self.scale_y = scale_y  # Radial scale factor for Y
        self.flip_x = flip_x  # Flip X after rotation
        self.flip_y = flip_y  # Flip Y after rotation
        
        # Print transformation parameters for debugging
        if rotation_angle != 0.0 or scale_x != 1.0 or scale_y != 1.0 or flip_x or flip_y:
            print(f"Coordinate transformations enabled:")
            print(f"  Rotation: {rotation_angle}°")
            print(f"  Scale X: {scale_x}")
            print(f"  Scale Y: {scale_y}")
            if flip_x:
                print(f"  Flip X: True")
            if flip_y:
                print(f"  Flip Y: True")
        
        # Store Table 6 parameters
        self.parameters = {
            'U0': arm_cog_x,             # ARM COG X (m)
            'U1': arm_cog_y,             # ARM COG Y (m)
            'U2': arm_cog_z,             # ARM COG Z (m)
            'U3': arm_weight,            # ARM WEIGHT (kg)
            'U4': arm_length_z,          # ARM LENGTH Z (m)
            'U5': arm_weight_confidence  # ARM WEIGHT CONFIDENCE (ratio)
        }
        
        # Validate sample rate
        if sample_rate not in self.SAMPLE_RATES:
            raise ValueError(f"Invalid sample rate. Must be one of: {list(self.SAMPLE_RATES.keys())}")
    
    def connect(self):
        """
        Establish serial connection to the sensor.
        
        Returns:
        --------
        bool : True if connection successful, False otherwise
        """
        try:
            self.serial_connection = serial.Serial(
                port=self.com_port,
                baudrate=self.baud_rate,
                timeout=0.1,  # Shorter timeout for faster reading
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            # Disable input/output buffering
            self.serial_connection.reset_input_buffer()
            self.serial_connection.reset_output_buffer()
            time.sleep(2)  # Wait for connection to stabilize
            print(f"Connected to Forcen sensor on {self.com_port}")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to {self.com_port}: {e}")
            return False
    
    def disconnect(self):
        """Close the serial connection."""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Disconnected from sensor")
    
    def _clear_buffer(self):
        """
        Clear any pending data in the serial input buffer.
        Useful before sending commands to avoid reading stale data or RTD messages.
        """
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.reset_input_buffer()
    
    def _send_command(self, command, expect_ack=True, timeout=2):
        """
        Send a command to the sensor following ASCII protocol.
        
        Parameters:
        -----------
        command : str
            Command string (should already include <> brackets)
        expect_ack : bool
            If True, wait for and return acknowledgment response (not RTD data)
        timeout : float
            How long to wait for acknowledgment
        
        Returns:
        --------
        str : Response from sensor (acknowledgment), or None if error
        """
        if not self.serial_connection or not self.serial_connection.is_open:
            print("Error: Serial connection not open")
            return None
        
        try:
            # Clear buffer before sending command to avoid reading stale data
            self._clear_buffer()
            
            # Ensure command has proper format
            if not command.startswith('<'):
                command = '<' + command
            if not command.endswith('>'):
                command = command + '>'
            
            # Send command with newline (IFS)
            full_command = command + '\n'
            self.serial_connection.write(full_command.encode('ascii'))
            
            if not expect_ack:
                return None
            
            # Wait for acknowledgment response (not RTD data)
            # Acknowledgments typically contain 'a' (success) or 'e' (error) codes
            start_time = time.time()
            while time.time() - start_time < timeout:
                response = self.serial_connection.readline().decode('ascii').strip()
                
                if response:
                    # Check if this is an acknowledgment (starts with 'a' or 'e')
                    # and not RTD data (which would have command type after SOF)
                    if response.startswith('<'):
                        # Parse content after SOF
                        content = response[1:].lstrip('<').rstrip('>')
                        # Acknowledgments are short: a0x1, e0x7, etc.
                        if content.startswith('a') or content.startswith('e'):
                            return response
                    elif response.startswith('a') or response.startswith('e'):
                        # Some responses might not have brackets
                        return response
                    # Otherwise it's likely RTD data, keep reading
                
                time.sleep(0.05)
            
            # Timeout - no acknowledgment received
            return None
            
        except Exception as e:
            print(f"Error sending command '{command}': {e}")
            return None
    
    def _read_response(self, timeout=2):
        """
        Read a response from the sensor.
        
        Parameters:
        -----------
        timeout : float
            Time to wait for response in seconds
        
        Returns:
        --------
        str : Response string, or None if timeout
        """
        if not self.serial_connection or not self.serial_connection.is_open:
            return None
        
        try:
            old_timeout = self.serial_connection.timeout
            self.serial_connection.timeout = timeout
            response = self.serial_connection.readline().decode('ascii').strip()
            self.serial_connection.timeout = old_timeout
            return response
        except Exception as e:
            print(f"Error reading response: {e}")
            return None
    
    def initialize(self):
        """
        Initialize the sensor by sending configuration parameters.
        Configures sample rate and sends Table 6 parameters (EoAT parameters).
        
        Note: Does NOT start data streaming. Call start_streaming() after calibration.
        
        Returns:
        --------
        bool : True if initialization successful, False otherwise
        """
        if not self.serial_connection or not self.serial_connection.is_open:
            print("Error: Not connected to sensor. Call connect() first.")
            return False
        
        print("\n=== Initializing Forcen Sensor ===\n")
        
        # Clear any existing data stream before configuration
        self._clear_buffer()
        time.sleep(0.2)
        
        # Step 1: Set sample rate
        sample_rate_cmd = self.SAMPLE_RATES[self.sample_rate]
        response = self._send_command(sample_rate_cmd, expect_ack=True, timeout=3)
        if response and self.RESPONSE_SUCCESS in response:
            print(f"✓ Sample rate set to {self.sample_rate} Hz")
        else:
            print(f"⚠ Could not confirm sample rate. Response: {response}")
            print("  Continuing anyway - sensor may already be configured")
        
        time.sleep(0.5)
        
        # Step 2: Send Table 6 parameters (End of Arm Tool parameters)
        print("\nConfiguring End of Arm Tool parameters...")
        param_names = {
            'U0': 'ARM COG X',
            'U1': 'ARM COG Y',
            'U2': 'ARM COG Z',
            'U3': 'ARM WEIGHT',
            'U4': 'ARM LENGTH Z',
            'U5': 'ARM WEIGHT CONFIDENCE'
        }
        
        for register, value in self.parameters.items():
            # Format command as <SxxVVVV> where xx is register and VVVV is value
            command = f"<S{register}{value}>"
            response = self._send_command(command, expect_ack=True, timeout=3)
            
            if response and self.RESPONSE_SUCCESS in response:
                print(f"  ✓ {param_names[register]} ({register}) set to: {value}")
            else:
                print(f"  ⚠ Could not confirm {param_names[register]}. Response: {response}")
            
            time.sleep(0.3)
        
        print("\n=== Initialization Complete ===")
        return True
    
    def calibrate(self, taring_time=10, save_to_eeprom=None):
        """
        Run the sensor calibration (taring) process.
        Sends calibration commands and optionally saves to EEPROM.
        
        Parameters:
        -----------
        taring_time : int
            Time in seconds for taring data collection (default: 10)
        save_to_eeprom : bool or None
            If True, save to EEPROM. If False, don't save. If None, prompt user (default: None)
        
        Returns:
        --------
        bool : True if calibration successful, False otherwise
        """
        if not self.serial_connection or not self.serial_connection.is_open:
            print("Error: Not connected to sensor. Call connect() first.")
            return False
        
        print("\n" + "="*50)
        print("        FORCEN SENSOR CALIBRATION")
        print("="*50 + "\n")
        
        # Clear buffer before starting calibration
        self._clear_buffer()
        time.sleep(0.2)
        
        # Step 1: Send <SU60> command
        print("Setting taring to vertical...")
        response = self._send_command("<SU60>", expect_ack=True, timeout=3)
        if response:
            print(f"✓ Response: {response}")
        else:
            print("⚠ No acknowledgment received")
        
        time.sleep(0.3)
        
        # Step 2: Set taring time <STXX>
        print(f"\nSetting taring time to {taring_time} seconds...")
        response = self._send_command(f"<STT{taring_time}>", expect_ack=True, timeout=3)
        if response:
            print(f"✓ Response: {response}")
        else:
            print("⚠ No acknowledgment received")
        
        time.sleep(0.3)
        
        # Step 3: Initiate calibration <SDM5>
        print("\nInitiating calibration mode...")
        print("Sensor should be VERTICAL and STATIONARY...")
        self._clear_buffer()
        response = self._send_command(self.CMD_CALIBRATION_MODE, expect_ack=True, timeout=3)
        if response:
            print(f"✓ Initial Response: {response}")
        else:
            print("⚠ No acknowledgment received")
        
        # Step 4: Monitor calibration status codes
        print("\nMonitoring calibration process...")
        print("(Sensor is collecting data and calculating offsets...)\n")
        
        calibration_complete = False
        calibration_success = False
        start_time = time.time()
        max_wait_time = 30  # 30 seconds should be enough for vertical-only
        last_status = None
        
        while not calibration_complete and (time.time() - start_time) < max_wait_time:
            # Read any incoming data (status codes)
            if self.serial_connection.in_waiting > 0:
                try:
                    line = self.serial_connection.readline().decode('ascii').strip()
                    if line and line.startswith('<') and line.endswith('>'):
                        # Extract code from <rXXXX> or <aXXXX> format
                        code = line[1:-1]  # Remove < and >
                        
                        # Print all status codes
                        print(f"  {line}")
                        
                        # Check for completion codes
                        if code == 'r0x0':
                            calibration_complete = True
                            calibration_success = True
                        elif code == 'r0xD0':
                            print("⚠ Calibration failed - improper sensor orientation")
                            calibration_complete = False
                            calibration_success = False
                        elif code == 'r0xE0':
                            calibration_complete = True
                            calibration_success = True
                except Exception as e:
                    pass
            
            time.sleep(0.05)
        
        if not calibration_complete:
            print("\n⚠ Calibration timeout - no completion code received")
            print("   Sensor may still be processing. Check manually.")
            calibration_success = False
        
        print("\n" + "="*50)
        if calibration_success:
            print("    CALIBRATION COMPLETED SUCCESSFULLY")
        else:
            print("    CALIBRATION STATUS UNCERTAIN")
        print("="*50)
        
        if not calibration_success:
            return False
        
        time.sleep(0.5)
        
        # Step 5: Save to EEPROM based on parameter or prompt
        if save_to_eeprom is None:
            print("\n" + "-"*50)
            save_choice = input("Save calibration to sensor EEPROM? (y/n): ").strip().lower()
            save_to_eeprom = (save_choice == 'y')
        
        if save_to_eeprom:
            print("\nSaving calibration to EEPROM...")
            self._clear_buffer()
            response = self._send_command(self.CMD_SAVE, expect_ack=True, timeout=5)
            
            if response and self.RESPONSE_SAVE_SUCCESS in response:
                print("✓ Calibration saved successfully!")
            else:
                print(f"⚠ Save response: {response}")
        else:
            print("\nCalibration not saved. Settings will be lost on power cycle.")
        
        # Step 6: Calibrate position offsets
        print("\n" + "-"*50)
        print("Calibrating position and weight offsets...")
        if self._calibrate_position_offsets(target_weight=0):  # 531g + 268g for aligning jig weight
            print(f"✓ Position offsets calibrated: X={self.offset_x:.2f}mm, Y={self.offset_y:.2f}mm")
            print(f"✓ Weight offset calibrated: {self.offset_weight:.2f}g")
        else:
            print("⚠ Could not calibrate offsets")
        
        print("\n" + "="*50)
        print("      CALIBRATION PROCESS COMPLETE")
        print("="*50 + "\n")
        
        return True
    
    def read_data(self):
        """Read one frame of real-time data from sensor."""
        if not self.serial_connection or not self.serial_connection.is_open:
            return None
        
        try:
            # Clear old buffered data to get fresh readings
            if self.serial_connection.in_waiting > 100:
                self._clear_buffer()
                time.sleep(0.01)
            
            # Read and parse line: < Mx My Fz Weight Pitch >
            line = self.serial_connection.readline().decode('ascii').strip()
            if not (line.startswith('<') and line.endswith('>')):
                return None
            
            parts = line[1:-1].strip().split()
            
            if len(parts) == 5:
                return {
                    'Mx': float(parts[0]),
                    'My': float(parts[1]),
                    'Fz': float(parts[2]),
                    'Weight': float(parts[3]),
                    'Pitch_Angle': float(parts[4])
                }
            elif len(parts) == 3:
                return {
                    'Mx': float(parts[0]),
                    'My': float(parts[1]),
                    'Fz': float(parts[2])
                }
        except:
            return None
    
    def _calibrate_position_offsets(self, num_samples=20, target_weight=531.0):
        """
        Calibrate position and weight offsets by averaging current ball position and weight.
        Assumes ball is at center of platform during calibration.
        
        Parameters:
        -----------
        num_samples : int
            Number of samples to average for offset calculation
        target_weight : float
            Expected weight in grams (default: 531.0)
        
        Returns:
        --------
        bool : True if successful
        """
        x_samples = []
        y_samples = []
        weight_samples = []
        
        # Collect samples
        for i in range(num_samples):
            data = self.read_data()
            if data:
                x, y = self._estimate_ball_position_raw(data)
                if x is not None and y is not None:
                    x_samples.append(x)
                    y_samples.append(y)
                    # Use sensor's Weight field (already in grams)
                    # NOTE: Weight field is inverted - negate it
                    if 'Weight' in data:
                        weight_samples.append(-data['Weight'])
                    elif 'Fz' in data:
                        # Fallback to Fz if Weight not available
                        weight_g = data['Fz'] / 9.81
                        weight_samples.append(weight_g)
            time.sleep(0.05)  # 50ms between samples
        
        # Calculate offsets if we got enough samples
        if len(x_samples) >= num_samples // 2:
            self.offset_x = sum(x_samples) / len(x_samples)
            self.offset_y = sum(y_samples) / len(y_samples)
            
            # Calculate weight offset
            avg_weight = sum(weight_samples) / len(weight_samples)
            self.offset_weight = target_weight - avg_weight
            
            self.offsets_calibrated = True
            return True
        
        return False
    
    def get_weight(self, data):
        """
        Get calibrated weight from sensor data.
        
        Parameters:
        -----------
        data : dict
            Sensor data containing 'Weight' or 'Fz'
        
        Returns:
        --------
        float : Weight in grams with offset applied, or None if cannot calculate
        """
        if not data:
            return None
        
        # Prefer sensor's Weight field (already in grams)
        # NOTE: Weight field is inverted - negate it
        if 'Weight' in data:
            weight_g = -data['Weight']
        elif 'Fz' in data:
            # Fallback to converting Fz if Weight not available
            weight_g = data['Fz'] / 9.81
        else:
            return None
        
        # Apply offset
        return weight_g + self.offset_weight
    
    def start_streaming(self):
        """
        Put the device in RUNNING mode to start streaming real-time data (RTD).
        Should be called after calibration is complete.
        
        Returns:
        --------
        bool : True if successful
        """
        print("Setting device to RUNNING mode (continuous data streaming)...")
        self._clear_buffer()
        response = self._send_command(self.CMD_RUNNING_MODE, expect_ack=True, timeout=3)
        success = response and self.RESPONSE_SUCCESS in response
        
        if success:
            print("✓ Data streaming started")
        else:
            print(f"✗ Failed to start streaming. Response: {response}")
        
        return success
    
    def _estimate_ball_position_raw(self, data):
        """
        Raw position estimation without offset correction.
        
        Parameters:
        -----------
        data : dict
            Sensor data containing 'Mx', 'My', and 'Fz'
        
        Returns:
        --------
        tuple : (x, y) position in mm, or (None, None) if cannot calculate
        """
        if not data or 'Mx' not in data or 'My' not in data or 'Fz' not in data:
            return (None, None)
        
        # Get values (Mx, My in Nmm, Fz in mN)
        Mx = data['Mx']  # Moment around X-axis (Nmm)
        My = data['My']  # Moment around Y-axis (Nmm)
        Fz = data['Fz']  # Force in Z (mN)
        
        # Avoid division by zero
        if abs(Fz) < 10:  # Less than 10 mN is too small
            return (None, None)
        
        # Convert Fz from mN to N
        Fz_N = Fz / 1000.0
        
        # Calculate position in mm
        # Moment around X is caused by Y position: Mx = Fz * y
        # Moment around Y is caused by X position: My = Fz * x
        y = Mx / Fz_N  # mm (moment in Nmm, force in N gives mm)
        x = My / Fz_N  # mm
        
        return (x, y)
    
    def estimate_ball_position(self, data):
        """
        Estimate the position of a ball on a circular platform using sensor data.
        Applies calibrated offsets, rotation, and scaling to align with real-world coordinates.
        
        The ball creates moments around X and Y axes based on its position.
        Moment = Force × Distance, so Distance = Moment / Force
        
        Transformation order:
        1. Calculate raw position from sensor data
        2. Apply center offset correction (in raw sensor frame)
        3. Apply radial scale factors (still in sensor frame)
        4. Apply coordinate frame rotation (to align with real-world frame)
        
        Parameters:
        -----------
        data : dict
            Sensor data containing 'Mx', 'My', and 'Fz'
        
        Returns:
        --------
        tuple : (x, y) position in mm with all transformations applied, or (None, None) if cannot calculate
        """
        x_raw, y_raw = self._estimate_ball_position_raw(data)
        
        if x_raw is None or y_raw is None:
            return (None, None)
        
        # Step 1: Apply center offset correction (subtract raw offsets in sensor frame)
        x_centered = x_raw - self.offset_x
        y_centered = y_raw - self.offset_y
        
        # Step 2: Apply radial scale factors (still in sensor frame)
        x_scaled = x_centered * self.scale_x
        y_scaled = y_centered * self.scale_y
        
        # Step 3: Apply rotation to align sensor frame with real-world frame
        # Rotation matrix: [cos(θ) -sin(θ)] [x]
        #                  [sin(θ)  cos(θ)] [y]
        import math
        cos_theta = math.cos(self.rotation_rad)
        sin_theta = math.sin(self.rotation_rad)
        
        x_final = x_scaled * cos_theta - y_scaled * sin_theta
        y_final = x_scaled * sin_theta + y_scaled * cos_theta
        
        # Step 4: Apply axis flips if needed (after rotation)
        if self.flip_x:
            x_final = -x_final
        if self.flip_y:
            y_final = -y_final
        
        return (x_final, y_final)
    
    def visualize_ball_position(self):
        """
        Set up visualization for ball position on a 300mm diameter circular platform.
        This is non-blocking - it sets up the plot and returns objects for updating.
        
        Returns:
        --------
        tuple : (fig, ball, coord_text) - matplotlib figure, ball patch, and text object
                Returns (None, None, None) if matplotlib is not available
        """
        try:
            import matplotlib.pyplot as plt
            import matplotlib.patches as patches
        except ImportError:
            print("Error: matplotlib is required for visualization")
            print("Install with: pip install matplotlib")
            return (None, None, None)
        
        # Set up the plot
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_xlim(-200, 200)
        ax.set_ylim(-200, 200)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X Position (mm)')
        ax.set_ylabel('Y Position (mm)')
        ax.set_title('Ball Position on Platform')
        
        # Draw the 300mm diameter platform
        platform = patches.Circle((0, 0), 150, fill=False, edgecolor='black', linewidth=2)
        ax.add_patch(platform)
        
        # Initialize the ball (40mm diameter = 20mm radius)
        ball = patches.Circle((0, 0), 20, color='orange', alpha=0.8)
        ax.add_patch(ball)
        
        # Add text for coordinates and weight
        coord_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                            verticalalignment='top', fontfamily='monospace',
                            fontsize=10)
        
        plt.ion()  # Interactive mode
        plt.show()
        
        return (fig, ball, coord_text)
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()


# Example usage
if __name__ == "__main__":
    # Example parameters for a ball tracking setup
    sensor = ForcenInterface(
        com_port='COM7',
        baud_rate=115200,
        sample_rate=100,
        arm_cog_x=0.0,
        arm_cog_y=0.0,
        arm_cog_z=0.005,  # 5 mm
        arm_weight=1.2,    # 1.2 kg
        arm_length_z=0.009,  # 9 mm
        arm_weight_confidence=0.7,
        rotation_angle=135,  # Rotate coordinate frame (degrees, counter-clockwise)
        scale_x=1.1,         # X-axis radial scale factor
        scale_y=1.1,         # Y-axis radial scale factor
        flip_x=False,        # Flip X-axis after rotation
        flip_y=True,         # Flip Y-axis after rotation
    )
    
    # Using context manager
    with sensor:
        # Step 1: Initialize sensor
        if not sensor.initialize():
            print("Initialization failed!")
            exit(1)
        
        # Step 2: Run calibration (auto-save to EEPROM)
        if not sensor.calibrate(taring_time=5, save_to_eeprom=False):
            print("Calibration failed!")
            exit(1)
        
        # Step 3: Start data streaming
        print("\nStarting data stream...")
        if not sensor.start_streaming():
            print("Failed to start streaming!")
            exit(1)
        
        print("✓ Streaming started\n")
        
        # Set up visualization (non-blocking)
        import matplotlib.pyplot as plt
        fig, ball, coord_text = sensor.visualize_ball_position()
        
        if fig is None:
            print("Visualization setup failed!")
            exit(1)
        
        print("Visualizing ball position - Press Ctrl+C to stop\n")
        
        # Main loop: read data and update both console and visualization
        try:
            while True:
                data = sensor.read_data()
                if data:
                    x, y = sensor.estimate_ball_position(data)
                    
                    if x is not None and y is not None:
                        # Update ball position
                        ball.center = (x, y)
                        
                        # Get calibrated weight
                        weight_g = sensor.get_weight(data)
                        
                        # Update coordinate text with position and weight
                        coord_text.set_text(
                            f'X: {x:7.1f} mm\n'
                            f'Y: {y:7.1f} mm\n'
                            f'Weight: {weight_g:5.1f} g'
                        )
                        
                        # Check if ball is outside platform or weight dropped
                        distance_from_center = (x**2 + y**2)**0.5
                        if weight_g < 300:  # Weight dropped significantly (below 300g)
                            ball.set_color('grey')
                            ball.set_alpha(0.3)  # Translucent
                        elif distance_from_center > 150:
                            ball.set_color('orange')
                            ball.set_alpha(0.4)  # Dim if outside
                        else:
                            ball.set_color('orange')
                            ball.set_alpha(0.8)  # Normal brightness
                        
                        # Update plot
                        fig.canvas.draw_idle()
                        fig.canvas.flush_events()
                        
                        # Print to console with all sensor values
                        pitch = data.get('Pitch_Angle', 'N/A')
                        pitch_str = f"{pitch:5.1f}°" if pitch != 'N/A' else "  N/A"
                        
                        print(f"\rMx:{data['Mx']:7.1f}  My:{data['My']:7.1f}  Fz:{data['Fz']:7.1f}mN  "
                              f"Weight:{weight_g:5.1f}g  Pitch:{pitch_str}  X:{x:6.1f}mm  Y:{y:6.1f}mm  ", 
                              end='', flush=True)
                    else:
                        # No position - still show raw data
                        pitch = data.get('Pitch_Angle', 'N/A')
                        pitch_str = f"{pitch:5.1f}°" if pitch != 'N/A' else "  N/A"
                        weight_g = sensor.get_weight(data)
                        
                        print(f"\rMx:{data['Mx']:7.1f}  My:{data['My']:7.1f}  Fz:{data['Fz']:7.1f}mN  "
                              f"Weight:{weight_g:5.1f}g  Pitch:{pitch_str}  [No position]  ", 
                              end='', flush=True)
                
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n\nStopped")
            plt.ioff()
            plt.close()
