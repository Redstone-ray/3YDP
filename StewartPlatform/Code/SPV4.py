"""
Stewart Platform Kinematics - Clean Class Implementation
Provides inverse and forward kinematics for a 3-DOF Stewart Platform.

Based on the original implementation from Learn content.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class StewartPlatform:
    """
    Stewart Platform with 3 actuator chains (6 DOF total, 2 joints per chain).
    
    Geometry:
    - Base: Circle with 3 attachment points at 90°, 210°, 330°
    - Platform: Circle with 3 attachment points at 0°, 120°, 240°
    - Each chain: 2 links connected by revolute joints
    
    Angle Convention (IMPORTANT):
    - theta_x1 (x=1,2,3): Servo angle for first link of chain x
    - Measured from the HORIZONTAL (x-y plane), NOT from vertical
    - theta = 0° means link is horizontal (parallel to ground)
    - theta > 0° means link angles upward from horizontal
    - Example: theta_11 = 30° means chain 1's first link makes a 30° angle above horizontal
    
    Coordinate System:
    - Z-axis points upward (vertical)
    - X-Y plane is the horizontal ground plane
    - Base attachment points lie in the Z=0 plane
    """
    
    def __init__(self, l=10, l_base=10, l_link1=8, l_link2=8):
        """
        Initialize Stewart Platform with geometric parameters.
        
        Args:
            l: Platform equilateral triangle SIDE LENGTH (matches original definition)
                - Platform circumradius a = l / (2*cos(30°)) ≈ l / 1.732
                - 3 attachment points form an equilateral triangle with side length l
                - All vertices lie on a circle of radius a
            l_base: Base circle radius (same for all chains)
            l_link1: First link length (same for all chains)
            l_link2: Second link length (same for all chains)
        """
        self.l = l
        self.l_base = l_base
        self.l_link1 = l_link1
        self.l_link2 = l_link2
        
        # Visualization state
        self.fig = None
        self.ax = None
    
    def inverse_kinematics(self, normal_vector, center_position):
        """
        Calculate joint angles from desired platform orientation and position.
        
        Args:
            normal_vector: [nx, ny, nz] - will be normalized
            center_position: [x, y, z]
            
        Returns:
            dict with x1, x2, x3, theta_11, theta_12, theta_21, theta_22, theta_31, theta_32
        """
        nrm = np.array(normal_vector, dtype=float)
        S = np.array(center_position, dtype=float)
        
        # Calculate platform attachment points
        # x1 is at 90°, x2 is at 330°, x3 is at 210°
        x1, x2, x3 = self._calculate_platform_points(nrm, S)
        
        # Calculate angles for each chain
        # Physical Chain 1 (base at 90°) → x1 (platform at 90°)
        # Physical Chain 2 (base at 210°) → x3 (platform at 210°)
        # Physical Chain 3 (base at 330°) → x2 (platform at 330°)
        theta_11, theta_12 = self._calc_angles_at_base(x1, 90)   # Chain 1
        theta_21, theta_22 = self._calc_angles_at_base(x3, 210)  # Chain 2
        theta_31, theta_32 = self._calc_angles_at_base(x2, 330)  # Chain 3
        
        return {
            'x1': x1, 'x2': x2, 'x3': x3,
            'theta_11': theta_11, 'theta_12': theta_12,
            'theta_21': theta_21, 'theta_22': theta_22,
            'theta_31': theta_31, 'theta_32': theta_32
        }
    
    def forward_kinematics(self, theta_11, theta_21, theta_31, center_position):
        """
        Calculate platform positions from primary servo angles only.
        Secondary angles are calculated geometrically using the constraint that
        all platform attachment points must lie on a circle of radius l.
        
        This matches the original implementation from Learn content.
        
        Args:
            theta_11, theta_21, theta_31: Primary servo joint angles in degrees
            center_position: [x, y, z] estimated platform center (default [0,0,12])
            
        Returns:
            dict with x1, x2, x3 and all 6 angles
        """
        S = np.array(center_position, dtype=float)
        
        # Calculate secondary angles using geometric constraints
        # Platform radius (circumradius of triangle)
        a = self.l / (2 * np.cos(30 * np.pi / 180))
        
        # Chain 1: Calculate theta_12 such that x1 is at distance a from S
        x1, theta_12 = self._forward_chain1_constrained(theta_11, S, a)
        
        # Chain 2 (base at 210°): Calculate theta_22 for x3 (at 210°)
        # Note: _forward_chain3_constrained produces point at 210° (negative x)
        x3, theta_22 = self._forward_chain3_constrained(theta_21, S, a)
        
        # Chain 3 (base at 330°): Calculate theta_32 for x2 (at 330°)
        # Note: _forward_chain2_constrained produces point at 330° (positive x)
        x2, theta_32 = self._forward_chain2_constrained(theta_31, S, a, x1)
        
        return {
            'x1': x1, 'x2': x2, 'x3': x3,
            'theta_11': theta_11, 'theta_12': theta_12,
            'theta_21': theta_21, 'theta_22': theta_22,
            'theta_31': theta_31, 'theta_32': theta_32
        }
    
    def visualize(self, x1, x2, x3, theta_11, theta_12, theta_21, theta_22, theta_31, theta_32):
        """
        Visualize the platform in 3D. Only updates when explicitly called.
        
        Args:
            x1, x2, x3: Platform attachment points
            theta_11, theta_12, theta_21, theta_22, theta_31, theta_32: All joint angles in degrees
        """
        # Create figure on first call
        if self.fig is None:
            self.fig = plt.figure(figsize=(10, 8))
            self.ax = self.fig.add_subplot(111, projection='3d')
            plt.ion()
        
        self.ax.clear()
        
        # Base circle
        theta = np.linspace(0, 2 * np.pi, 100)
        x_circ = self.l_base * np.cos(theta)
        y_circ = self.l_base * np.sin(theta)
        self.ax.plot(x_circ, y_circ, np.zeros_like(theta), 'k--', alpha=0.3)
        
        # Base points
        # Chain 1: 90° (top, +Y direction)
        # Chain 2: 210° (bottom-left, -120° from +X)
        # Chain 3: 330° (bottom-right, -30° from +X)
        base1 = np.array([0, self.l_base, 0])  # 90°
        base2 = np.array([self.l_base * np.cos(210*np.pi/180), self.l_base * np.sin(210*np.pi/180), 0])  # 210°
        base3 = np.array([self.l_base * np.cos(330*np.pi/180), self.l_base * np.sin(330*np.pi/180), 0])  # 330°
        self.ax.scatter(*zip(base1, base2, base3), color='black', s=100, marker='s')
        
        # Platform circle
        # Calculate the center and normal of the platform
        platform_center = (x1 + x2 + x3) / 3
        
        # Calculate the actual radius from the center to the attachment points
        actual_radius = np.linalg.norm(x1 - platform_center)
        
        # Create a circle in the plane of the platform
        circle_theta = np.linspace(0, 2 * np.pi, 100)
        # We need to recreate the v_hat and u_hat basis vectors for the platform orientation
        # For simplicity, if the platform is roughly horizontal, we can approximate
        # But let's properly calculate the basis from the three points
        v_platform = (x1 - platform_center) / np.linalg.norm(x1 - platform_center)
        normal = np.cross(x2 - x1, x3 - x1)
        normal = normal / np.linalg.norm(normal)
        u_platform = np.cross(normal, v_platform)
        u_platform = u_platform / np.linalg.norm(u_platform)
        
        # Create circle points using the ACTUAL radius from FK results
        platform_circle = np.array([platform_center + actual_radius * (np.cos(t) * v_platform + np.sin(t) * u_platform) 
                                   for t in circle_theta])
        self.ax.plot(platform_circle[:, 0], platform_circle[:, 1], platform_circle[:, 2], 
                    'b-', linewidth=2, alpha=0.3, label=f'Platform Circle (r={actual_radius:.1f})')
        
        # Platform triangle (inscribed in circle)
        tri = np.array([x1, x2, x3, x1])
        self.ax.plot(tri[:, 0], tri[:, 1], tri[:, 2], 'b-', linewidth=2, label='Platform Triangle')
        
        # Platform attachment points
        self.ax.scatter(*zip(x1, x2, x3), color='blue', s=100, marker='o')
        
        # Actuator chains with labels
        # NOTE: Platform points are at 90°, 330°, 210° (x1, x2, x3)
        #       Base points are at 90°, 210°, 330° (base1, base2, base3)
        #       So: Chain 1 connects base1→x1, Chain 2 connects base2→x3, Chain 3 connects base3→x2
        self._plot_chain(base1, x1, theta_11, 'red', f'Chain 1: θ={theta_11:.1f}°')
        self._plot_chain(base2, x3, theta_21, 'green', f'Chain 2: θ={theta_21:.1f}°')  # Swapped: base2→x3
        self._plot_chain(base3, x2, theta_31, 'purple', f'Chain 3: θ={theta_31:.1f}°')  # Swapped: base3→x2
        
        # Add angle text annotations on the plot
        self.ax.text(base1[0], base1[1], base1[2] + 1, f'{theta_11:.1f}°', color='red', fontsize=9)
        self.ax.text(base2[0], base2[1], base2[2] + 1, f'{theta_21:.1f}°', color='green', fontsize=9)
        self.ax.text(base3[0], base3[1], base3[2] + 1, f'{theta_31:.1f}°', color='purple', fontsize=9)
        
        # Configure plot
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim([-20, 20])
        self.ax.set_ylim([-20, 20])
        self.ax.set_zlim([0, 25])
        self.ax.legend(loc='upper right')
        
        plt.draw()
        plt.pause(0.01)
    
    # ========== Internal Methods ==========
    
    def _calculate_platform_points(self, nrm, S):
        """Calculate 3 platform attachment points from normal vector and center.
        MUST match original position_and_orientation() from Learn content EXACTLY.
        """
        # Normalize
        nrm = nrm / np.linalg.norm(nrm)
        
        # CRITICAL: Must match original exactly!
        # Original: v_hat = cross(nrm, [1,0,0]) then u_hat = cross(v_hat, nrm)
        vector_i = np.array([1, 0, 0])
        cross_product = np.cross(nrm, vector_i)
        anorm = np.linalg.norm(cross_product)
        v_hat = cross_product / anorm
        
        # Calculate circumradius (for equilateral triangle with side length l)
        a = self.l / (2 * np.cos(30 * np.pi / 180))
        
        # u_hat completes the basis - CRITICAL ORDER: cross(v_hat, nrm) NOT cross(nrm, v_hat)!
        cross_product = np.cross(v_hat, nrm)
        anorm = np.linalg.norm(cross_product)
        u_hat = cross_product / anorm
        
        # Platform attachment points - MUST match original formulas exactly:
        # x1 = S + a * v_hat
        # x2 = S - a*sin(30°)*v_hat + a*cos(30°)*u_hat
        # x3 = S - a*sin(30°)*v_hat - a*cos(30°)*u_hat
        x1 = S + a * v_hat
        x2 = S - a * np.sin(30.0 * np.pi / 180.0) * v_hat + a * np.cos(30.0 * np.pi / 180.0) * u_hat
        x3 = S - a * np.sin(30.0 * np.pi / 180.0) * v_hat - a * np.cos(30.0 * np.pi / 180.0) * u_hat
        
        return x1, x2, x3
    
    def _calc_angles_at_base(self, target, base_angle_deg):
        """Calculate angles for any chain at a given base angle.
        
        Args:
            target: [x, y, z] target position in 3D space
            base_angle_deg: Base position angle in degrees (0-360)
        
        Returns:
            (theta_1, theta_2): Primary and secondary joint angles in degrees
        
        Angle Definition:
        theta_1 is the angle between link1 and the HORIZONTAL (x-y plane).
        - theta_1 = 0° means link1 is horizontal (parallel to x-y plane)
        - theta_1 > 0° means link1 angles upward from horizontal
        """
        base_angle_rad = base_angle_deg * np.pi / 180
        base_pos = np.array([self.l_base * np.cos(base_angle_rad), 
                            self.l_base * np.sin(base_angle_rad), 
                            0])
        
        # Check if position is reachable
        distance = np.linalg.norm(target - base_pos)
        max_reach = self.l_link1 + self.l_link2
        
        if distance > max_reach:
            raise ValueError(f"Base at {base_angle_deg}°: Target unreachable! Distance={distance:.2f}, max_reach={max_reach:.2f}")
        
        # Use radial distance from Z-axis for all chains
        y_tmp = np.sqrt(target[0]**2 + target[1]**2)
        z_tmp = target[2]
        
        # For chains at 90°: use special formula (chain points in +Y direction)
        if abs(base_angle_deg - 90) < 1:
            AA = self.l_link2**2 - (self.l_base - target[1])**2 - self.l_link1**2 - target[2]**2
            bb = 2 * (self.l_base - target[1]) * self.l_link1
            cc = 2 * target[2] * self.l_link1
            denom = np.sqrt(bb**2 + cc**2)
            
            arg1 = AA / denom
            arg2 = bb / denom
            if abs(arg1) > 1.0 or abs(arg2) > 1.0:
                raise ValueError(f"Base at {base_angle_deg}°: Invalid arccos argument")
            
            theta_1 = np.arccos(arg1) - np.arccos(arg2)
            theta_2 = np.arcsin((target[2] - self.l_link1 * np.sin(theta_1)) / self.l_link2)
            
        # For other chains: use radial formula
        else:
            AA = -(self.l_link2**2 - (self.l_base - y_tmp)**2 - self.l_link1**2 - z_tmp**2)
            bb = 2 * (self.l_base - y_tmp) * self.l_link1
            cc = 2 * z_tmp * self.l_link1
            denom = np.sqrt(bb**2 + cc**2)
            
            arg1 = AA / denom
            arg2 = bb / denom
            if abs(arg1) > 1.0 or abs(arg2) > 1.0:
                raise ValueError(f"Base at {base_angle_deg}°: Invalid arcsin argument")
            
            theta_1 = np.arcsin(arg1) + np.arcsin(arg2)
            theta_2 = np.arcsin((target[2] - self.l_link1 * np.sin(theta_1)) / self.l_link2)
        
        return theta_1 * 180 / np.pi, theta_2 * 180 / np.pi
    
    def _calc_angles_chain1(self, x1):
        """Calculate angles for chain 1. DEPRECATED - use _calc_angles_at_base instead."""
        return self._calc_angles_at_base(x1, 90)
    
    def _calc_angles_chain2(self, x2):
        """Calculate angles for chain 2. DEPRECATED - use _calc_angles_at_base instead."""
        return self._calc_angles_at_base(x2, 210)
    
    def _calc_angles_chain3(self, x3):
        """Calculate angles for chain 3. DEPRECATED - use _calc_angles_at_base instead."""
        return self._calc_angles_at_base(x3, 330)
    
    def _forward_chain1_constrained(self, theta_11, S, a):
        """Calculate x1 and theta_12 for chain 1 using platform circle constraint.
        Based on original calculate_position_1 from Learn content.
        
        Note: The circle constraint has TWO solutions. We choose the one where
        the platform center z-coordinate is closest to S[2].
        """
        th11_rad = theta_11 * np.pi / 180
        
        # Position after first link
        AA_tmp = self.l_base + self.l_link1 * np.cos(th11_rad)
        BB_tmp = -S[2] + self.l_link1 * np.sin(th11_rad)
        CC_tmp = a*a - AA_tmp**2 - BB_tmp**2 - self.l_link2**2
        denom_AA = np.sqrt(AA_tmp**2 + BB_tmp**2)
        
        # Calculate theta_12 using geometric constraint
        # There are TWO possible solutions from the circle constraint
        cc = np.arccos(CC_tmp / (denom_AA * 2 * self.l_link2))
        bb2 = np.arccos(AA_tmp / denom_AA)
        
        # Solution 1: π - (cc - bb2)
        theta_12_rad_1 = np.pi - (cc - bb2)
        x1_1 = np.array([
            0,
            self.l_base + self.l_link1*np.cos(th11_rad) - self.l_link2*np.cos(theta_12_rad_1),
            self.l_link1*np.sin(th11_rad) + self.l_link2*np.sin(theta_12_rad_1)
        ])
        
        # Solution 2: π - (cc + bb2) OR -(π - (cc - bb2))
        # Actually, the other solution is just -theta_12_1 in terms of the link angle
        # Let's use arcsin approach for the other branch
        # Or we can try: θ12 = -π + (cc - bb2)
        theta_12_rad_2 = -np.pi + (cc - bb2)
        x1_2 = np.array([
            0,
            self.l_base + self.l_link1*np.cos(th11_rad) - self.l_link2*np.cos(theta_12_rad_2),
            self.l_link1*np.sin(th11_rad) + self.l_link2*np.sin(theta_12_rad_2)
        ])
        
        # Choose the solution where z is closer to S[2]
        # (This ensures we get the configuration that puts the platform at the right height)
        if abs(x1_1[2] - S[2]) < abs(x1_2[2] - S[2]):
            return x1_1, theta_12_rad_1 * 180 / np.pi
        else:
            return x1_2, theta_12_rad_2 * 180 / np.pi
    
    def _forward_chain2_constrained(self, theta_21, S, a, x1):
        """Calculate x2 and theta_22 for chain 2 using platform circle constraint.
        Based on original calculate_position_2 from Learn content.
        
        Note: The circle constraint has TWO solutions. We choose the one where
        the platform center z-coordinate is closest to S[2].
        """
        th21_rad = theta_21 * np.pi / 180
        
        # Position calculations
        AA_tmp = -(self.l_base + self.l_link1 * np.cos(th21_rad))
        BB_tmp = S[2] - self.l_link1 * np.sin(th21_rad)
        CC_tmp = a*a - AA_tmp**2 - BB_tmp**2 - self.l_link2**2
        denom_AA = np.sqrt(AA_tmp**2 + BB_tmp**2)
        
        # Calculate theta_22 - two possible solutions
        cc = np.arcsin(-CC_tmp / (denom_AA * 2 * self.l_link2))
        bb2 = np.arcsin(AA_tmp / denom_AA)
        
        # Triangle side length (since a = l/(2*cos(30°)), we have: l = a * sqrt(3))
        side_length = a * np.sqrt(3)
        
        # Solution 1: π - (cc - bb2)
        theta_22_rad_1 = np.pi - (cc - bb2)
        zz_1 = self.l_link1 * np.sin(th21_rad) + self.l_link2 * np.sin(theta_22_rad_1)
        yyy_1 = (a*a - (zz_1 - S[2])**2 - side_length*side_length + (zz_1 - x1[2])**2 + x1[1]**2) / (2 * x1[1])
        xxx_arg_1 = a*a - (zz_1 - S[2])**2 - yyy_1**2
        if xxx_arg_1 >= 0:
            xxx_1 = np.sqrt(xxx_arg_1)
            x2_1 = np.array([xxx_1, yyy_1, zz_1])
        else:
            x2_1 = None
        
        # Solution 2: -π + (cc - bb2)
        theta_22_rad_2 = -np.pi + (cc - bb2)
        zz_2 = self.l_link1 * np.sin(th21_rad) + self.l_link2 * np.sin(theta_22_rad_2)
        yyy_2 = (a*a - (zz_2 - S[2])**2 - side_length*side_length + (zz_2 - x1[2])**2 + x1[1]**2) / (2 * x1[1])
        xxx_arg_2 = a*a - (zz_2 - S[2])**2 - yyy_2**2
        if xxx_arg_2 >= 0:
            xxx_2 = np.sqrt(xxx_arg_2)
            x2_2 = np.array([xxx_2, yyy_2, zz_2])
        else:
            x2_2 = None
        
        # Choose the solution where z is closer to S[2]
        if x2_1 is not None and x2_2 is not None:
            if abs(zz_1 - S[2]) < abs(zz_2 - S[2]):
                return x2_1, theta_22_rad_1 * 180 / np.pi
            else:
                return x2_2, theta_22_rad_2 * 180 / np.pi
        elif x2_1 is not None:
            return x2_1, theta_22_rad_1 * 180 / np.pi
        elif x2_2 is not None:
            return x2_2, theta_22_rad_2 * 180 / np.pi
        else:
            # Neither solution is valid
            raise ValueError(f"Chain 2: No valid solution found for theta_21={theta_21:.2f}°")
    
    def _forward_chain3_constrained(self, theta_31, S, a):
        """Calculate x3 and theta_32 for chain 3 using platform circle constraint.
        Based on original calculate_position_3 from Learn content.
        
        Note: The circle constraint has TWO solutions. We choose the one where
        the platform center z-coordinate is closest to S[2].
        """
        th31_rad = theta_31 * np.pi / 180
        
        # Position calculations
        AA_tmp = -(self.l_base + self.l_link1 * np.cos(th31_rad))
        BB_tmp = S[2] - self.l_link1 * np.sin(th31_rad)
        CC_tmp = a*a - AA_tmp**2 - BB_tmp**2 - self.l_link2**2
        denom_AA = np.sqrt(AA_tmp**2 + BB_tmp**2)
        
        # Calculate theta_32 - two possible solutions
        cc = np.arcsin(-CC_tmp / (denom_AA * 2 * self.l_link2))
        bb2 = np.arcsin(AA_tmp / denom_AA)
        
        # Solution 1: π - (cc - bb2)
        theta_32_rad_1 = np.pi - (cc - bb2)
        zz_1 = self.l_link1 * np.sin(th31_rad) + self.l_link2 * np.sin(theta_32_rad_1)
        yy_arg_1 = a*a - (zz_1 - S[2])**2
        if yy_arg_1 >= 0:
            yy_1 = np.sqrt(yy_arg_1)
            x3_1 = np.array([-np.cos(30*np.pi/180)*yy_1, -np.sin(30*np.pi/180)*yy_1, zz_1])
        else:
            x3_1 = None
        
        # Solution 2: -π + (cc - bb2)
        theta_32_rad_2 = -np.pi + (cc - bb2)
        zz_2 = self.l_link1 * np.sin(th31_rad) + self.l_link2 * np.sin(theta_32_rad_2)
        yy_arg_2 = a*a - (zz_2 - S[2])**2
        if yy_arg_2 >= 0:
            yy_2 = np.sqrt(yy_arg_2)
            x3_2 = np.array([-np.cos(30*np.pi/180)*yy_2, -np.sin(30*np.pi/180)*yy_2, zz_2])
        else:
            x3_2 = None
        
        # Choose the solution where z is closer to S[2]
        if x3_1 is not None and x3_2 is not None:
            if abs(zz_1 - S[2]) < abs(zz_2 - S[2]):
                return x3_1, theta_32_rad_1 * 180 / np.pi
            else:
                return x3_2, theta_32_rad_2 * 180 / np.pi
        elif x3_1 is not None:
            return x3_1, theta_32_rad_1 * 180 / np.pi
        elif x3_2 is not None:
            return x3_2, theta_32_rad_2 * 180 / np.pi
        else:
            # Neither solution is valid
            raise ValueError(f"Chain 3: No valid solution found for theta_31={theta_31:.2f}°")
    
    def _forward_chain1(self, theta_11, theta_12):
        """Forward kinematics for chain 1.
        Base at [0, l_base, 0] pointing toward +Y axis.
        """
        th11 = theta_11 * np.pi / 180
        th12 = theta_12 * np.pi / 180
        
        # Base position for chain 1 (at 90° on base circle)
        base = np.array([0, self.l_base, 0])
        
        # Link 1 extends radially outward from base at angle theta_11 from horizontal
        # Direction is radial (away from origin) in the y-direction
        joint = base + np.array([0, 
                                self.l_link1 * np.cos(th11),
                                self.l_link1 * np.sin(th11)])
        
        # Link 2 continues from joint
        endpoint = joint + np.array([0,
                                     -self.l_link2 * np.cos(th12),
                                     self.l_link2 * np.sin(th12)])
        
        return endpoint
    
    def _forward_chain2(self, theta_21, theta_22):
        """Forward kinematics for chain 2.
        Base at 210° on base circle.
        """
        th21 = theta_21 * np.pi / 180
        th22 = theta_22 * np.pi / 180
        
        # Base position for chain 2 (at 210°)
        base_angle = 210 * np.pi / 180
        base = np.array([self.l_base * np.cos(base_angle),
                        self.l_base * np.sin(base_angle),
                        0])
        
        # Radial direction from origin to base (in x-y plane)
        radial_dir = np.array([np.cos(base_angle), np.sin(base_angle), 0])
        
        # Link 1 extends radially outward from base
        joint = base + radial_dir * self.l_link1 * np.cos(th21) + np.array([0, 0, self.l_link1 * np.sin(th21)])
        
        # Link 2 continues from joint
        endpoint = joint - radial_dir * self.l_link2 * np.cos(th22) + np.array([0, 0, self.l_link2 * np.sin(th22)])
        
        return endpoint
    
    def _forward_chain3(self, theta_31, theta_32):
        """Forward kinematics for chain 3.
        Base at 330° on base circle.
        """
        th31 = theta_31 * np.pi / 180
        th32 = theta_32 * np.pi / 180
        
        # Base position for chain 3 (at 330°)
        base_angle = 330 * np.pi / 180
        base = np.array([self.l_base * np.cos(base_angle),
                        self.l_base * np.sin(base_angle),
                        0])
        
        # Radial direction from origin to base (in x-y plane)
        radial_dir = np.array([np.cos(base_angle), np.sin(base_angle), 0])
        
        # Link 1 extends radially outward from base
        joint = base + radial_dir * self.l_link1 * np.cos(th31) + np.array([0, 0, self.l_link1 * np.sin(th31)])
        
        # Link 2 continues from joint
        endpoint = joint - radial_dir * self.l_link2 * np.cos(th32) + np.array([0, 0, self.l_link2 * np.sin(th32)])
        
        return endpoint
    
    def _plot_chain(self, base, endpoint, theta_1, color, label=None):
        """Plot a single actuator chain."""
        # Calculate joint position based on which chain
        if base[1] == self.l_base:  # Chain 1
            joint = np.array([0, 
                            self.l_link1*np.cos(theta_1*np.pi/180) + self.l_base,
                            self.l_link1*np.sin(theta_1*np.pi/180)])
        elif base[0] > 0:  # Chain 2
            joint = np.array([0.866*(self.l_link1*np.cos(theta_1*np.pi/180) + self.l_base),
                            -0.5*(self.l_link1*np.cos(theta_1*np.pi/180) + self.l_base),
                            self.l_link1*np.sin(theta_1*np.pi/180)])
        else:  # Chain 3
            joint = np.array([-0.866*(self.l_link1*np.cos(theta_1*np.pi/180) + self.l_base),
                            -0.5*(self.l_link1*np.cos(theta_1*np.pi/180) + self.l_base),
                            self.l_link1*np.sin(theta_1*np.pi/180)])
        
        # Plot first link with label (for legend)
        self.ax.plot([base[0], joint[0]], [base[1], joint[1]], [base[2], joint[2]], 
                    color=color, linewidth=2, label=label)
        # Plot second link
        self.ax.plot([joint[0], endpoint[0]], [joint[1], endpoint[1]], [joint[2], endpoint[2]], 
                    color=color, linewidth=2)
        self.ax.scatter(*joint, color=color, s=50)


# ========== Interactive Demo ==========
if __name__ == "__main__":
    print("="*60)
    print("Stewart Platform Interactive Demo")
    print("="*60)
    
    platform = StewartPlatform(l=25.9, l_base=10, l_link1=8, l_link2=10)
    
    while True:
        try:
            print("\n" + "-"*60)
            nrm_input = input("Normal vector [nx,ny,nz] (or 'quit'): ").strip()
            if nrm_input.lower() in ['quit', 'exit', 'q']:
                break
            
            pos_input = input("Center height z: ").strip()
            
            nrm = [float(x) for x in nrm_input.split(',')]
            pos = [0, 0, float(pos_input)]
            
            # Inverse kinematics
            ik = platform.inverse_kinematics(nrm, pos)
            print(f"\nServo Angles:")
            print(f"  Chain 1: θ11={ik['theta_11']:.2f}°, θ12={ik['theta_12']:.2f}°")
            print(f"  Chain 2: θ21={ik['theta_21']:.2f}°, θ22={ik['theta_22']:.2f}°")
            print(f"  Chain 3: θ31={ik['theta_31']:.2f}°, θ32={ik['theta_32']:.2f}°")
            
            # Verify with forward kinematics (using only primary angles)
            # FK will calculate secondary angles automatically
            fk = platform.forward_kinematics(ik['theta_11'], ik['theta_21'], ik['theta_31'],
                                            center_position=pos)
            
            e1 = np.linalg.norm(ik['x1'] - fk['x1'])
            e2 = np.linalg.norm(ik['x2'] - fk['x2'])
            e3 = np.linalg.norm(ik['x3'] - fk['x3'])
            print(f"FK Errors: {e1:.6f}, {e2:.6f}, {e3:.6f}")
            print(f"FK calculated secondary angles: θ12={fk['theta_12']:.2f}°, θ22={fk['theta_22']:.2f}°, θ32={fk['theta_32']:.2f}°")
            
            # Visualize
            platform.visualize(fk['x1'], fk['x2'], fk['x3'],
                             fk['theta_11'], fk['theta_12'],
                             fk['theta_21'], fk['theta_22'],
                             fk['theta_31'], fk['theta_32'])
            
        except Exception as e:
            print(f"Error: {e}")
    
    print("Exiting...")
    plt.close('all')
