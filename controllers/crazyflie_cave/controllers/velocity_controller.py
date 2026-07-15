import numpy as np

class VelocityController:
    """
    PID velocity controller for a quadcopter in Webots
    Outputs desired roll, pitch, and thrust commands
    """
    def __init__(self, Kp_xy, Kd_xy, Kp_z, Ki_z, Kd_z, dt, F_hover, tau_d):
        """
        Args:
            Kp_xy: proportional gain for lateral velocity (roll/pitch)
            Kd_xy: derivative gain for lateral velocity
            Kp_z: proportional gain for vertical velocity
            Ki_z: integral gain for vertical velocity
            Kd_z: derivative gain for vertical velocity
            dt: timestep in seconds
        """
        # Lateral
        self.Kp_xy = Kp_xy
        self.Kd_xy = Kd_xy
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        # Vertical - PID Controller
        self.F_hover = F_hover
        self.Kp_z = Kp_z
        self.Ki_z = Ki_z
        self.Kd_z = Kd_z
        self.prev_error_z = 0.0
        self.integral_z = 0.0
        self.deriv_z = 0.0
        self.tau_d = tau_d   # derivative filter time constant

        # States for general-form difference equation
        # e_z_1 = e[k-1], e_z_2 = e[k-2]
        self.e_z_1 = 0.0
        self.e_z_2 = 0.0

        # u_z_1 = u[k-1], u_z_2 = u[k-2]
        # u is the delta controller output added to baseline thrust for the total control signal
        self.u_z_1 = 0.0
        self.u_z_2 = 0.0

        self.dt = dt

    def compute_control_signals(self, vx_ref, vy_ref, vz_ref, vx, vy, vz):
        """
        Args:
            vx_ref, vy_ref, vz_ref: desired velocities (m/s)
            vx, vy, vz: current velocities (m/s)

        Returns:
            desired_roll
            desired_pitch
            thrust_command
        """

        # --- Lateral PID for desired roll/pitch ---
        error_x = vx_ref - vx
        error_y = vy_ref - vy

        # Deadband to prevent creeping drift
        if abs(error_x) < 0.01:
            error_x = 0.0
        if abs(error_y) < 0.01:
            error_y = 0.0

        deriv_x = (error_x - self.prev_error_x) / self.dt
        deriv_y = (error_y - self.prev_error_y) / self.dt

        # Clip lateral velocity errors to [-1, 1]
        error_x_clipped = np.clip(error_x, -1, 1)
        error_y_clipped = np.clip(error_y, -1, 1)

        desired_pitch = (self.Kp_xy * error_x_clipped + self.Kd_xy * deriv_x)
        desired_roll  = -(self.Kp_xy * error_y_clipped + self.Kd_xy * deriv_y)

        self.prev_error_x = error_x
        self.prev_error_y = error_y

        # --- Aufgabe 2a: Vertical velocity controller  ---
        
        # Comparison:
        #
        # thrust_command_pid    = original PID implementation
        # thrust_command        = equivalent general-form difference equation

        # ============================================================
        # Reference PID implementation (used only for comparison)

        error_z = vz_ref - vz

        self.integral_z += error_z * self.dt
        self.integral_z = np.clip(self.integral_z, -2.0, 2.0)

        deriv_raw = (error_z - self.prev_error_z) / self.dt

        alpha = self.tau_d / (self.tau_d + self.dt)

        self.deriv_z = (
            alpha * self.deriv_z
            + (1.0 - alpha) * deriv_raw
        )

        deriv_z = self.deriv_z

        thrust_command_pid = (
            self.F_hover
            + self.Kp_z * error_z
            + self.Ki_z * self.integral_z
            + self.Kd_z * deriv_z
        )

        self.prev_error_z = error_z

        # -----------------------------------------------------
        # Aufgabe 2a: Implement your controller by replacing the coefficients with those obtained after by discretization after z-transform.
        #
        # Difference equation:
        #
        # u[k] = b0 e[k] + b1 e[k-1] + b2 e[k-2] - a1 u[k-1] - a2 u[k-2]
        #

        # Coefficients (0 as Placeholders)
        b0 = 0
        b1 = 0
        b2 = 0
        a1 = 0
        a2 = 0

        # Difference equation
        u_z_k = 0 # Placeholder
        
        # Control signal - thrust command
        thrust_command = 0.001 # Placeholder

        # Update difference-equation states
        # self.e_z_2 = ...
        # ... Placeholders ...

        # Compare PID implementation (thrust_command_pid) with equivalent difference equation (thrust_command).
        print(f"Difference: {thrust_command_pid - thrust_command:.3e}")

        # If you want to actually use the thrust_command_pid output,
        # uncomment this line:
        #
        #thrust_command = thrust_command_pid

        return desired_roll, desired_pitch, thrust_command
