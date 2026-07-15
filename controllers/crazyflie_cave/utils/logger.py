# logger.py
from utils.motor_setup import MOTOR_SIGNS
import matplotlib
matplotlib.use("Agg")  # Saves plots without opening a GUI window in Webots
import matplotlib.pyplot as plt

class Logger:
    def __init__(self, print_interval=0.1):
        """
        Initialize logger.

        Args:
            print_interval (float): Minimum time (seconds) between prints.
        """
        self.print_interval = print_interval
        self.time_since_last_print = 0.0
        self.motor_signs = MOTOR_SIGNS

        # Step-response data storage
        self.step_time = []
        self.step_z = []
        self.step_z_ref = []
        self.step_plot_created = False

    def log(self, state, z_ref, dt, sim_time):
        """
        Log the sensor state if the print interval has passed.

        Args:
            state (dict): Sensor state dictionary from CrazyflieSensors.read()
            dt (float): Time since last loop (seconds)
        """
        self.time_since_last_print += dt



        if self.time_since_last_print >= self.print_interval:
            pos_x, pos_y, pos_z = state["position"]
            vel_x, vel_y, vel_z = state["velocity"]
            roll, pitch, yaw = state["attitude"]
            roll_rate, pitch_rate, yaw_rate = state["attitude_rates"]

            # Altitude error (for feedback)
            error_z = z_ref - pos_z

            # Print sensor readings:
            print(f"Time: {sim_time:.2f} s --- Sensor Log ---")
            print(f"Current altitude: {pos_z:.3f} m, Vertical velocity: {vel_z:.3f} m/s, Altitude error: {error_z:.3f} m")
            print(f"Position X: {pos_x:.3f}, Position Y: {pos_y:.3f}, Position Z: {pos_z:.3f}")
            print(f"Velocity X: {vel_x:.3f}, Velocity Y: {vel_y:.3f}, Velocity Z: {vel_z:.3f}")
            print(f"Roll: {roll:.3f}, Pitch: {pitch:.3f}, Yaw: {yaw:.3f}")
            print(f"Roll rate: {roll_rate:.3f}, Pitch rate: {pitch_rate:.3f}, Yaw rate: {yaw_rate:.3f}")
            print("-----------------\n")

            self.time_since_last_print = 0.0


    def record_step_response(self, state, reference, sim_time, max_log_time=30.0):
        """
        Store altitude step-response data up to max_log_time seconds.
        """
        if sim_time <= max_log_time:
            pos_z = state["position"][2]

            self.step_time.append(sim_time)
            self.step_z.append(pos_z)
            self.step_z_ref.append(reference["z"])


    def finalize_step_response(self, filename="step_response_30s.png", target=1.0, tolerance=0.05):
        """
        Create standardized step-response plot and print:
        - rise time from 10% to 90%
        - maximum overshoot in %
        - settling time within ±5%
        """

        if self.step_plot_created:
            return

        self.step_plot_created = True

        if len(self.step_time) < 2:
            print("Not enough step-response data to create plot.")
            return

        # Convert time so plot starts at t = 0
        t0 = self.step_time[0]
        t = [ti - t0 for ti in self.step_time]
        z = self.step_z
        z_ref = self.step_z_ref

        # ------------------------------------------------------------
        # Rise time: time from 10% to 90% of step height
        # ------------------------------------------------------------
        z_initial = z[0]
        z_final = target

        z_10 = z_initial + 0.10 * (z_final - z_initial)
        z_90 = z_initial + 0.90 * (z_final - z_initial)

        def first_crossing_time(level):
            for i in range(1, len(z)):
                if z[i - 1] <= level <= z[i]:
                    # Linear interpolation between samples
                    dz = z[i] - z[i - 1]
                    if abs(dz) < 1e-9:
                        return t[i]
                    ratio = (level - z[i - 1]) / dz
                    return t[i - 1] + ratio * (t[i] - t[i - 1])
            return None

        t_10 = first_crossing_time(z_10)
        t_90 = first_crossing_time(z_90)

        if t_10 is not None and t_90 is not None:
            rise_time = t_90 - t_10
        else:
            rise_time = None

        # ------------------------------------------------------------
        # Maximum overshoot
        # ------------------------------------------------------------
        max_z = max(z)
        max_overshoot = max(0.0, ((max_z - target) / target) * 100.0)

        # ------------------------------------------------------------
        # Settling time within ±5%
        # Definition: first time after which all remaining samples of the altitude response stay inside the ±5% band.
        # ------------------------------------------------------------
        band = tolerance * abs(target)
        lower_bound = target - band
        upper_bound = target + band

        inside_band = [
            lower_bound <= zi <= upper_bound
            for zi in z
        ]

        last_outside_index = -1
        for i, inside in enumerate(inside_band):
            if not inside:
                last_outside_index = i

        if last_outside_index == -1:
            settling_time = t[0]
        elif last_outside_index < len(t) - 1:
            settling_time = t[last_outside_index + 1]
        else:
            settling_time = None

        # ------------------------------------------------------------
        # Print standardized metrics
        # ------------------------------------------------------------
        rise_text = f"{rise_time:.3f} s" if rise_time is not None else "N/A"
        settling_text = f"{settling_time:.3f} s" if settling_time is not None else "Not settled within 30 s"

        print("\n========== Step-response results ==========")
        print(f"Rise time, 10% to 90%: {rise_text}")
        print(f"Maximum overshoot:      {max_overshoot:.2f} %")
        print(f"Settling time, ±5%:     {settling_text}")
        print(f"Plot saved as:          {filename}")
        print("===========================================\n")

        # ------------------------------------------------------------
        # Create standardized plot
        # ------------------------------------------------------------
        plt.figure(figsize=(8, 5))

        plt.plot(t, z, linewidth=2.0, label="Measured altitude z(t)")
        plt.plot(t, z_ref, "k--", linewidth=1.5, label="Reference altitude")

        plt.axhline(lower_bound, color="green", linestyle=":", linewidth=1.2, label="±5% settling band")
        plt.axhline(upper_bound, color="green", linestyle=":", linewidth=1.2)

        plt.xlim(0, 30)
        plt.xlabel("Time [s]")
        plt.ylabel("Altitude [m]")
        plt.title("Crazyflie altitude step response")
        plt.grid(True)
        plt.legend()

        metric_text = (
            f"Rise time: {rise_text}\n"
            f"Max overshoot: {max_overshoot:.2f} %\n"
            f"Settling time ±5%: {settling_text}"
        )

        plt.text(
            0.98,
            0.02,
            metric_text,
            transform=plt.gca().transAxes,
            fontsize=10,
            verticalalignment="bottom",
            horizontalalignment="right",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8)
        )

        plt.tight_layout()
        plt.savefig(filename, dpi=300)
        plt.close()


    def outputMotorVelocities(self, velocities):
            print(f"Motor velocities (with signs applied):")
            for i, (vel, sign) in enumerate(zip(velocities, self.motor_signs), start=1):
                print(f"m{i}: {sign*vel:.2f} rad/s")
