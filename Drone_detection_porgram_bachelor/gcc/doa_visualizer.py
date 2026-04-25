import matplotlib.pyplot as plt
from gcc.doa_estimator import DOAEstimator
from receivers.stm32_usb_receiver import STM32UsbReceiver
import numpy as np

class DOAVisualizer():

    def __init__(self):
        pass


    def visualize_doa(self, estimated_deg : DOAEstimator, estimated_rad : DOAEstimator, receiver_time : STM32UsbReceiver):
        fig, ax = plt.subplots(figsize=(6, 6))

        center_x = 0.5
        center_y = 0.5

        doa_radius = 0.3
        station_radius = 0.04

        # Outer DOA circle
        doa_circle = plt.Circle(
            (center_x, center_y),
            doa_radius,
            color="blue",
            fill=False,
            linewidth=2,
            label="DOA detection area"
        )

        # Station circle
        station_circle = plt.Circle(
            (center_x, center_y),
            station_radius,
            color="black",
            fill=True,
            label="Station"
        )

        ax.add_patch(doa_circle)
        ax.add_patch(station_circle)

        # Use radians for trig
        doa_x = center_x + doa_radius * np.cos(estimated_rad)
        doa_y = center_y + doa_radius * np.sin(estimated_rad)

        # Direction line
        ax.plot(
            [center_x, doa_x],
            [center_y, doa_y],
            color="red",
            linewidth=2
        )

        # DOA point
        ax.scatter(
            doa_x,
            doa_y,
            color="red",
            s=80,
            zorder=5,
            label="Estimated DOA"
        )

        # Angle label in degrees
        ax.text(
            doa_x,
            doa_y,
            f"{estimated_deg:.1f}°",
            fontsize=12,
            ha="left",
            va="bottom"
        )

        ax.set_aspect("equal")
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_title("Direction of Arrival Visualization")
        ax.grid(True)
        ax.legend()
        plt.savefig(f'debug_figures/Debug_doa_estimation{receiver_time}.png')
        plt.show()