import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import math


class Visualizer:

    @staticmethod
    def show_models(dataset):
        # Create plot
        fig = plt.figure(figsize=(20, 10))
        # Get number of models
        n = len(dataset.get_models())
        models = [dataset.get_model(f"gp_{i}") for i in range(n)]
        for i, model in enumerate(models):
            ax = fig.add_subplot(4, 4, i + 1)
            ax.set_title(f"Iteration {i + 1}")
            # Plot model
            samples = model["sample"] * 0.4
            means = model["mean"]
            lower = model["mean"] - model["variance"]
            upper = model["mean"] + model["variance"]
            ax.plot(samples, means)
            # Plot uncertainty
            ax.fill_between(samples, lower, upper, color='gray', alpha=0.2)
            # Create another plot with same x-axis
            ax2 = ax.twinx()
            ax2.plot(samples, model["acquisition"], c="#FF7F0E")
            ax2.yaxis.set_visible(False)
        # Add legend
        ax = fig.add_subplot(4, 2, 8)
        ax.plot([], [], label="Model")
        ax.plot([], [], label="Acquisition")
        ax.axis("off")
        ax.legend(loc='center', shadow=True, fontsize='x-large')
        fig.tight_layout()
        plt.savefig("../results/figures/model.png")

    @staticmethod
    def show_distances(dataset):
        # Create plot
        fig = plt.figure(figsize=(20, 10))
        ax = fig.add_subplot(111)
        # Iterate files
        for filename, df in dataset.get_distances().items():
            ax.plot(df["time"], df["distance"], label=filename)
        ax.set_xlabel("Time")
        ax.set_ylabel("Distance From Goal")
        ax.set_ylim(bottom=0)
        ax.legend(loc="upper right")
        plt.savefig("../results/figures/distance.png")

    @staticmethod
    def show_energy(dataset):
        # Create plot
        fig = plt.figure(figsize=(20, 10))
        ax = fig.add_subplot(111)
        # Iterate files
        for filename, df in dataset.get_distances().items():
            # Get accumulated energy. Energy = 1 - x.
            energy = np.insert((1 - df["x"]).to_numpy(), 0, 0)
            energy = np.cumsum(energy)
            energy = np.insert(energy, -1, energy[-1])
            time = [0] + df["time"].tolist() + [133]
            print(energy)
            print(f"{filename} average x: {df['x'].mean()}")
            ax.plot(time, energy, label=filename)
        ax.set_xlabel("Time")
        ax.set_ylabel("Total Energy")
        ax.legend(loc="upper left")
        plt.savefig("../results/figures/energy.png")
