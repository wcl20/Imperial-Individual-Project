import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import subprocess
import os


class Visualizer:

    @staticmethod
    def show_archive(archive, name):
        # Create plot
        fig = plt.figure(figsize=(15, 10))
        ax = fig.add_subplot(111, aspect="equal")
        # Get behavior descriptors
        archive = archive[["desc1", "desc2", "value", "fitness"]]
        # Create color map
        norm = matplotlib.colors.Normalize()
        norm.autoscale(archive.iloc[:, 3])
        cm = matplotlib.cm.viridis
        # # Plot desc1 and desc2
        X = archive.iloc[:, 0]
        Y = archive.iloc[:, 1]
        U = np.cos(archive.iloc[:, 2])
        V = np.sin(archive.iloc[:, 2])
        ax.quiver(X, Y, U, V, units='x', color=cm(norm(archive.iloc[:, 3])))
        # Add x, y label
        ax.set_xlabel("Forward Displacement")
        ax.set_ylabel("Lateral Displacement")
        ax.set_xlim((0,1))
        ax.set_ylim((0,1))
        # Plot color bar
        plt.colorbar(matplotlib.cm.ScalarMappable(cmap=cm, norm=norm))
        plt.savefig(f"../results/figures/{name}.png")

    @staticmethod
    def show_versatility(archive, primitives):
        # Create plot
        fig = plt.figure(figsize=(20, 10))
        controllers = archive[[f"ctrl{i + 1}" for i in range(12)]].values.tolist()
        if os.path.exists("/git/sferes2/exp/layer1/results/controllers.csv"):
            controllers = pd.read_csv(f"/git/sferes2/exp/layer1/results/controllers.csv", header=None)
        else:
            for controller in controllers:
                arguments = list(map(str, controller))
                subprocess.call(["/git/sferes2/build/exp/layer1/example", *arguments])
            controllers = pd.read_csv(f"/git/sferes2/exp/layer1/temp.csv", header=None)
        # Iterate through each primitive repertoire
        for i, primitive in enumerate(primitives):
            ax = fig.add_subplot(2, 3, i + 1, aspect="equal")
            ax.set_title(f"Leg {i + 1}")
            # Get all the used primitives
            selected = controllers.iloc[:, i * 3 : (i + 1) * 3].drop_duplicates()
            # Swing action with smallest height
            swing = selected[(selected.iloc[:,2] > 0.5)].iloc[:, 2].min()
            # Stance action with 'smallest' height
            stance = selected[(selected.iloc[:,2] < 0.5)].iloc[:, 2].max()
            # Get all primitves with with height 1 or 0
            primitive = primitive[["desc1", "desc2", "desc3"]]
            primitive = primitive[(primitive["desc3"] >= swing) | (primitive["desc3"] <= stance)]
            ax.scatter(primitive.iloc[:, 0], primitive.iloc[:,1], alpha=0.1)
            ax.scatter(selected.iloc[:, 0], selected.iloc[:, 1], alpha=0.5)
            ax.set_xlabel(r"$p_s$")
            ax.set_ylabel(r"$p_e$")
            ax.set_xlim((0, 1))
            ax.set_ylim((0, 1))
            print(f"Versatility (Repertoire {i + 1}): {selected.shape[0] / primitive.shape[0]}")
        plt.savefig(f"../results/figures/versatility.png")

    @staticmethod
    def show_coverage(archive, baseline):
        # Create plot
        fig = plt.figure(figsize=(20, 10))
        ax = fig.add_subplot(111)
        generations = np.arange(0, 2100, 100)
        # Plot Hierarchical control
        size = [0] + archive.get_progress()["archive_size"].tolist()
        ax.plot(generations, size, label="Hierarchical")
        # Plot single layer control
        size = [0] + baseline.get_progress()["archive_size"].tolist()
        ax.plot(generations, size, label="Single Layer")
        ax.set_ylim(bottom=0)
        ax.set_xlim((0, 2000))
        ax.set_xlabel("Generation")
        ax.set_ylabel("Collection Size")
        ax.legend(loc="upper left")
        plt.savefig("../results/figures/coverage.png")

    @staticmethod
    def show_fitness(archive, baseline):
        # Create plot
        fig = plt.figure(figsize=(20, 10))
        ax = fig.add_subplot(111)
        generations = np.arange(100, 2100, 100)
        # Plot Hierarchical control
        total_fitness = archive.get_progress()["sum_fit"].to_numpy()
        size = archive.get_progress()["archive_size"].to_numpy()
        average_fitness = np.divide(total_fitness, size)
        ax.plot(generations, average_fitness, label="Hierarchical")
        # Plot single layer control
        total_fitness = baseline.get_progress()["sum_fit"].to_numpy()
        size = baseline.get_progress()["archive_size"].to_numpy()
        average_fitness = np.divide(total_fitness, size)
        ax.plot(generations, average_fitness, label="Single Layer")
        ax.set_xlim((0, 2000))
        ax.set_xlabel("Generation")
        ax.set_ylabel("Average Fitness")
        ax.legend(loc="upper left")
        plt.savefig("../results/figures/fitness.png")
