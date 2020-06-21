import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import numpy as np
import math
import subprocess
import os

class Visualizer:

    @staticmethod
    def show_archive(archive):
        # Create plot
        fig = plt.figure(figsize=(18, 12))

        gs = GridSpec(2, 3, figure=fig)

        leg = archive.leg
        archive = archive.df
        slice = archive[abs(archive["desc1"] - 0.2) < 0.02]
        heights = slice[(abs(slice["desc2"] - 0.8) < 0.02)]
        widths = slice[(abs(slice["desc3"] - 0.8) < 0.02)]

        # Show archive
        ax = fig.add_subplot(gs[0, 0], projection="3d")
        ax.scatter(archive.iloc[:, 1], archive.iloc[:, 2], archive.iloc[:, 3], alpha=0.01)
        ax.scatter(slice.iloc[:, 1], slice.iloc[:, 2], slice.iloc[:, 3], alpha=0.5)
        # Labels
        ax.set_xlabel(r"$p_s$")
        ax.set_ylabel(r"$p_e$")
        ax.set_zlabel(r"$h$")

        # Show archive slice
        ax = fig.add_subplot(gs[1, 0], aspect="equal")
        ax.scatter(slice.iloc[:, 2], slice.iloc[:, 3])
        ax.scatter(heights.iloc[:, 2], heights.iloc[:, 3], c="#FF7F0E")
        ax.scatter(widths.iloc[:, 2], widths.iloc[:, 3], c="#FF7F0E")
        # Axis label
        ax.set_xlabel(r"$p_e$")
        ax.set_ylabel(r"$h$")

        # Show heights trajectories
        ax = fig.add_subplot(gs[0, 1:])
        controllers = heights[[f'ctrl{i + 1}' for i in range(3)]].values.tolist()
        ax = Visualizer.show_controllers(leg, controllers, ax)

        # Show width trajectories
        ax = fig.add_subplot(gs[1, 1:])
        controllers = widths[[f'ctrl{i + 1}' for i in range(3)]].values.tolist()
        ax = Visualizer.show_controllers(leg, controllers, ax)

        plt.savefig("../results/figures/archive.png")

    @staticmethod
    def show_archives(archives, titles, name):
        # Create plot
        fig = plt.figure(figsize=(20, 10))
        col = math.ceil(len(archives) / 2)
        for i, archive in enumerate(archives):
            ax = fig.add_subplot(2, col, i + 1, projection="3d")
            df = archive.df
            df = df[["desc1", "desc2", "desc3"]]
            # Plot
            ax.scatter(df.iloc[:, 0], df.iloc[:, 1], df.iloc[:, 2], alpha=0.05)
            # Labels
            ax.set_title(titles[i])
            ax.set_xlabel(r"$p_s$")
            ax.set_ylabel(r"$p_e$")
            ax.set_zlabel(r"$h$")
        plt.savefig(f"../results/figures/{name}.png")

    @staticmethod
    def show_coverages(archives, random, population):

        def compute_coverage(archive):
            # Create grid
            resolution = 10
            grid = np.zeros((resolution, resolution, resolution))
            for i, indiv in archive.df.iterrows():
                desc1 = int(round(indiv["desc1"] * (resolution - 1)))
                desc2 = int(round(indiv["desc2"] * (resolution - 1)))
                desc3 = int(round(indiv["desc3"] * (resolution - 1)))
                grid[desc1][desc2][desc3] = 1
            unique, count = np.unique(grid, return_counts=True)
            coverage = dict(zip(unique, count))
            return coverage[1.0] / (coverage[0.0] + coverage[1.0])

        # Create plot
        fig = plt.figure(figsize=(20, 10))
        ax = fig.add_subplot(111)
        generations = np.arange(0, (len(archives) + 1) * 100, 100)
        ax.set_xlabel("Generation")
        ax.set_xlim((0, 2000))

        # Compute coverages
        coverages = [0] + [compute_coverage(archive) for archive in archives]
        ax.plot(generations, coverages, label="Coverage")
        for i, coverage in enumerate(coverages):
            print(f"Archive coverage ({i * 100}): {coverage}")
        # Coverage of random repertoire
        random = compute_coverage(random)
        ax.plot(generations, [random] * len(generations), label="Random", linestyle='dashed')
        print(f"Random coverage: {random}")
        # Converage of ns population
        population = compute_coverage(population)
        ax.plot(generations, [population] * len(generations), label="Population", linestyle='dashed')
        print(f"Population coverage: {population}")
        ax.set_ylabel("Coverage")
        ax.set_ylim((0, 1))
        ax.legend(loc="upper left")

        # Create another plot with same x-axis
        ax2 = ax.twinx()
        # Compute collection size
        sizes = [0] + [archive.df.shape[0] for archive in archives]
        ax2.plot(generations, sizes, c="#D62728", label="Collection Size")
        for i, size in enumerate(sizes):
            print(f"Archive size ({i * 100}): {size}")
        ax2.set_ylabel("Collection Size")
        ax2.set_ylim(bottom=0)

        # Legend
        ax2.legend( loc="upper right")
        fig.tight_layout()
        plt.savefig("../results/figures/coverage.png")


    @staticmethod
    def show_controllers(leg, ctrls, ax):
        # Plot controllers
        for ctrl in ctrls:
            # Run each controller
            arguments = list(map(str, [leg] + ctrl))
            subprocess.call(["/git/sferes2/build/exp/layer0/example", *arguments])
        if os.path.exists("/git/sferes2/exp/layer0/temp.csv"):
            positions = pd.read_csv("/git/sferes2/exp/layer0/temp.csv")
            os.remove("/git/sferes2/exp/layer0/temp.csv")
            ax.scatter(positions.iloc[:, 0], positions.iloc[:, 2])
        ax.scatter([0], [0], marker="X", s=80, label="origin")
        # Axis label
        ax.set_xlabel("x")
        ax.set_ylabel("z")
        ax.set_xlim((-0.06, 0.06))
        ax.legend(loc="upper right")
        return ax
