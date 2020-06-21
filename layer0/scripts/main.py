from dataset import Dataset
from visualizer import Visualizer

if __name__ == '__main__':

    # Visualize leg archives
    print("Archives --------------------------------------------------")
    datasets = [Dataset(i, f"../results/NS/leg{i}") for i in range(6)]
    archives = [dataset.get_archive("archive_1900") for dataset in datasets]
    labels = [f"Leg {i + 1}" for i in range(6)]
    Visualizer.show_archives(archives, labels, "archives")
    for i, archive in enumerate(archives):
        print(f"Archive {i + 1}: {archive.df.shape[0]}")

    # Dataset for leg 1
    dataset = Dataset(1, "../results/NS/leg1")

    # Visualize detailed view
    print("Archive --------------------------------------------------")
    archive = dataset.get_archive("archive_1900")
    Visualizer.show_archive(archive)

    # NS Population
    population = dataset.get_archive("parent_0")

    # Random Repertoire
    random = Dataset(1, "../results/random/leg1").get_archive("offspring_0")

    # Visualize archive evolution
    print("Archive Evolution --------------------------------------------------")
    archives = [dataset.get_archive(f"archive_{i}") for i in [0, 100, 200, 500, 1000, 1900]] + [population, random]
    labels = [f"Generation {i}" for i in [0, 100, 200, 500, 1000, 2000]] + ["Population", "Random"]
    Visualizer.show_archives(archives, labels, "archiveEvolution")

    # Visualize coverage
    print("Archive Coverage --------------------------------------------------")
    archives = [dataset.get_archive(f"archive_{i}") for i in range(0, 2000, 100)]
    Visualizer.show_coverages(archives, random, population)
