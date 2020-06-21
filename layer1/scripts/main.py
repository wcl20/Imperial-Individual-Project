from dataset import Dataset
from visualizer import Visualizer

if __name__ == '__main__':

    # Load data
    dataset = Dataset("../results/2020-06-05")
    baseline = Dataset("../results/baseline")

    # Visualize archive
    Visualizer.show_archive(dataset.get_archive("archive_1900"), "archive")

    # Visualize baseline
    Visualizer.show_archive(baseline.get_archive("archive_1900"), "baseline")

    # Visualize coverage
    Visualizer.show_coverage(dataset, baseline)

    # Visualize Fitness
    Visualizer.show_fitness(dataset, baseline)

    # Get primitive behavior archive
    primitives = [Dataset("../data/archive1900").get_archive(f"leg{i}") for i in range(6)]
    archive = dataset.get_archive("archive_1900")
    Visualizer.show_versatility(archive, primitives)
