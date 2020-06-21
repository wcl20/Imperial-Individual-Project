from dataset import Dataset
from visualizer import Visualizer

if __name__ == '__main__':

    dataset = Dataset("../results")
    Visualizer.show_models(dataset)
    Visualizer.show_distances(dataset)
    Visualizer.show_energy(dataset)
