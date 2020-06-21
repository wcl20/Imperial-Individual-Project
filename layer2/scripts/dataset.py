import glob
import os
import pandas as pd

class Dataset:

    def __init__(self, dir):
        self.dir = dir
        self._collect_distances()
        self._collect_models()

    def _collect_models(self):
        self.models = dict()
        model_files = glob.glob(f"{self.dir}/models/gp_*[0-9].dat")
        names = ["sample", "mean", "variance", "acquisition"]
        for file in model_files:
            df = pd.read_csv(file, delim_whitespace=True)
            df.columns = names
            filename = os.path.splitext(os.path.basename(file))[0]
            self.models.update({filename: df})

    def _collect_distances(self):
        self.distances = dict()
        distance_files = glob.glob(f"{self.dir}/distances/*.csv")
        for file in distance_files:
            df = pd.read_csv(file, names=["time", "distance", "x"])
            filename = os.path.splitext(os.path.basename(file))[0]
            self.distances.update({filename: df})

    def get_models(self):
        return self.models

    def get_models_names(self):
        return list(self.models.keys())

    def get_model(self, model):
        return self.models.get(model)

    def get_distances(self):
        return self.distances
