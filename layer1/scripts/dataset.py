import glob
import os
import pandas as pd

class Dataset:

    def __init__(self, dir):
        self.dir = dir
        # If reading from result directory
        if "result" in self.dir:
            self._collect_progress()
            self._collect_bestfit()
        self._collect_archives()

    def _collect_progress(self):
        progress_file = f"{self.dir}/progress.dat"
        names = ["generation", "archive_size", "best_fit",
                 "sum_fit", "sum_novelty", "var_novelty"]
        self.progress = pd.read_csv(
            progress_file, delim_whitespace=True, names=names)

    def _collect_bestfit(self):
        bestfit_file = f"{self.dir}/bestfit.dat"
        names = ["generation", "evaluations", "best_fit"]
        self.bestfit = pd.read_csv(
            bestfit_file, delim_whitespace=True, names=names)

    def _collect_archives(self):
        if "baseline" in self.dir:
            archives_files = glob.glob(f"{self.dir}/archive*.dat")
            names = ["generation", "desc1", "desc2", "value", "fitness"] + [f"ctrl{i + 1}" for i in range(24)]
        elif "data" in self.dir:
            archives_files = glob.glob(f"{self.dir}/leg*.dat")
            names = ["generation", "desc1", "desc2", "desc3", "fitness"] + [f"ctrl{i + 1}" for i in range(3)]
        else:
            archives_files = glob.glob(f"{self.dir}/archive*.dat")
            names = ["generation", "desc1", "desc2", "value", "fitness"] + [f"ctrl{i + 1}" for i in range(12)]
        self.archives = dict()
        for file in archives_files:
            df = pd.read_csv(file, delim_whitespace=True, names=names)
            filename = os.path.splitext(os.path.basename(file))[0]
            self.archives.update({filename: df})

    def get_progress(self):
        return self.progress

    def get_bestfit(self):
        return self.bestfit

    def get_archives(self):
        return self.archives

    def get_archive_names(self):
        return list(self.archives.keys())

    def get_archive(self, archive):
        return self.archives.get(archive)
