import glob
import os
import pandas as pd

class Dataset:

    def __init__(self, leg, dir):
        self.dir = dir
        self.leg = leg
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
        archives_files = glob.glob(f"{self.dir}/archive*.dat")
        archives_files += glob.glob(f"{self.dir}/parent*.dat")
        archives_files += glob.glob(f"{self.dir}/offspring*.dat")
        self.archives = dict()
        for file in archives_files:
            names = ["generation", "desc1", "desc2", "desc3",
                     "fitness", "ctrl1", "ctrl2", "ctrl3"]
            df = pd.read_csv(file, delim_whitespace=True, names=names)
            filename = os.path.splitext(os.path.basename(file))[0]
            self.archives.update({filename: DataFrame(self.leg, df)})

    def get_progress(self):
        return self.progress

    def get_bestfit(self):
        return self.bestfit

    def get_archive_names(self):
        return list(self.archives.keys())

    def get_archive(self, archive):
        return self.archives.get(archive)


class DataFrame:

    def __init__(self, leg, df):
        self.leg = leg
        self.df = df
