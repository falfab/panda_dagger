import os
import pickle


class DatasetHandler():
    def __init__(self, num=None):
        self.dataset = []
        self.outfile = "./dataset/dataset%d.pkl" % num if num != None else "./dataset/dataset.pkl"
        if not os.path.exists("./dataset"):
            os.mkdir("./dataset")

    def append(self, img_pos_tuple):
        self.dataset.append(img_pos_tuple)

    def save(self):
        with open(self.outfile, mode="wb") as fd:
            pickle.dump(self.dataset, fd)
