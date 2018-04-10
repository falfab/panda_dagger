import os
import pickle


class DatasetHandler():
    def __init__(self):
        self.dataset = []
        if not os.path.exists("../dataset"):
            os.mkdir("../dataset")

    def append(self, img_pos_tuple):
        self.dataset.append(img_pos_tuple)

    def save(self):
        with open("../dataset/dataset.pkl", mode="wb") as fd:
            pickle.dump(self.dataset, fd)
