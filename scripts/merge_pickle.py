"""
merge pickle file created by the dataset_generator and located inside
the dataset folder
"""
import pickle
import glob
import os


def main():
    """merge file inside dataset"""
    os.chdir("../dataset")
    merged_vect = []

    # read all the dataset file
    for pkl in glob.glob("*.pkl"):
        with open(pkl, 'rb') as pkl:
            for i in pickle.load(pkl):
                merged_vect.append(i)

    # merge everything inside a single file
    with open('merged.pkl', 'wb') as pkl:
        pickle.dump(merged_vect, pkl)

    # remove old dataset
    for dataset in glob.glob("dataset*.pkl"):
        os.remove(dataset)

if __name__ == "__Main__":
    main()
