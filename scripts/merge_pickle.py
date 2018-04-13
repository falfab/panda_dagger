import pickle
import glob, os

os.chdir("./dataset")
merged_vect = []
for f in glob.glob("*.pkl"):
    with open(f, 'rb') as pkl:
        for i in pickle.load(pkl):
            merged_vect.append(i)

with open('merged.pkl', 'wb') as pkl:
    pickle.dump(merged_vect, pkl)
