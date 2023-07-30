import pickle

# create some data
data = [1, 2, 3, 4, 5]

filepath = r'TEST_TRAINING_DATA/myfile.pkl'
with open(filepath, 'wb') as f:
    pickle.dump(data, f)
