import pickle

filepath = r'TEST_TRAINING_DATA/rfPickle.pkl'
with open(filepath, 'rb') as f:
    unpickled_data = pickle.load(f)

print(unpickled_data)
