import cv2
import numpy as np


class KalmanFilter:
    def __init__(self):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    def predict_coords(self, coords):
        ''' This function estimates the position of the object'''
        predicted_coords = []
        for coord in coords:
            measured = np.array([[np.float32(coord[0])], [np.float32(coord[1])]])
            self.kf.correct(measured)
            predicted = self.kf.predict()
            x, y = int(predicted[0]), int(predicted[1])
            predicted_coords.append((x, y))

        return predicted_coords

    def predict(self, data):

        measured = np.array([[np.float32(data[0])], [np.float32(data[1])]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        return int(predicted[0]), int(predicted[1])
