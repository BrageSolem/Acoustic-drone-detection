import numpy as np
import pandas as pd
import sklearn as sk

import librosa
import joblib
import mfcc_extraction


class AccumulativeModel:
    """
    Accumulative Model trained on audio of white noise and drone audio,
    can be updated with new data, without the need to retrain the model from scratch.
    """

    def __init__(self, model_path):
        self.model = joblib.load(model_path)
        self.mfcc_extractor = mfcc_extraction.MFCCExtractor()

        self.labels = ["Drone", "Not_a_drone"]

    def train_model(self):
        """
        Train the accumulative model based on data obtained at:
        https://www.kaggle.com/datasets/yehiellevi/dataset-balanced-n-weighted-final

        Not run in the programed, used to save a pre trained accumulative model for speed purposes,
        since training the model takes a long time. The trained model is saved as "accumulativemodel.joblib"

        :return:
        """
        x_acum = []
        y_acum = []

        labcsv = pd.read_csv("audio_metadata_shuffled.csv")

        for unused, row in labcsv.iterrows():
            if row["class"] == "Drone":
                label = 0
            else:
                label = 1

            file_path = os.path.join("datasett", row["slice_file_name"])
            temp_extractor = mfcc_extraction.MFCCExtractor()

            x_acum.append(temp_extractor.extract_features(file_path))
            y_acum.append(label)

        acum_model = sk.tree.DecisionTreeClassifier()
        acum_model.fit(x_acum, y_acum)
        return acum_model

    def check_label_validy(self, label):
        """
        Check if label is valid and converts to correct int value

        :param label: label to be checked
        :return: 0 if label is "Drone", 1 if label is "Not_a_drone"
        """
        if label not in self.labels:
            raise ValueError(f"Invalid label: {label}. Valid labels are: {self.labels}")
        if label == self.labels[0]:
            return 0
        else:
            return 1

    def update_training_internal_extract(self, audio_file, label):
        """
        Add new data to the accumulative Model.

        The features are extracted internally in this function, using the mfcc_extractor.

        :param audio_file: a .wav file
        :param label: Label of audio file, Has to be ether "Drone" or "Not_a_drone"
        :return:
        """
        temp_list = [self.mfcc_extractor.extract_features(audio_file)]
        temp_label = self.check_label_validy(label)
        self.model.partial_fit(temp_list, temp_label)

    def update_training(self, extracted_audio_file, label):
        """
        Add new data to the accumulative Model.

        The features are extracted externally,
        and passed as a list of accumulative values for a audio file.

        :param extracted_audio_file: A list of accumulative values for a audio file
        :param label: Label of audio file, Has to be ether "Drone" or "Not_a_drone"
        :return: Void
        """
        temp_list = [self.mfcc_extractor.extract_features(extracted_audio_file)]
        self.model.partial_fit(temp_list, [label])

    def predict_internal_extract(self, audio_file):
        """
        Predict the label of an audio file

        The features are extracted internally in this function, using the mfcc_extractor.

        :param audio_file: a .wav file
        :return: Label of audio file, ether "Drone" or "Not_a_drone
        """
        temp_list = [self.mfcc_extractor.extract_features(audio_file)]
        acum_prediction = self.model.predict(temp_list)
        return self.labels[acum_prediction[0]]

    def acum_predict(self, extracted_audio_file):
        """
        Predict the label of an audio file

        The features are extracted externally,
        and passed as a list of accumulative values for a audio file.
        :param extracted_audio_file: A list of accumulative values for a audio file
        :return: Label of audio file, ether "Drone" or "Not_a_drone
        """
        return self.labels[self.model.predict([extracted_audio_file])[0]]

