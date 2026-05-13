import numpy as np
import pandas as pd
import sklearn as sk

import librosa
import joblib
import mfcc_extraction


class AccumulativeModel:
    """
    Accumulative Model trained on audio of white noise and drone audio,
    can not be updated with new data, to update the model with new data, the model needs to be retrained from scratch.

    The Default model is trained on both public data and audio files from microphone array, under debug_figures,
     only the old training method for public data given files under debug figures contain more then audio files.
    """

    def __init__(self, model_path="AccumulativeModel.joblib"):
        """
        Initialize the AccumulativeModel class

        Defaults model to "Accumulativemodel.joblib"

        :param model_path: The file path of the accumulative model, expects a .joblib file
        """
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

        acum_model = sk.tree.DecisionTreeClassifier(max_depth = 6,min_samples_leaf=20,random_state=42)
        acum_model.fit(x_acum, y_acum)
        return acum_model


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

