import os
import numpy as np
import librosa

from tensorflow.keras.models import Sequential,load_model
from tensorflow.keras.layers import Conv2D, MaxPooling2D
from tensorflow.keras.layers import Dense, Flatten, Dropout
from tensorflow.keras.utils import to_categorical


# noinspection PyInterpreter
class CnnModel:
    """
    Cnn Model trained on a dataset of drone and white noise audio,
    can be updated with new data, without the need to retrain the model from scratch.
    """

    def __init__(self):
        self.model = load_model("CNNmodel.keras")

        self.labels = ["Drone", "Not_a_drone"]

    def extract_mfcc(self,file_path, n_mfcc=40, max_len=174):
        """
        Extract MFCC features from a .wav file, and pad or truncate them to a fixed length.
        For standardization of the input to the CNN model.

        :param file_path: File path for a .wav file
        :param n_mfcc: The number of mfccs to be extracted
        :param max_len: Maximum length of the mfccs to be extracted
        :return: mfcc: mfccs extracted from the .wav file, padded or truncated to max_len
        """
        signal, sr = librosa.load(file_path, sr=22050)

        signal = librosa.util.normalize(signal)

        mfcc = librosa.feature.mfcc(
        y=signal,
        sr=sr,
        n_mfcc=n_mfcc
        )

        if mfcc.shape[1] < max_len:
            pad_width = max_len - mfcc.shape[1]
            mfcc = np.pad(mfcc, pad_width=((0, 0), (0, pad_width)))
        else:
            mfcc = mfcc[:, :max_len]
        return mfcc

    def train_CNN(self):
        """
        Train the CNN model based on data obtained at:
        https://www.kaggle.com/datasets/yehiellevi/dataset-balanced-n-weighted-final

        Not run in the programed, used to save a pre trained CNN model for speed purposes,
        since training the model takes a long time. The trained model is saved as "CNNmodel.keras"

        :returns: model: the trained CNN model
        """

        x = []
        y = []

        labcsv = pd.read_csv("audio_metadata_shuffled.csv")
        labels = ["Drone", "Not_a_drone"]

        for unused, row in labcsv.iterrows():
            if row["class"] == "Drone":
                label = 0
            else:
                label = 1

            file_path = os.path.join("datasett", row["slice_file_name"])
            mfcc = extract_mfcc(file_path)

            x.append(mfcc)
            y.append(label)

        x = np.array(x)
        y = to_categorical(y, num_classes=len(labels))

        x = x[..., np.newaxis]

        model = Sequential()

        model.add(Conv2D(32, (3, 3), activation='relu',
                         input_shape=(40, 174, 1)))
        model.add(MaxPooling2D((2, 2)))

        model.add(Conv2D(64, (3, 3), activation='relu'))
        model.add(MaxPooling2D((2, 2)))

        model.add(Flatten())
        model.add(Dense(128, activation='relu'))
        model.add(Dropout(0.3))

        model.add(Dense(len(labels), activation='softmax'))

        model.compile(
            optimizer='adam',
            loss='categorical_crossentropy',
            metrics=['accuracy']
        )

        model.fit(
            x,
            y,
            epochs=20,
            batch_size=32,
            validation_split=0.2
        )
        return model


    def cnn_predict(self,file_path):
        """
        Takes a .wav file and predicts the label

        :param file_path: A .wav file
        :return: label: "Drone" or "Not_a_drone"
        """


        mfcc = self.extract_mfcc(file_path)
        mfcc = mfcc[np.newaxis, ..., np.newaxis]
        prediction = self.model.predict(mfcc)
        return self.labels[np.argmax(prediction)]

    def update_training(self,audio_file,label):
        """
        Add new data to the CNN model,Check if label is valid

        :param audio_file: A .wav file
        :param label: Label of audio file, Has to be ether "Drone" or "Not_a_drone"
        :return: Void
        :raises ValueError: Invalid label
        """
        x = self.extract_mfcc(audio_file)
        if label in self.labels:
            if label == self.labels[0]:
                y = 0
            else:
                y = 1
        else:
            raise ValueError(f"Invalid label: {label}. Valid labels are: {self.labels}")
        self.model.fit(x, y, epochs=5)
