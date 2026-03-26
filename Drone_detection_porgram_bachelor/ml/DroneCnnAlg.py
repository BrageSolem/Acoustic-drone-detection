import os
import numpy as np
import librosa

from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D
from tensorflow.keras.layers import Dense, Flatten, Dropout
from tensorflow.keras.utils import to_categorical

def extract_mfcc(file_path, n_mfcc=40, max_len=174):
    signal, sr = librosa.load(file_path, sr=22050)

    signal = librosa.util.normalize(signal)

    mfcc = librosa.feature.mfcc(
        y=signal,
        sr=sr,
        n_mfcc=n_mfcc
    )

    # Pad or truncate to fixed length
    if mfcc.shape[1] < max_len:
        pad_width = max_len - mfcc.shape[1]
        mfcc = np.pad(mfcc, pad_width=((0, 0), (0, pad_width)))
    else:
        mfcc = mfcc[:, :max_len]

    return mfcc


def train_CNN(dataset):
    X = []
    y = []

    labels = os.listdir(dataset)

    for label_index, label in enumerate(labels):
        folder = os.path.join(dataset, label)

        for file in os.listdir(folder):
            file_path = os.path.join(folder, file)
            mfcc = extract_mfcc(file_path)

            X.append(mfcc)
            y.append(label_index)

    X = np.array(X)
    y = to_categorical(y, num_classes=len(labels))

    X = X[..., np.newaxis]

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
        X,
        y,
        epochs=20,
        batch_size=32,
        validation_split=0.2
    )
    return model


def run_CNN(soundfile, model):
    mfcc = extract_mfcc(soundfile)
    mfcc = mfcc[np.newaxis, ..., np.newaxis]

    prediction = model.predict(mfcc)
    predicted_label = labels[np.argmax(prediction)]
    return predicted_label


def run_CNN_with_mfcc_in_database(librosa_mfcc_db, model):
    mfcc = librosa.feature.mfcc(librosa_mfcc_db)

    prediction = model.predict(mfcc)
    predicted_label = labels[np.argmax(prediction)]
    return predicted_label

def run_CNN_with_librosa_mfcc(librosa_mfcc):
    prediction = model.predict(librosa_mfcc)
    predicted_label = labels[np.argmax(prediction)]
    return predicted_label