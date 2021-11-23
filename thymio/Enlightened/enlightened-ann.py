import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

SPEED = 400
action_space = np.array([(SPEED,SPEED),(SPEED,0),(SPEED,-SPEED),(-SPEED,0),(-SPEED,-SPEED),(0,-SPEED),(-SPEED,SPEED),(0,SPEED),(0,0)], dtype="i,i")

sensor_inputs = keras.Input(shape=(9,))

hidden_layer_1 = layers.Dense(2, activation="relu")(sensor_inputs)
outputs_layer_1 = layers.Dense(9)(hidden_layer_1)           #Output layer, 2 is left and right wheel velocity
model = keras.Model(inputs=sensor_inputs, outputs=outputs_layer_1, name="obstacle_avoidance")

print(model.summary())
keras.utils.plot_model(model, "something.png")