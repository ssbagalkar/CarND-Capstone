from styx_msgs.msg import TrafficLight
import numpy as np
import os.path
import sys
import tensorflow as tf

from collections import defaultdict
from io import StringIO
from PIL import Image

class TLClassifier(object):
    def __init__(self):
        current_path = os.path.dirname(__file__)
        CKPT = os.path.join(current_path, 'frozen_inference_graph.pb')

        PATH_TO_LABELS = os.path.join(current_path, 'label_map.pbtxt')

        NUM_CLASSES = 14

        self.detection_graph = tf.Graph()

        # create config
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True

        # Create graph
        with self.detection_graph.as_default():
            graph_definition = tf.GraphDef()

            # load pre trained model
            with tf.gfile.GFile(CKPT, 'rb') as fid:
                serial_graph = fid.read()
                graph_definition.ParseFromString(serial_graph)
                tf.import_graph_def(graph_definition, name='')

            # Create a reusable sesion attribute
            self.sess = tf.Session(graph=self.detection_graph, config=config)

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        
        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    # load image from disk
    def load_image_into_numpy_array(self, image):
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape(
            (im_height, im_width, 3)).astype(np.uint8)

    def label_to_traffic_light_map(self, label):
        return {
            1: TrafficLight.GREEN,
            2: TrafficLight.RED,
            3: TrafficLight.GREEN,
            4: TrafficLight.GREEN,
            5: TrafficLight.RED,
            6: TrafficLight.RED,
            7: TrafficLight.YELLOW,
            8: TrafficLight.UNKNOWN,
            9: TrafficLight.RED,
            10: TrafficLight.GREEN,
            11: TrafficLight.GREEN,
            12: TrafficLight.GREEN,
            13: TrafficLight.RED,
            14: TrafficLight.RED
        }.get(label, TrafficLight.UNKNOWN)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        label = 8 #i.e. TrafficLight.UNKNOWN
        traffic_light_id = TrafficLight.UNKNOWN

        # image_np = self.load_image_into_numpy_array(image)
        
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image, axis=0)

        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],feed_dict={self.image_tensor: image_np_expanded})

        # convert to np arrays
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        SCORE_THRESH = 0.5
        for i in range(boxes.shape[0]):
            if scores[i] > SCORE_THRESH:
                label = classes[i]

        traffic_light_id = self.label_to_traffic_light_map(label)
        
        return traffic_light_id