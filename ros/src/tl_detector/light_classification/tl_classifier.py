from styx_msgs.msg import TrafficLight
import numpy as np
import os
import sys
import tensorflow as tf

from collections import defaultdict
from io import StringIO
from PIL import Image

class TLClassifier(object):
    def __init__(self):
        CKPT = 'frozen_inference_graph.pb'
        PATH_TO_LABELS = 'label_map.pbtxt'

        NUM_CLASSES = 14

        # load frozen tensorflow model into memory
        detection_graph = tf.Graph()

        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(CKPT, 'rb') as fid:

                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        # load label map
        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        category_index = label_map_util.create_category_index(categories)
        


    # load image from disk
    def load_image_into_numpy_array(image):
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape(
            (im_height, im_width, 3)).astype(np.uint8)

    def label_to_traffic_light_map(label):
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

        label = 8
        traffic_light_id = TrafficLight.UNKNOWN
        with detection_graph.as_default():
            with tf.Session(graph=detection_graph) as sess:
                # Definite input and output Tensors for detection_graph
                image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                
                # Each box represents a part of the image where a particular object was detected.
                detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
                
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
                detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = detection_graph.get_tensor_by_name('num_detections:0')


            image_np = self.load_image_into_numpy_array(image)
            
            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    	    image_np_expanded = np.expand_dims(image_np, axis=0)

            # Actual detection.
            (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        traffic_light_id = self.label_to_traffic_light_map(label)

        return traffic_light_id
