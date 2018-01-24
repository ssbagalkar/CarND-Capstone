import numpy as np
import os
import sys
import tensorflow as tf

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image

# not sure if really needed
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util



# change to actual file paths
CKPT = 'output_inference_graph/frozen_inference_graph.pb'
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




# load image from disk TODO: change it to image from camera feed from simulator/ros/etc
def load_image_into_numpy_array(image):
  (im_width, im_height) = image.size
  return np.array(image.getdata()).reshape(
      (im_height, im_width, 3)).astype(np.uint8)



# predict on loaded image
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
        
	image = Image.open(image_path) # TODO replace with actual image from simulator/ros/etc
    	# the array based representation of the image will be used later in order to prepare the
    	# result image with boxes and labels on it.
    	image_np = load_image_into_numpy_array(image) 
    	# Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    	image_np_expanded = np.expand_dims(image_np, axis=0)

    	time0 = time.time() # can be deleted

    	# Actual detection.
    	(boxes, scores, classes, num) = sess.run(
      	[detection_boxes, detection_scores, detection_classes, num_detections],
      	feed_dict={image_tensor: image_np_expanded})

    	time1 = time.time() # can be deleted
    	print("Time in milliseconds", (time1 - time0) * 1000) # can be deleted