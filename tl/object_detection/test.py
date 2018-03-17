import os
import glob
from PIL import Image
import numpy as np
import tensorflow as tf
import label_map_util
import visualization_utils as vis_util

PATH_TO_GRAPH = 'output_inference_graph.pb'
PATH_TO_DATA = 'data'
PATH_TO_CKPT = os.path.join(PATH_TO_GRAPH, 'frozen_inference_graph.pb')
PATH_TO_LABELS = os.path.join(PATH_TO_DATA, 'label_map.pbtxt')

NUM_CLASSES = 2

PATH_TO_TEST_IMAGES_DIR = 'test_images'
TEST_IMAGE_PATHS = glob.glob(os.path.join(PATH_TO_TEST_IMAGES_DIR,'*.jpg'))
TEST_IMAGE_PATHS += glob.glob(os.path.join(PATH_TO_TEST_IMAGES_DIR,'*.png'))
OUTPUT_PATH = 'test_output'

detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

def load_image_into_numpy_array(image):
  (im_width, im_height) = image.size
  return np.array(image.getdata()).reshape(
      (im_height, im_width, 3)).astype(np.uint8)

import time
total_time = 0.
n_images = 0

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
        for image_path in TEST_IMAGE_PATHS:
            image = Image.open(image_path)
            # the array based representation of the image will be used later in order to prepare the
            # result image with boxes and labels on it.
            image_np = load_image_into_numpy_array(image)
            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            image_np_expanded = np.expand_dims(image_np, axis=0)

            # Actual detection.
            detect_start_time = time.time()
            (boxes, scores, classes, num) = sess.run(
                [detection_boxes, detection_scores, detection_classes, num_detections],
                feed_dict={image_tensor: image_np_expanded})
            total_time += time.time() - detect_start_time
            n_images+=1

            # print(num)
            # print(classes)
            # print(scores)
            # Visualization of the results of a detection.
            vis_util.visualize_boxes_and_labels_on_image_array(
                image_np,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                category_index,
                use_normalized_coordinates=True,
                line_thickness=8)
            image = Image.fromarray(image_np)
            image.save(os.path.join(OUTPUT_PATH,os.path.basename(image_path)))

print('------- stats ---------')
print('number of images: {}'.format(n_images))
print('total time: {}'.format(total_time))
print('time per image: {}'.format(total_time/n_images))
print('images per second: {}'.format(float(n_images)/total_time))
