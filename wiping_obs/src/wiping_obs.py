#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from sklearn.cluster import KMeans
from scipy.ndimage import label
from std_msgs.msg import Float64, Float64MultiArray

class WipingObsDetector:
    def __init__(self):
        rospy.init_node('wiping_obs_detector', anonymous=True)

        # Create a CvBridge to convert between OpenCV and ROS image formats
        self.bridge = CvBridge()

        # Subscribe to the RealSense camera topic (adjust the topic name accordingly)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # Publisher for the wiping observations (centroid and radius)
        self.obs_pub = rospy.Publisher('/wiping_obs', Float64MultiArray, queue_size=10)


    def darkest_cluster(self,image,colors=2):

        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Convert the pixels to float type
        pixels = image_rgb.reshape(-1, 3)
        pixels = np.float32(pixels)

        kmeans = KMeans(n_clusters=colors)
        kmeans.fit(pixels)

        labels = kmeans.labels_
        label_counts = np.bincount(labels)

        # Convert the centers to uint8 type
        centers = np.uint8(kmeans.cluster_centers_)

        # Calculate the average RGB value for each cluster
        average_colors = []
        for center in centers:
            average_color = tuple(center)
            average_colors.append(average_color)

        darkest_color_idx = np.argmax(label_counts)
        darkest_color = kmeans.cluster_centers_[darkest_color_idx]

        # Create a mask to highlight the foreground object based on the darkest color
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        reshaped_labels = labels.reshape(image.shape[:2])
        mask[reshaped_labels == darkest_color_idx] = 255

        # Set the darkest color to bright red
        result = cv2.bitwise_and(image, image, mask=mask)

        # Set a threshold value (adjust this value as needed)
        threshold_value = 1

        # Apply binary thresholding
        _, result_binary = cv2.threshold(result, threshold_value, 255, cv2.THRESH_BINARY)

        result_binary = cv2.cvtColor(result_binary, cv2.COLOR_BGR2GRAY)

        # Display the binary image
        # cv2.imshow('Binary Image', result_binary)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # print('result_binary.shape',result_binary.shape)
        # print('result_binary',result_binary)
        # print('unique', np.unique(result_binary))

        return result_binary, average_colors, darkest_color, darkest_color_idx, labels
        
    def find_centroid_and_radius(self,binary_image):
        # Find connected components
        labeled_image, num_features = label(binary_image == 0)

        # Calculate centroids and radii
        min_distance = np.inf
        closest_centroid = None
        closest_radius = None

        center_y, center_x = binary_image.shape[0] / 2, binary_image.shape[1] / 2

        for i in range(1, num_features + 1):
            component = (labeled_image == i)
            y_coords, x_coords = np.where(component)
            if len(y_coords) == 0 or len(x_coords) == 0:
                continue  # Skip disconnected components with no valid centroid

            centroid_y, centroid_x = np.mean(y_coords), np.mean(x_coords)

            if np.isnan(centroid_x) or np.isnan(centroid_y):
                continue  # Skip components with NaN centroids

            # Calculate the distance between the centroid and the center of the image
            distance_to_center = np.sqrt((centroid_y - center_y)**2 + (centroid_x - center_x)**2)

            # Update the closest centroid and its radius if it is closer to the center
            if distance_to_center < min_distance:
                min_distance = distance_to_center
                closest_centroid = (centroid_x, centroid_y)

                # Calculate the radius as the maximum distance between centroid and any point within the component
                distances = np.sqrt((y_coords - centroid_y)**2 + (x_coords - centroid_x)**2)
                closest_radius = np.max(distances)

        return closest_centroid, closest_radius

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr(e)
            return

        # Perform wiping observations processing (use your existing functions)
        colors = 2  # Number of clusters
        result_binary, _, _, _, _ = self.darkest_cluster(cv_image, colors)
        closest_centroid, closest_radius = self.find_centroid_and_radius(result_binary)

        # Publish the wiping observations
        self.publish_wiping_obs(closest_centroid, closest_radius)

    def publish_wiping_obs(self, centroid, radius):
        # Create a Float64MultiArray message to publish centroid and radius
        obs_msg = Float64MultiArray()
        obs_msg.data = [centroid[0], centroid[1], radius]

        # Publish the wiping observations
        self.obs_pub.publish(obs_msg)

if __name__ == '__main__':
    try:
        # Initialize the WipingObsDetector class
        wiping_obs_detector = WipingObsDetector()

        # Spin the ROS node
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
