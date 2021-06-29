import cv2
import tflite_runtime.interpreter as tflite
import numpy as np

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LaneTracking(Node):

    def __init__(self, model_path):
        super().__init__('lane_tracking')
        self.cap = cv2.VideoCapture(0)
        self.interpreter = self.load_model(model_path)  # Load Model
        self.input_details = self.interpreter.get_input_details()  # Get input tensor details.
        self.output_details = self.interpreter.get_output_details()  # Get output tensors details.
        self.input_shape = self.input_details[0]['shape']
        self.linear_velocity = [0.2, 0.1, 0.1, 0.1, 0.0]  # unit: m/s
        self.angular_velocity = [0.0, 0.2, 0.40, -0.40, 0.0]  # unit: m/s

        self.safety_distance = 0.3  # unit: m
        self.a_width = 0.9
        self.fl = 144.443
        self.scan_ranges = []
        self.init_scan_state = False
        self.direction = [True, True, True, True] # Values for Front, Back, Right, Left.
        self.obstacle = [True, True, True, True] # Values for Front, Back, Right, Left. True if obstacle present False if obstacle is not available.

        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.cmd_vel_raw_sub = self.create_subscription(Twist, 'cmd_vel_raw', self.cmd_vel_raw_callback, qos)
        self.update_timer = self.create_timer(0.010, self.update_callback)
        self.get_logger().info("Lane Tracker Node is Initialized!.")

    def scan_callback(self, msg):
            self.scan_ranges = msg.ranges
            self.front = self.scan_ranges[315:359] + self.scan_ranges[0:45]
            while 0.0 in self.front:
                self.front.remove(0.0)
            self.right = self.scan_ranges[45:135]
            while 0.0 in self.right:
                self.right.remove(0.0)
            self.back = self.scan_ranges[135:225]
            while 0.0 in self.back:
                self.back.remove(0.0)
                #print(self.back)
            self.left = self.scan_ranges[225:315]
            while 0.0 in self.left:
                self.left.remove(0.0)

            self.obstacle = [(min(self.front) < self.safety_distance),
                    (min(self.back) < self.safety_distance),
                    (min(self.right) < self.safety_distance),
                    (min(self.left) < self.safety_distance)]

            self.init_scan_state = True
    
    def cmd_vel_raw_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def load_model(self, model_path):
        """Load TFLite model and allocate tensors.
        Arguments:
        model_path -- path to the tflte model
        Returns:
        Returns an instance of loaded model
        """
        interpreter = tflite.Interpreter(model_path=model_path)
        interpreter.allocate_tensors()
        return interpreter

    def preprocess(self, img):
        """Preprocess the input image
        Arguments:
        img -- input image buffer
        Returns:
        Returns a preprocessed input buffer for tflite and red color extraction
        """
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        _, w, h, c = self.input_shape
        img = cv2.resize(img, (w, h))
        x = np.expand_dims(img, axis=0)
        input_data_tf = np.array(x/255.0, dtype=np.float32)
        return input_data_tf, img

    def create_mask(self, pred_mask):
        """Creates segmenation map from the output predicted by network
        Arguments:
        pred_mask -- buffer containing probility matrix of bg and fg
        Returns:
        Returns a segmentaion map
        """
        pred_mask = np.argmax(pred_mask, axis=-1)
        pred_mask = np.uint8(pred_mask)
        return np.array(pred_mask[0])

    def infer_tflite(self, input_data):
        """Preprocess the input image
        Arguments:
        input_data -- input image buffer
        Returns:
        Returns segmentation map
        """
        self.interpreter.set_tensor(
            self.input_details[0]['index'], input_data)  # set input data
        self.interpreter.invoke()
        output_data = self.interpreter.get_tensor(
            self.output_details[0]['index'])  # fetch output after inferring
        seg = self.create_mask(output_data)  # get segmenation map
        return seg

    def extract_red(self, img):
        """Extract red color region
        Arguments:
        img -- input image
        Returns:
        Returns mask.
        """
        img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        # lower mask (0-10)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

        # upper mask (170-180)
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

        # join masks
        mask = mask0+mask1
        return mask

    def bounding_box(self, img, roi):
        """calculates bounding box for the blob
        Arguments:
        img -- input image
        roi -- segmenation map
        Returns:
        Returns list of areas and list of corresponding rectangular box coordinates
        """
        roi_copy = roi.copy()
        img_copy = img.copy()
        img_w = roi.shape[1]
        img_h = roi.shape[0]
        _, thresh1 = cv2.threshold(roi_copy, 150, 255, cv2.THRESH_BINARY)
        mask1 = thresh1
        kernel = np.ones((5, 5), np.int8)
        mask1 = cv2.dilate(mask1, kernel, iterations=1)
        cnts, hierarchy = cv2.findContours(
            mask1.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[
            :5]  # get largest contour area
        rects = []
        areas = []
        for c in cnts:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)
            rec = x, y, w, h
            area = (w/img_w) * (h/img_h)
            # cv2.rectangle(img_copy, (x, y), (x+w, y+h), (0, 255, 0), 1)
            rects.append(rec)
            areas.append(area)
        return (areas, rects)

    def calculate_distance(self, p_width):
        """Calculates distance to object
        Arguments:
        p_width -- width of bounding box in pixels
        Returns:
        returns distance to object
        """
        return (self.fl * self.a_width)/p_width

    def calculate_area(self, img, mask_tf, mask_cv):
        """calculates area for detected bounding box
        Arguments:
        img -- input image
        mask_tf -- segmenation map from tflite prediction
        mask_cv -- segmenation map from red color extraction
        Returns:
        Returns area and distance
        """
        areas_tf, recs_tf = self.bounding_box(img, mask_tf * 255)
        areas_cv, recs_cv = self.bounding_box(img, mask_cv)
        if recs_tf and recs_cv:
            recs_tf.extend(recs_cv)
            single_rect, weights = cv2.groupRectangles(recs_tf, 1, 1.5)
            if len(single_rect):
                img_copy = img.copy()
                x, y, w, h = single_rect[0]
                cv2.rectangle(img_copy, (x, y), (x+w, y+h), (0, 255, 0), 1)
                area = (w/mask_tf.shape[1]) * (h/mask_tf.shape[0])
                distance = round(self.calculate_distance(w), 3)
            else:
                area = 0.0
                distance = 0.0
        else:
                area = 0.0
                distance = 0.0
        return area, distance
    
    def update_callback(self):
        if self.init_scan_state is True:
            self.track_road()

    def get_direction(self, img):
        (H, W, C) = img.shape
        turn = [0, 0, 0, 0, 0]
        Y = int((H/3)*2)
        X = int((W/4))
        X_E = (X*3)
        im = img.copy()
        img = img[Y:H, X:X_E]

        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, img = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY)

        (H, W) = img.shape
        X = int((W/5))
        X_E = (X*4)
        left = img[:,0:X]
        mid = img[:,X:X_E]
        right = img[:,X_E:]

        if(255 in left):
            turn[2] = 1
            print("Left")
        elif(255 in right):
            turn[3] = 1
            print("Right")
        elif(255 in mid):
            turn[0] = 1
            print("Forward")
        else:
            turn[4] = 1
            print("stopped")
        
        return turn

    def stop_sign_detector(self, frame):
        input_data_tf, input_data_cv = self.preprocess(frame)
        out_tf = self.infer_tflite(input_data_tf)
        out_cv = self.extract_red(input_data_cv)

        area, d = self.calculate_area(input_data_cv, out_tf, out_cv)
        print("Stop Sign Info\narea={}\ndistance={}\n--------".format(area, d))

        if area == 0.0:
            st_indx = 0 #not stop sign
        elif d > 12:
            st_indx = 0 #not stop sign
        elif d >= 9 and d <= 12:
            st_indx = 1 #Far stop sign
        elif d < 9:
            st_indx = 2 #stop sign ahead

        return st_indx

    def track_road(self):
        twist = Twist()
        ret, img = self.cap.read()
        indx = 4
        st_indx = self.stop_sign_detector(img)
        if st_indx == 0:
            if self.obstacle[0] is False:
                self.direction = self.get_direction(img)
                if self.direction[0] == 1:
                    indx = 0
                elif self.direction[1] == 1:
                    indx = 1
                elif self.direction[2] == 1:
                    indx = 2
                elif self.direction[3] == 1:
                    indx = 3
                else:
                    self.get_logger().info("Not able to locate the road. Robot stopped.")
            else:
                self.get_logger().info("Obstracle ahead. Robot stopped. Please clear it!")
        else:
            self.get_logger().info("STOP Sign ahead. Robot stopped.")

        twist.linear.x = self.linear_velocity[indx]
        twist.angular.z = self.angular_velocity[indx]  
        self.cmd_vel_pub.publish(twist)
