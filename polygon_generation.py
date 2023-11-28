import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
import yaml

class ImagePolygonDrawer:
    def __init__(self):
        self.points = []
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cart_bscan", Image, self.image_callback)
        self.pixel = 3000 * 1.8 / 1024
        self.x0 = 1025
        self.y0 = 1025
        self.scale_factor = 0.5  # You can adjust this as needed
        rospy.spin()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Resize the image for better visibility
        cv_image = cv2.resize(cv_image, None, fx=self.scale_factor, fy=self.scale_factor)
        
        self.show_image_and_get_polygon(cv_image)

        # Unsubscribe to avoid processing further images after drawing the polygon
        self.image_sub.unregister()

    def show_image_and_get_polygon(self, img):
        cv2.imshow("Draw Polygon", img)
        cv2.setMouseCallback("Draw Polygon", self.draw_polygon, img)
        
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        transformed_points = [val for point in self.points for val in (self.f(point[0]), self.g(point[1]))]
        self.publish_to_rosparam(transformed_points)
        self.save_to_yaml(transformed_points)
        
        # Exit after processing the first image and saving the polygon
        rospy.signal_shutdown("Polygon drawn, exiting...")

    def draw_polygon(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Inverse the scale factor
            orig_x, orig_y = int(x / self.scale_factor), int(y / self.scale_factor)

            cv2.circle(param, (x, y), 3, (0, 255, 0), -1)
            self.points.append((orig_x, orig_y))
            
            # Drawing the line segment without closing the polygon
            if len(self.points) > 1:
                cv2.line(param, (int(self.points[-2][0] * self.scale_factor), int(self.points[-2][1] * self.scale_factor)), 
                         (x, y), (0, 255, 0), 2)
            
            cv2.imshow("Draw Polygon", param)

    def f(self, x):
        # Your transformation for x
        x = (x - self.x0) * self.pixel
        return x

    def g(self, y):
        # Your transformation for y
        y = (self.y0 - y) * self.pixel
        return y

    def publish_to_rosparam(self, points):
        # Publishing to rosparam
        rospy.set_param('/polygon/points', points)
        
    def save_to_yaml(self, points):
        data = {'points': points}
        with open('polygon_points.yaml', 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)

if __name__ == "__main__":
    rospy.init_node('image_polygon_drawer', anonymous=True)
    ipd = ImagePolygonDrawer()