import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import os
from .bot_localization import bot_localizer
from .bot_mapping import bot_mapper, Graph
from .bot_pathplanning import bot_pathplanner

from nav_msgs.msg import Odometry
from .bot_motionplanning import bot_motionplanner

import numpy as np

class maze_solver(Node):

    def __init__(self):
        super().__init__("maze_solving_node")

        # Create a publisher
        self.velocity_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.2
        self.timer_ = self.create_timer(timer_period, self.mazeSolver)

        # Created a subscriber
        self.subscriber_ = self.create_subscription(Image, '/upper_camera/image_raw', self.get_video_feed_cb, 10)

        # converting ros_images to opencv data
        self.bridge = CvBridge()
        self.vel_msg = Twist()

        self.bot_localizer = bot_localizer()
        self.bot_mapper = bot_mapper()
        self.bot_pathplanner = bot_pathplanner()
        self.bot_motionplanner = bot_motionplanner()

        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.bot_motionplanner.get_pose, 10)
       
        self.sat_view = np.zeros((100, 100))

    def get_video_feed_cb(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8') # performing conversion
        self.sat_view = frame
        cv2.imshow("sat_view", self.sat_view) # display what is being recorded
        cv2.waitKey(1) # will save video until it is interrupted

    def mazeSolver(self):
        # Creating frame to display current robot state to user
        frame_disp = self.sat_view.copy()

        # Stage 1: Localization- Localizing robot at each iteration
        self.bot_localizer.localize_bot(self.sat_view, frame_disp)

        # Stage 2: Mapping - Converting Image to Graph
        self.bot_mapper.graphify(self.bot_localizer.maze_og)
        
        # Stage 3: PathPlanning
        start = self.bot_mapper.Graph.start
        #end = (31, 300)
        end = self.bot_mapper.Graph.end
        maze = self.bot_mapper.maze
        self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze,method="a_star")

        # Stage 4: Motionplanning
        bot_loc = self.bot_localizer.loc_car
        path = self.bot_pathplanner.path_to_goal
        self.bot_motionplanner.nav_path(bot_loc, path, self.vel_msg, self.velocity_publisher_)
        
        # Displaying bot solving maze (Live)
        img_shortest_path = self.bot_pathplanner.img_shortest_path
        self.bot_motionplanner.display_control_mechanism_in_action(bot_loc, path, img_shortest_path, self.bot_localizer, frame_disp)
        cv2.imshow("Maze (Live)", frame_disp)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = maze_solver()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()