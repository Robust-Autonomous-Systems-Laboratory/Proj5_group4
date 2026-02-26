import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math as m
import numpy as np
from sensor_msgs.msg import LaserScan
import statistics
from std_msgs.msg import Float64

#controls timer rate 
scanFreq = 20

class calibration(Node):
    def __init__(self):
        super().__init__('calibration_node')
        
        #Setting parameter values here for testing purposes
        self.declare_parameter('target_distance', 0.5)
        self.declare_parameter('target_angle', 1)
        self.declare_parameter('angle_window', 0.1)

        self.targetDistance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.targetAngle = self.get_parameter('target_angle').get_parameter_value().double_value
        self.angleWindow = self.get_parameter('angle_window').get_parameter_value().double_value

        #Create timer that runs 'scanFreq' times a second
        self.dt = 1/scanFreq
        self.timer = self.create_timer(self.dt, self.timerCallback)

        #Create subscriber to laser scan topic
        subscriber_qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.laserSub = self.create_subscription(LaserScan, '/scan', self.laserSubCallback, subscriber_qos_profile)
        self.newScanData = False
        
        #Initialize empty list that will hold proper scan data
        self.scanData = []

        #Create range error pupblisher
        self.rangeErrorPub = self.create_publisher(Float64, 'calibration/range_error', 100)
        

    #Update scan data
    def laserSubCallback(self, msg):
        self.angMin = msg.angle_min
        self.angMax = msg.angle_max
        self.angIncrement = msg.angle_increment
        self.ranges = msg.ranges
        self.rangeMin = msg.range_min
        self.rangeMax = msg.range_max
        self.newScanData = True
    
    #checks scan data for any unexpected values, also pulls correct range
    def updateScanList(self):
        #Find indexes to search through for set parameters
        self.indexMax = int(((self.targetAngle + self.angleWindow/2) - self.angMin) / self.angIncrement)
        self.indexMin = int(((self.targetAngle - self.angleWindow/2) - self.angMin) / self.angIncrement)
        #check if desired range crosses 0 or 360 degrees
        rangeSize = len(self.ranges)
        if self.indexMin < 0 or self.indexMax >= rangeSize:
            
            if self.indexMin < 0:
                for i in range((rangeSize + self.indexMin), rangeSize):
                    if(self.filterScans(self.ranges[i])):
                        self.scanData.append(self.ranges[i])
                for i in range(self.indexMax + 1):
                    if(self.filterScans(self.ranges[i])):
                        self.scanData.append(self.ranges[i])
            
            elif self.indexMax >= rangeSize:
                for i in range(self.indexMin, rangeSize):
                    if(self.filterScans(self.ranges[i])):
                        self.scanData.append(self.ranges[i])
                for i in range(self.indexMax - rangeSize + 1):
                    if(self.filterScans(self.ranges[i])):
                        self.scanData.append(self.ranges[i])
        #range is between 0 and 360 with no crossover:
        else:
            for i in range((self.indexMin, self.indexMax + 1)):
                if(self.ranges[i] < self.rangeMax & self.ranges[i] > self.rangeMin):
                    if(self.filterScans(self.ranges[i])):
                        self.scanData.append(self.ranges[i])

    def filterScans(self, dist):
        #check that LiDAR distance reported is within valid bounds and not a NaN values
        if ((dist > self.rangeMin) and (dist < self.rangeMax) and (not np.isnan(dist))):
            return True
        else:
            return False



    def timerCallback(self):
        #print("test point 1 \n")
        
        #wait for new scan data
        if(self.newScanData == True):
            #Append new data to scanData list
            self.updateScanList()
            #Find median scanData
            self.median = statistics.median(self.scanData)
            #Find and publish range error
            self.rangeError = self.median - self.targetDistance
            print(self.median)
            print(self.scanData[0])
            #self.rangeErrorPub(self.rangeError)


def main(args=None):
    rclpy.init(args=args)
    calibration_node = calibration()
    rclpy.spin(calibration_node)
    calibration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()