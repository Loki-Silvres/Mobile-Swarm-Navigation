#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import OccupancyGrid, MapMetaData
import cv2 as cv
import numpy as np
from scipy.spatial import distance
import heapq
import matplotlib.pyplot as plt

class SemanticMap(Node):

    def __init__(self, debug = False):
        super().__init__('semantic_map')

        self.get_logger().info('Semantic map started') 

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.display_map, 10)

    def display_map(self, msg):
        map = self.occupancygrid_to_numpy(msg)
        cv.imshow('map', map)
        if cv.waitKey(1) & 0xFF == ord('q'):
            cv.destroyAllWindows()
            raise KeyboardInterrupt

    def occupancygrid_to_numpy(self, msg):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        print(f"orig: {np.unique(data)}")
        data[data == 100] = 1
        data[data == 0] = 100
        data[data == -1] = 127
        h,w = data.shape

        data = cv.resize(data, (w*4, h*4), interpolation=cv.INTER_NEAREST)
        print(f"resized: {np.unique(data)}")
        return data

    def numpy_to_occupancy_grid(self, arr, info=None):
        if not len(arr.shape) == 2:
                raise TypeError('Array must be 2D')
        if not arr.dtype == np.int8:
                raise TypeError('Array must be of int8s')

        grid = OccupancyGrid()
        if isinstance(arr, np.ma.MaskedArray):
                # We assume that the masked value are already -1, for speed
                arr = arr.data
        grid.data = arr.ravel()
        grid.info = info or MapMetaData()
        grid.info.height = arr.shape[0]
        grid.info.width = arr.shape[1]

        return grid

    

def main():
    rclpy.init()
    semantic_map = SemanticMap(debug=True)

    try:
        rclpy.spin(semantic_map)
    except KeyboardInterrupt:
        semantic_map.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        semantic_map.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        

        