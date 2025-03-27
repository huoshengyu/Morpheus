#!/usr/bin/env python

# General Packages
import subprocess
# ROS Packages
import rospy
import rosbag

### Wrapper for rosbag, allowing convenient recording start/stop ###
class Data:
    def __init__(self, filename=""):
        self._file = None
        self._filename = filename
        self._bag = None
        self._open = False
    
    def record(self):
        if self._open:
            self.close()
        self._bag = rosbag.Bag(self._filename, 'w')
        self._open = True

    def read(self):
        if self._open:
            self.close()
        self._bag = rosbag.Bag(self._filename, 'r')
        self._open = True
    
    def close(self):
        self._file.close()
        self._open = False


def main():
    rospy.init_node('data')
    data = Data()
    data.record()

    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        rate.sleep()
    data.close()

if __name__=='__main__':
    main()