#!/usr/bin/env python

# ROS Packages
import rospy
# Keyboard input imports
from readchar import readkey, key
# Local imports
from data import Data
from utils import *

# Documentation for rosbag py on ros wiki is poor, instead see source code here:
# https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosbag/src/rosbag/bag.py

### Session object, organizing multiple data recordings (trials) ###
class Session():
    def __init__(self, id="", rate=60):
        id = sanitize_filename(id)
        self._id = id # subject ID (for tracking latest trial)
        self._task = "" # trial name (for tracking latest trial)
        self._trial_number = 0 # trial number (for tracking latest trial)
        self._rate = rate # loop rate for key press reading
        self._data_dict = {} # key = task ID, value = list of trial data objects
    
    def prompt_session(self):
        print("Starting recording session.")
        
        print("Press ENTER to submit, or press ESC to cancel.")

        while not rospy.is_shutdown():
            id = ""
            print("Please enter subject ID: ")
            while not rospy.is_shutdown():
                k = readkey()
                if k.isalnum() or k in "._ ": # If key==char, join to input string
                    id = id.join(k)
                elif k == key.ENTER: # If key==enter, accept input and move on
                    id = sanitize_filename(id)
                    self._id = id
                    print("Subject ID is %s" (id))
                    self.prompt_task(id)
                    break
                elif k == key.ESC: # If key==esc, exit
                    print("Ending recording session.")
                    return
    
    def prompt_task(self, id):
        while not rospy.is_shutdown():
            task = ""
            print("Please enter task name: ")
            while not rospy.is_shutdown():
                k = readkey()
                if k.isalnum() or k in "._ ": # If key==char, join to input string
                    task = task.join(k)
                elif k == key.ENTER: # If key==enter, accept input and move on
                    task = sanitize_filename(task)
                    self._task = task
                    print("Task is %s" (task))
                    self.prompt_trial(id, task)
                    break
                elif k == key.ESC: # If key==esc, exit
                    print("Cancelling task selection")
                    return
    
    def prompt_trial(self, id, task):
        while not rospy.is_shutdown():
            trial_number = 0
            if task in self._data_dict:
                trial_number = len(self._data_dict[id][task])
            trial_string = ""
            print("Please enter trial number (or press ENTER to use next unused index: %s): " (trial_number))
            while not rospy.is_shutdown():
                k = readkey()
                if k.numeric(): # If key==number, join to input string
                    trial_string = trial_string.join(k)
                elif k == key.ENTER: # If key==enter, accept input and move on
                    if trial_string:
                        trial_number = int(trial_string)
                    self._trial_number = trial_number
                    print("Trial number is %s" (trial_number))
                    filename = self.create_filename(id, task, trial_number)
                    print("Filename is %s" (filename))
                    trial = self.create_trial(filename)
                    self._save_trial(trial, id, task, trial_number) # Save trial w/o starting to record
                    self.prompt_record(id, task, trial_number) # Prompt user to start/stop recording
                    break
                elif k == key.ESC: # If key==esc, exit
                    print("Cancelling trial selection")
                    return

    def prompt_record(self, id, task, trial_number):
        recording = False
        
        print("Press ENTER to toggle recording (id=%s, task=%s, trial_number=%s)" (id, task, trial_number))
        while not rospy.is_shutdown():
            k = readkey()
            if k == key.ENTER: # If key==enter, toggle record
                if not recording:
                    print("Starting recording (id=%s, task=%s, trial_number=%s)" (id, task, trial_number))
                    self.start_trial(id, task, trial_number)
                else:
                    print("Stopping recording (id=%s, task=%s, trial_number=%s)" (id, task, trial_number))
                    self.stop_trial(id, task, trial_number)
            elif k == key.ESC: # If key==esc, exit
                print("Exiting recording (id=%s, task=%s, trial_number=%s)" (id, task, trial_number))
                self.stop_trial(id, task, trial_number)
                return

    def get_data(self):
        # Return data dict
        return self._data_dict
    
    def get_next_trial_number(self, task):
        # Get next trial number (0 indexed)
        trial_number = 0
        if task in self._data_dict:
            trial_number = len(self._data_dict[task])
        return trial_number

    def create_filename(self, id, task, trial_number):
        # Create filename based on  subject id, task type, trial number (0 indexed)
        filename = "%s%s%s.bag" (str(id), str(task), str(trial_number))
        return filename

    def create_trial(self, filename):
        # Create new trial, sorted by task
        trial = Data(filename, mode='w') # Prepare to write
        return trial

    def start_trial(self, id, task, trial_number):
        # Start recording trial
        data = self._data_dict[id][task][trial_number] # Get from dict
        data.write() # Start writing

    def stop_trial(self, id, task, trial_number):
        # Stop recording trial
        data = self._data_dict[id][task][trial_number] # Get from dict
        data.close() # Stop writing

    def _save_trial(self, trial, id=None, task=None, trial_number=None):
        # Save trial to dict (Does not start recording)
        try:
            if id not in self._data_dict: # If key not initialized
                self._data_dict[id] = {} # Initialize value as empty dict
            if task not in self._data_dict[id]: # If key not initialized
                self._data_dict[id][task] = [] # Initialize value as empty list
            if trial_number > len(self._data_dict[id][task]):
                self._data_dict[id][task].append(trial)
            else:
                self._data_dict[id][task][trial_number] = trial
            return True
        except: # Report exception and return false
            print("Failed to save trial to data dict: (id=%s, task=%s, trial_number=%s)" (id, task, trial_number))
            return False
        
    def _get_last_trial(self, id=None, task=None, trial_number=None):
        # Default to last trial of last used task if inputs are not given
        if id == None:
            id = self._id
        if task == None:
            task = self._task
        if trial_number == None:
            trial_number = len(self._data_dict[task] - 1)
        return id, task, trial_number


def main():
    rospy.init_node('session')
    Session = Session()

    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__=='__main__':
    main()