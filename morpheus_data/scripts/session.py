#!/usr/bin/env python

# General Packages
import os
import logging
from collections import deque
# ROS Packages
import rospy
# Keyboard input imports
from readchar import readkey, readchar, key
import pygame as pg
# Local imports
from data import Data
from utils import *
from ui_strings import *

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

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # pygame setup
        # self.screen = pg.display.set_mode((1280, 720))
        self.clock = pg.time.Clock()
        self.running = True
        self.dt = 0

        # self.screen.fill((30, 30, 30))

        self.input_boxes = [InputBox(100, 100, 140, 32), InputBox(100, 200, 140, 32), InputBox(100, 300, 140, 32)]
    
    def prompt_session(self):
        print("Starting recording session.")
        
        print("Press ENTER to submit, or press ESC to cancel.")

        while not rospy.is_shutdown():
            id = ""
            print("Please enter subject ID: ")
            while not rospy.is_shutdown():
                k = get_key()
                if k.isalnum() or k in "._ ": # If key==char, join to input string
                    id = id + k
                elif k == key.BACKSPACE or k == key.DELETE:
                    if len(id) > 0:
                        id = id[:-1]
                elif k == key.ENTER: # If key==enter, accept input and move on
                    id = sanitize_filename(id)
                    self._id = id
                    print("\rSubject ID is %s" % (id))
                    self.prompt_task(id)
                    break
                elif k == key.ESC: # If key==esc, exit
                    print("\rEnding recording session.")
                    return
                print("\x1b[2K", id, end="\r")
    
    def prompt_task(self, id):
        while not rospy.is_shutdown():
            task = ""
            print("Please enter task name: ")
            while not rospy.is_shutdown():
                k = get_key()
                if k.isalnum() or k in "._ ": # If key==char, join to input string
                    task = task + k
                elif k == key.BACKSPACE or k == key.DELETE:
                    if len(task) > 0:
                        task = task[:-1]
                elif k == key.ENTER: # If key==enter, accept input and move on
                    task = sanitize_filename(task)
                    self._task = task
                    print("\rTask is %s" % (task))
                    self.prompt_trial(id, task)
                    break
                elif k == key.ESC: # If key==esc, exit
                    print("\rCancelling task selection")
                    return
                print("\x1b[2K", task, end="\r")
    
    def prompt_trial(self, id, task):
        while not rospy.is_shutdown():
            trial_number = self.get_next_trial_number(id, task)
            trial_string = ""
            print("Please enter trial number (or press ENTER to use next index: %s): " % (trial_number))
            while not rospy.is_shutdown():
                k = get_key()
                if k.isnumeric(): # If key==number, join to input string
                    trial_string = trial_string + k
                elif k == key.BACKSPACE or k == key.DELETE:
                    if len(trial_string) > 0:
                        trial_string = trial_string[:-1]
                elif k == key.ENTER: # If key==enter, accept input and move on
                    if trial_string:
                        trial_number = int(trial_string)
                    self._trial_number = trial_number
                    print("\rTrial number is %s" % (trial_number))
                    filename = self.get_filename(id, task, trial_number)
                    print("\rFilename is %s" % (filename))
                    filepath = self.get_filepath(filename)
                    trial = self.create_trial(filepath)
                    self._save_trial(trial, id, task, trial_number) # Save trial w/o starting to record
                    self.prompt_record(id, task, trial_number) # Prompt user to start/stop recording
                    break
                elif k == key.ESC: # If key==esc, exit
                    print("\rCancelling trial selection")
                    return
                print("\x1b[2K", trial_string, end="\r")

    def prompt_record(self, id, task, trial_number):
        recording = False
        
        print("Press ENTER to toggle recording (id=%s, task=%s, trial_number=%s)" % (id, task, trial_number))
        while not rospy.is_shutdown():
            k = get_key()
            if k == key.ENTER: # If key==enter, toggle record
                if not recording:
                    print("\rStarting recording (id=%s, task=%s, trial_number=%s)" % (id, task, trial_number))
                    self.start_trial(id, task, trial_number)
                    recording = True
                else:
                    print("\rStopping recording (id=%s, task=%s, trial_number=%s)" % (id, task, trial_number))
                    self.stop_trial(id, task, trial_number)
                    recording = False
            elif k == key.ESC: # If key==esc, exit
                print("\rExiting recording (id=%s, task=%s, trial_number=%s)" % (id, task, trial_number))
                if recording:
                    self.stop_trial(id, task, trial_number)
                recording = False
                return
    
    def prompt(self, state="", id=None, task=None, trial=None):
        # Initialize input and output variables
        queue = deque([], 3)
        state = ""
        input = ""

        # Handle state-related events
        if state == "recording":
            self.start_trial(id, task, trial)

        # Add static strings to queue
        queue = self.enqueue_header(queue, id, task, trial)
        queue = self.enqueue_string(queue, state + "_description")
        
        # Get keyboard input events
        event = None
        input, event = self.handle_key_events(input, event)

        # Respond to keyboard input events
        if event == key.ENTER:
            if state == "recording":
                event == key.ESC
            else:
                for child_state in session_state_dict[state]:
                    self.prompt(child_state, id, task, trial_number)
        if event == key.ESC:
            return

    def handle_key_events(self, input="", event=None)
        k = get_key()
        if k.isalnum() or k in "._ ":
            input += k
        elif k == key.ENTER:
            event = key.ENTER
        elif k == key.ESC:
            event = key.ESC
        return input, event
    
    def enqueue_string(self, queue, key, suffix=""):
        try:
            queue.append(self.session_string_dict[key] + (suffix or "")) # OR statement to handle case when suffix==None
        except KeyError: # If Key is missing, assume this is intended and add nothing
            continue
        return queue
    
    def enqueue_header(self, queue, id, task, trial):
        queue = self.enqueue_string(queue, "header_title")
        queue = self.enqueue_string(queue, "header_id", id)
        queue = self.enqueue_string(queue, "header_task", task)
        queue = self.enqueue_string(queue, "header_trial", trial)
        return queue

    def get_trial(self, id, task, trial_number):
        # Retrieve data from file
        data = None
        filename = get_filename(id, task, trial_number)
        filepath = get_filepath(filename)
        filepath_exists = os.path.isfile(filepath)
        if filepath_exists:
            print("Filename already exists! Double check file number and try again.")
        else:
            data = create_trial(filepath)
        return data
    
    def get_next_trial_number(self, id, task):
        # Get next available trial number (0 indexed)
        trial_number = -1 # Start negative so that first increment goes to 0
        filepath_exists = True
        while filepath_exists:
            trial_number = trial_number + 1 # Try next trial number
            filepath = self.get_filepath(self.get_filename(id, task, trial_number))
            filepath_exists = os.path.isfile(filepath) # Check if filepath is available
        return trial_number

    def get_filename(self, id, task, trial_number):
        # Get filename based on  subject id, task type, trial number (0 indexed)
        filename = "%s%s%s.bag" % (str(id), str(task), str(trial_number))
        return filename
    
    def get_filepath(self, filename):
        # Get absolute filepath (places filename in the data directory)
        dirname = os.path.dirname(__file__)
        datadir = os.path.join(os.path.dirname(dirname), 'data')
        filepath = os.path.join(datadir, filename)
        return filepath

    def create_trial(self, filepath):
        # Create new trial with given filepath
        trial = Data(filepath, mode='w') # Prepare to write
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
                self._data_dict[id][task] = {} # Initialize value as empty dict
            self._data_dict[id][task][trial_number] = trial
            return True
        except: # Report exception and return false
            print("Failed to save trial to data dict: (id=%s, task=%s, trial_number=%s)" % (id, task, trial_number))
            return False
        
    def _get_last_trial(self, id=None, task=None, trial_number=None):
        # Default to +1 trial of last used task if inputs are not given
        if id == None:
            id = self._id
        if task == None:
            task = self._task
        if trial_number == None:
            trial_number = 0
            if task in self._data_dict:
                if len(self._data_dict[task]) > 0:
                    trial_number = max(self._data_dict[task].keys()) - 1
        return id, task, trial_number

    def handle_events_gui(self):
        self.screen.fill((30, 30, 30))
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self.running = False
                for input_box in self.input_boxes:
                    input_box.done = True
            if event.type == pg.MOUSEBUTTONDOWN:
                # If the user clicked on the input_box rect.
                for input_box in self.input_boxes:
                    if input_box.input_box.collidepoint(event.pos):
                        # Activate when clicked on
                        input_box.active = True
                    else:
                        # Deactivate when clicked off
                        input_box.active = False
                    # Change the current color of the input box.
                    input_box.color = input_box.color_active if input_box.active else input_box.color_inactive
            if event.type == pg.KEYDOWN:
                for input_box in self.input_boxes:
                    if input_box.active:
                        if event.key == pg.K_RETURN:
                            print(input_box.text)
                            input_box.text = ''
                        elif event.key == pg.K_BACKSPACE:
                            input_box.text = input_box.text[:-1]
                        else:
                            input_box.text += event.unicode
        for input_box in self.input_boxes:
            self.step(input_box)
        
        pg.display.flip()
        self.clock.tick(30)

    def step(self, input_box):
        # Render the current text.
        txt_surface = input_box.font.render(input_box.text, True, input_box.color)
        # Resize the box if the text is too long.
        width = max(200, txt_surface.get_width()+10)
        input_box.input_box.w = width
        # Blit the text.
        self.screen.blit(txt_surface, (input_box.input_box.x+5, input_box.input_box.y+5))
        # Blit the input_box rect.
        pg.draw.rect(self.screen, input_box.color, input_box.input_box, 2)

class InputBox():
    def __init__(self, left, top, width, height):
        self.input_box = pg.Rect(left, top, width, height)
        self.font = pg.font.Font(None, 32)
        self.color_inactive = pg.Color('lightskyblue3')
        self.color_active = pg.Color('dodgerblue2')
        self.color = self.color_inactive
        self.active = False
        self.text = ''
        self.done = False

def main():
    rospy.init_node('session')
    pg.init()
    session = Session()

    # while session.running:
    #     session.handle_events()

    session.prompt_session()
    pg.quit()

if __name__=='__main__':
    main()