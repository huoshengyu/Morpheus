#! /usr/bin/env python3

# General imports
import numpy as np
import rospy
import sensor_msgs.msg

# Keyboard input imports
from readchar import readchar, readkey, key
from pynput import keyboard

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

keybinds = {
    # 'key':[i,j,k]
    # where i,j select input channel [axes, buttons][i][j]
    # k selects input value
    # Left stick:   x = -axes[0], y = axes[1]
    # Right stick:  x = -axes[3], y = axes[4]
    # Triggers:     LT = axes[2], RT = axes[5]
    # Dpad:         L/R = -axes[6], U/D = axes[7]
    # Buttons:      [A, B, X, Y, LB, RB, Share, Menu, Xbox, Lstick, Rstick]
    'w':[0, 1, 1],  # Lstick up / Translate forward
    's':[0, 1,-1],  # Lstick down / Translate backward
    'a':[0, 0, 1],  # Lstick left / Translate left
    'd':[0, 0,-1],  # Lstick right / Translate right
    'q':[0, 2,-1],  # LT / Translate down
    'e':[1, 4, 1],  # LB / Translate up
    'i':[0, 4, 1],  # Rstick up / Pitch up
    'k':[0, 4,-1],  # Rstick down / Pitch down
    'j':[0, 3, 1],  # Rstick left / Yaw left
    'l':[0, 3,-1],  # Rstick right / Yaw right
    'u':[0, 5,-1],  # RT / Roll left / Translate toward
    'o':[1, 5, 1],  # RB / Roll right / Yaw right
    '1':[0, 6, 1],  # Dpad left / Speed coarse
    '2':[0, 6,-1],  # Dpad right / Speed fine
    '3':[0, 7, 1],  # Dpad up / Speed up
    '4':[0, 7,-1],  # Dpad down / Speed down
    '5':[1, 0, 1],  # A: Close gripper / Gripper PWM decrease
    '6':[1, 1, 1],  # B: Open gripper / Open gripper
    '7':[1, 2, 1],  # X: NA / Close gripper
    '8':[1, 3, 1],  # Y: NA / Gripper PWM increase
    '0':[1, 6, 1],  # Share / Toggle goal haptics (goal_haptics.py must be running. Probably only works on Linux anyway)
    '-':[1, 7, 1],  # Menu / Move to home position (temporarily disables Xbox control)
    '=':[1, 8, 1],  # Xbox / Swap reference frame (base <--> end effector)
    'x':[1, 9, 1],  # Lstick press / Prev trajectory segment
    ',':[1,10, 1],  # Rstick press / Next trajectory segment
}

class KeyToJoy():

    def __init__(self):
        self.rate = rospy.Rate(60) # 60 Hz publishing loop rate
        self.joy_pub = rospy.Publisher("joy", sensor_msgs.msg.Joy, queue_size=5)
        self.joy = sensor_msgs.msg.Joy()
        # Keep a record of what keys should be when not pressed
        self._default_axes = np.array([0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0]) # Triggers should be 1 when unpressed
        self._default_buttons = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0]) # Need 13 buttons to accomodate PS4 controller (Xbox needs 11)
        # Initialize joystick values
        self.joy.axes = np.copy(self._default_axes)
        self.joy.buttons = np.copy(self._default_buttons)

        # Create keyboard listener object
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()
    
    def on_press(self, key):
        # If key in binds, send command value (only takes one key at a time)
        # Note that instance is checked first to avoid requesting nonexistent attributes
        if isinstance(key, keyboard.KeyCode) and key.char in keybinds.keys():
            k = key.char
            if keybinds[k][0] == 0:
                self.joy.axes[keybinds[k][1]] = keybinds[k][2]
            elif keybinds[k][0] == 1:
                self.joy.buttons[keybinds[k][1]] = keybinds[k][2]
        elif isinstance(key, keyboard.Key) and key == keyboard.Key.esc: # If key==esc, exit
            sys.exit()
    
    def on_release(self, key):
        # If key in binds, reset to default value (only takes one key at a time)
        # Note that instance is checked first to avoid requesting nonexistent attributes
        if isinstance(key, keyboard.KeyCode) and key.char in keybinds.keys():
            k = key.char
            if keybinds[k][0] == 0:
                self.joy.axes[keybinds[k][1]] = self._default_axes[keybinds[k][1]]
            elif keybinds[k][0] == 1:
                self.joy.buttons[keybinds[k][1]] = self._default_buttons[keybinds[k][1]]

    def get_key(self):
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            key_timeout = 0.1
            settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            rlist, _, _ = select([sys.stdin], [], [], key_timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def key_to_joy(self):
        # Refresh joystick values to default
        self.joy = sensor_msgs.msg.Joy()
        self.joy.axes = np.copy(self.joy._default_axes)
        self.joy.buttons = np.copy(self.joy._default_buttons)

        k = self.get_key()
        # If key in binds, send command (only takes one key at a time)
        if k in keybinds.keys():
            if keybinds[k][0] == 0:
                self.joy.axes[keybinds[k][1]] = keybinds[k][2]
            elif keybinds[k][0] == 1:
                self.joy.buttons[keybinds[k][1]] = keybinds[k][2]
        elif k == key.ESC: # If key==esc, exit
            sys.exit()

        return self.joy
    
    def publish(self):
        self.joy_pub.publish(self.joy)

    def loop_once(self):
        self.key_to_joy()
        self.publish()
    
    def start(self):
        self.listener.start()
        rospy.loginfo("Keyboard listener started")
    
    def stop(self):
        self.listener.stop()
        rospy.loginfo("Keyboard listener stopped")

if __name__=="__main__":
    rospy.init_node('key_to_joy')

    key_to_joy = KeyToJoy()
    while not rospy.is_shutdown():
        key_to_joy.publish()
        key_to_joy.rate.sleep()
    key_to_joy.stop()
