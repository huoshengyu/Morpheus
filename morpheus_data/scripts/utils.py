#!/usr/bin/env python

# Keyboard input imports
from readchar import readkey, key

def get_key():
    k = readkey()
    return k

def input_args(func, keywords=[]):
    # Call a function and prompt the user to input argument values
    return

def sanitize_filename(input):
    valid_punctuation = "._ "
    input = input.strip()
    input = input.replace(' ', '_')
    filename = "".join(c for c in input if c.isalnum() or c in valid_punctuation)
    return filename