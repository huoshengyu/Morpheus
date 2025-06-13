#!/usr/bin/env python

# Keyboard input imports
from readchar import readkey, readchar, key

def get_key():
    k = readchar()
    return k

def sanitize_filename(input):
    valid_punctuation = "._ "
    input = input.strip()
    input = input.replace(' ', '_')
    filename = "".join(c for c in input if c.isalnum() or c in valid_punctuation)
    return filename