#!/usr/bin/env python
import os
from collections import deque
import rospy
from std_msgs.msg import String
from readchar import readkey, readchar, key
from utils import *
from ui_strings import *

class Session():
    def __init__(self, id="", rate=60):
        id = sanitize_filename(id)
        self._id = id
        self._task = ""
        self._trial_number = 0
        self._current_data = None   # unused now
        self._print_count = 0
        self._queue = deque()
        self._rate = rospy.Rate(rate)
        self._is_recording = False
        self._base_dir = rospy.get_param("~base_dir", "/root/catkin_ws/src/morpheus_data/data/")
        os.makedirs(self._base_dir, exist_ok=True)
        # Latched publisher for CSV basename (relative name!)
        self._csv_basename_pub = rospy.Publisher(
            "csv_basename", String, queue_size=1, latch=True
        )

    # ---------- helpers for filenames ----------
    def make_basename(self, id, task, trial):
        sid = sanitize_filename(str(id)) or "anon"
        stask = sanitize_filename(str(task)) or "task"
        try:
            tnum = int(trial)
        except Exception:
            tnum = 0
        return f"{sid}_{stask}_{tnum:03d}"

    def get_next_trial_number(self, id, task):
        """
        Return the smallest trial index whose CSVs don't exist yet.
        Uses <base>_joy_raw.csv as the canonical existence check.
        """
        trial = -1
        while True:
            trial += 1
            base = self.make_basename(id, task, trial)
            if not self._trial_exists(base):
                return trial
    def _trial_exists(self, base):
        candidates = [
            f"{base}_joy_raw.csv",
            f"{base}_armjoy_cmd.csv",
            f"{base}_joy_and_cmd.csv",
            f"{base}_cal_joy.csv",
        ]
        return any(os.path.exists(os.path.join(self._base_dir, fn)) for fn in candidates)

    def get_filename(self, id, task, trial_number):
        return "%s%s%s.bag" % (str(id), str(task), str(trial_number))

    def get_filepath(self, filename):
        dirname = os.path.dirname(__file__)
        datadir = os.path.join(os.path.dirname(dirname), 'data')
        return os.path.join(datadir, filename)

    # ---------- UI helpers (missing ones you crashed on) ----------
    def enqueue_string(self, queue, key_name, suffix=""):
        # session_string_dict is expected from ui_strings.py
        try:
            queue.append(session_string_dict[key_name] + str(suffix))
        except Exception:
            queue.append("")
        return queue

    def enqueue_header(self, queue, id, task, trial):
        queue = self.enqueue_string(queue, "header_title")
        queue = self.enqueue_string(queue, "header_id", id)
        queue = self.enqueue_string(queue, "header_task", task)
        queue = self.enqueue_string(queue, "header_trial", trial)
        return queue

    def dequeue(self, queue):
        # clear previous printed lines
        for _ in range(self._print_count):
            print("\x1b[1A\x1b[2K", end="")
        # print current buffer
        self._print_count = len(queue)
        for _ in range(len(queue)):
            print(queue.popleft())
        return queue

    def get_key(self):
        # simple wrapper; avoids NameError if you move input logic
        try:
            return readkey()
        except Exception:
            # If no TTY, you may want to handle differently (e.g., non-blocking mode)
            return None

    def handle_key_events(self, input_buf="", event=None):
        k = self.get_key()
        if k is None:
            return input_buf, event
        # Accept alnum and a small set of safe filename characters
        if isinstance(k, str) and (k.isalnum() or k in "._ "):
            input_buf += k
        elif k == key.BACKSPACE or k == key.DELETE:
            if len(input_buf) > 0:
                input_buf = input_buf[:-1]
        elif k == key.ENTER:
            event = key.ENTER
        elif k == key.ESC:
            event = key.ESC
        return input_buf, event

    # ---------- UI flow ----------
    def prompt(self, state="", id="", task="", trial=""):
        if state == "":
            state = "intro"
        input_buf = ""
        while not rospy.is_shutdown():
            self._queue = self.enqueue_header(self._queue, id, task, trial)

            if state == "recording":
                desc_key = "recording_on_description" if self._is_recording else "recording_off_description"
                if desc_key in session_string_dict:
                    self._queue = self.enqueue_string(self._queue, desc_key)
                else:
                    self._queue.append("▶ Recording... Press ENTER or ESC to stop." if self._is_recording
                                       else "Ready. Press ENTER to start. (ESC to exit)")
                status = "[REC]" if self._is_recording else "[IDLE]"
                try:
                    next_base = self.make_basename(id, task, trial)
                except Exception:
                    next_base = "(set id/task/trial)"
                self._queue.append(f"Status: {status} | CSV base: {next_base}")
            else:
                self._queue = self.enqueue_string(self._queue, state + "_description")

            self._queue.append(input_buf)
            self._queue = self.dequeue(self._queue)

            event = None
            input_buf, event = self.handle_key_events(input_buf, None)

            if state == "recording":
                if event == key.ENTER:
                    if self._is_recording:
                        self._stop_recording()
                        self.enqueue_string(self._queue, "recording_stopped")
                        trial = self.get_next_trial_number(id, task)
                        self._trial_number = trial
                    else:
                        self._start_recording(id, task, trial)
                        self.enqueue_string(self._queue, "recording_started")
                    continue
                if event == key.ESC:
                    self._stop_recording()
                    break
            else:
                if event == key.ENTER:
                    if state == "id":
                        id = sanitize_filename(input_buf)
                    elif state == "task":
                        task = sanitize_filename(input_buf)
                    elif state == "trial":
                        trial = input_buf
                        if trial == "":
                            trial = self.get_next_trial_number(id, task)
                        else:
                            try:
                                trial = int(trial)
                            except Exception:
                                pass
                        input_buf = ""
                        self.prompt("recording", id, task, trial)
                        continue
                    # session_state_dict is expected from ui_strings.py
                    for child_state in session_state_dict[state]:
                        self.prompt(child_state, id, task, trial)
                    input_buf = ""
                    continue
                if event == key.ESC:
                    break
            self._rate.sleep()

    # ---------- announce-only start/stop ----------
    def _start_recording(self, id, task, trial):
        base = self.make_basename(id, task, trial)
        self._csv_basename_pub.publish(String(data=base))

        self._id = sanitize_filename(str(id))
        self._task = sanitize_filename(str(task))
        try:
            self._trial_number = int(trial)
        except Exception:
            self._trial_number = 0
        self._is_recording = True

    def _stop_recording(self):
        # Tell loggers to close/idle
        self._csv_basename_pub.publish(String(data=""))
        self._is_recording = False

def main():
    rospy.init_node('UCDsession')
    session = Session()
    # Optional: also publish empty base on node shutdown to ensure CSVs close.
    rospy.on_shutdown(lambda: session._csv_basename_pub.publish(String(data="")))
    session.prompt()

if __name__ == '__main__':
    main()
