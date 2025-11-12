#!/usr/bin/env python3
import os
import csv
import threading
import time
import traceback
from collections import deque

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from pylsl import StreamInfo, StreamOutlet, resolve_stream, StreamInlet

# ==========================================================
# fallback utils / strings (in case your package ones aren't importable)
# ==========================================================
try:
    from utils import sanitize_filename
    from ui_strings import session_string_dict, session_state_dict
except Exception:
    def sanitize_filename(s):
        return "".join(c for c in s if c.isalnum() or c in ("_", "-")).rstrip()

    session_string_dict = {
        "header_title": "=== UCD session ===",
        "header_id": "ID: ",
        "header_task": "Task: ",
        "header_trial": "Trial: ",
        "intro_description": "Enter ID, task, and trial.",
        "id_description": "Enter subject ID and press ENTER:",
        "task_description": "Enter task name and press ENTER:",
        "trial_description": "Enter trial number and press ENTER:",
        "recording_on_description": "▶ Recording... Press ENTER to stop, ESC to exit.",
        "recording_off_description": "Ready. Press ENTER to start. (ESC to exit)",
    }
    session_state_dict = {
        "intro": ["id"],
        "id": ["task"],
        "task": ["trial"],
    }

# readchar may not exist on the robot, so be defensive
try:
    from readchar import readkey, key
except Exception:
    class DummyKey:
        ENTER = "\n"
        ESC = "\x1b"
        BACKSPACE = "\x08"
        DELETE = "\x7f"
    key = DummyKey()
    def readkey():
        return None


# ==========================================================
# Session: UI + joystick→event LSL
# ==========================================================
class Session(object):
    def __init__(self, id_="", rate=60):
        id_ = sanitize_filename(id_)
        self._id = id_
        self._task = ""
        self._trial_number = 0
        self._print_count = 0
        self._queue = deque()
        self._rate = rospy.Rate(rate)
        self._is_recording = False
        self._base_dir = rospy.get_param("~base_dir", "/root/catkin_ws/src/morpheus_data/data/")
        os.makedirs(self._base_dir, exist_ok=True)

        # comment handling
        self._awaiting_comment = False
        self._pending_trial_base = ""
        self._comment_buf = ""

        # ROS pubs
        self._csv_basename_pub = rospy.Publisher("csv_basename", String, queue_size=1, latch=True)
        self._csv_trial_comment_pub = rospy.Publisher("csv_trial_comment", String, queue_size=1, latch=True)

        # ===== LSL event outlet (only button presses) =====
        evt_info = StreamInfo(
            name='Controller_Events',
            type='event',
            channel_count=1,
            nominal_srate=0,
            channel_format='float32',
            source_id='ps3_events_01'
        )
        self._lsl_event_outlet = StreamOutlet(evt_info)
        self._prev_buttons = None  # to detect rising edges

        # define which buttons = which event
        # TODO: set these to your actual indices from your Joy message
        self.SHARE_BTN = 8   # example index
        self.OPEN_BTN  = 1   # example index
        self.CLOSE_BTN = 3   # example index
        self.HOME_BTN  = 9

    # ---------- helpers ----------
    def make_basename(self, id_, task, trial):
        sid = sanitize_filename(str(id_)) or "anon"
        stask = sanitize_filename(str(task)) or "task"
        try:
            tnum = int(trial)
        except Exception:
            tnum = 0
        return f"{sid}_{stask}_{tnum:03d}"

    def enqueue_string(self, queue, key_name, suffix=""):
        try:
            queue.append(session_string_dict[key_name] + str(suffix))
        except Exception:
            queue.append("")
        return queue

    def enqueue_header(self, queue, id_, task, trial):
        queue = self.enqueue_string(queue, "header_title")
        queue = self.enqueue_string(queue, "header_id", id_)
        queue = self.enqueue_string(queue, "header_task", task)
        queue = self.enqueue_string(queue, "header_trial", trial)
        return queue

    def dequeue(self, queue):
        for _ in range(self._print_count):
            print("\x1b[1A\x1b[2K", end="")
        self._print_count = len(queue)
        for _ in range(len(queue)):
            print(queue.popleft())
        return queue

    def get_key(self):
        try:
            return readkey()
        except Exception:
            return None

    def handle_key_events(self, input_buf="", event=None):
        k = self.get_key()
        if k is None:
            return input_buf, event
        if isinstance(k, str) and k not in (key.ENTER, key.ESC):
            input_buf += k
        elif k in (key.BACKSPACE, key.DELETE):
            if len(input_buf) > 0:
                input_buf = input_buf[:-1]
        elif k == key.ENTER:
            event = key.ENTER
        elif k == key.ESC:
            event = key.ESC
        return input_buf, event

    # ---------- ROS Joy → LSL events ----------
    def joy_cb(self, msg: Joy):
        # if we don't have a previous state yet, assume all buttons were 0
        if self._prev_buttons is None:
            self._prev_buttons = [0] * len(msg.buttons)
            # don't return — we want to detect a press even on the first message

        def pressed(idx):
            return (
                idx < len(msg.buttons)
                and msg.buttons[idx] == 1
                and self._prev_buttons[idx] == 0
            )

        # choose event codes
        if pressed(self.SHARE_BTN):
            ts = rospy.get_time()
            self._lsl_event_outlet.push_sample([1.0], ts)
        if pressed(self.OPEN_BTN):
            ts = rospy.get_time()
            self._lsl_event_outlet.push_sample([2.0], ts)
        if pressed(self.CLOSE_BTN):
            ts = rospy.get_time()
            self._lsl_event_outlet.push_sample([3.0], ts)
        if pressed(self.HOME_BTN):
            ts = rospy.get_time()
            self._lsl_event_outlet.push_sample([4.0], ts)

        # update previous
        self._prev_buttons = list(msg.buttons)

    # ---------- announce-only start/stop ----------
    def _start_recording(self, id_, task, trial):
        base = self.make_basename(id_, task, trial)
        self._csv_basename_pub.publish(String(data=base))
        self._is_recording = True

    def _stop_recording(self):
        self._csv_basename_pub.publish(String(data=""))
        self._is_recording = False

    # ---------- UI loop ----------
    def prompt(self, state="", id_="", task="", trial=""):
        if state == "":
            state = "intro"
        input_buf = ""
        while not rospy.is_shutdown():
            self._queue = self.enqueue_header(self._queue, id_, task, trial)

            if self._awaiting_comment:
                self._queue.append("Enter trial comment (e.g. 'good' or 'bad'):")
                self._queue.append(self._comment_buf)
            elif state == "recording":
                keyname = "recording_on_description" if self._is_recording else "recording_off_description"
                self._queue = self.enqueue_string(self._queue, keyname)
                status = "[REC]" if self._is_recording else "[IDLE]"
                next_base = self.make_basename(id_, task, trial)
                self._queue.append(f"Status: {status} | CSV base: {next_base}")
            else:
                self._queue = self.enqueue_string(self._queue, state + "_description")

            self._queue.append(input_buf)
            self._queue = self.dequeue(self._queue)

            event = None

            if self._awaiting_comment:
                self._comment_buf, event = self.handle_key_events(self._comment_buf, None)
                if event == key.ENTER:
                    msg = f"{self._pending_trial_base}::{self._comment_buf}"
                    self._csv_trial_comment_pub.publish(String(data=msg))
                    # save to file
                    try:
                        base, comment = msg.split("::", 1)
                        comment_file = os.path.join(self._base_dir, f"{base}_comment.txt")
                        with open(comment_file, "w") as f:
                            f.write(comment.strip() + "\n")
                        print("Saved comment to:", comment_file)
                    except Exception as e:
                        print("Failed to save comment:", e)
                    self._awaiting_comment = False
                    self._pending_trial_base = ""
                    self._comment_buf = ""
                elif event == key.ESC:
                    self._awaiting_comment = False
                    self._pending_trial_base = ""
                    self._comment_buf = ""
            else:
                input_buf, event = self.handle_key_events(input_buf, None)
                if state == "recording":
                    if event == key.ENTER:
                        if self._is_recording:
                            finished_base = self.make_basename(id_, task, trial)
                            self._stop_recording()
                            self._pending_trial_base = finished_base
                            self._awaiting_comment = True
                            try:
                                trial = int(trial)
                            except Exception:
                                trial = 0
                            trial += 1
                        else:
                            self._start_recording(id_, task, trial)
                        continue
                    if event == key.ESC:
                        self._stop_recording()
                        break
                else:
                    if event == key.ENTER:
                        if state == "id":
                            id_ = sanitize_filename(input_buf)
                        elif state == "task":
                            task = sanitize_filename(input_buf)
                        elif state == "trial":
                            try:
                                trial = int(input_buf or 0)
                            except Exception:
                                trial = 0
                            input_buf = ""
                            self.prompt("recording", id_, task, trial)
                            continue
                        for child_state in session_state_dict.get(state, []):
                            self.prompt(child_state, id_, task, trial)
                        input_buf = ""
                        continue
                    if event == key.ESC:
                        break

            self._rate.sleep()


# ==========================================================
# Multi LSL Recorder: Neon Gaze + Controller_Events → ONE CSV
# ==========================================================
class MultiLSLRecorder(object):
    """
    Writes rows whenever a sample arrives.
    Columns:
      ros_time,
      sys_utc_ns,
      neon_time, neon_ch0..,
      evt_time, evt_ch0
    """
    def __init__(self, save_dir):
        self.save_dir = save_dir
        os.makedirs(self.save_dir, exist_ok=True)

        self.stream_specs = [
            {"match": "Neon Companion_Neon Gaze", "label": "neon"},
            {"match": "Controller_Events", "label": "evt"},
        ]

        self.inlets = {}
        self.ch_counts = {}
        self.last_samples = {}
        self.last_ts = {}

        self.current_file = None
        self.current_writer = None
        self.current_basename = ""
        self.lock = threading.Lock()

        self._connect_streams_once(print_all=True)

        rospy.Subscriber("csv_basename", String, self.basename_cb)

        self.keep_running = True
        self.reconnect_thread = threading.Thread(target=self._reconnect_loop_safe)
        self.reconnect_thread.daemon = True
        self.reconnect_thread.start()

    def _connect_streams_once(self, print_all=False):
        try:
            streams = resolve_stream()
        except Exception:
            rospy.logerr("[MultiLSL] resolve_stream failed:\n%s", traceback.format_exc())
            return

        if print_all:
            rospy.loginfo("[MultiLSL] Available LSL streams right now:")
            for s in streams:
                rospy.loginfo("   - %s (%s)", s.name(), s.type())

        for spec in self.stream_specs:
            match = spec["match"]
            label = spec["label"]
            if label in self.inlets:
                continue

            chosen = None
            # exact match
            for s in streams:
                if match.lower() == s.name().lower():
                    chosen = s
                    break
            # substring match
            if chosen is None:
                for s in streams:
                    if match.lower() in s.name().lower():
                        chosen = s
                        break

            if chosen is None:
                rospy.logwarn("[MultiLSL] stream '%s' not found yet.", match)
                continue

            try:
                inlet = StreamInlet(chosen, max_buflen=360)
                ch_count = inlet.info().channel_count()
                self.inlets[label] = inlet
                self.ch_counts[label] = ch_count
                self.last_samples[label] = ["" for _ in range(ch_count)]
                self.last_ts[label] = ""
                rospy.loginfo("[MultiLSL] CONNECTED to '%s' as '%s' with %d channels",
                              chosen.name(), label, ch_count)
            except Exception:
                rospy.logerr("[MultiLSL] failed to connect to '%s':\n%s", match, traceback.format_exc())

    def _reconnect_loop_safe(self):
        try:
            while self.keep_running and not rospy.is_shutdown():
                missing = [s for s in self.stream_specs if s["label"] not in self.inlets]
                if missing:
                    self._connect_streams_once(print_all=False)
                time.sleep(2.0)
        except Exception:
            rospy.logerr("[MultiLSL] reconnect crashed:\n%s", traceback.format_exc())

    def basename_cb(self, msg: String):
        name = msg.data.strip()
        with self.lock:
            if name == "":
                if self.current_file is not None:
                    self.current_file.close()
                    self.current_file = None
                    self.current_writer = None
                    self.current_basename = ""
                    rospy.loginfo("[MultiLSL] closed file")
                return

            if self.current_file is not None:
                self.current_file.close()

            out_path = os.path.join(self.save_dir, f"{name}_lsl.csv")
            self.current_file = open(out_path, "w")
            self.current_writer = csv.writer(self.current_file)

            # header: ros_time, sys_utc_ns, then per-stream
            header = ["ros_time", "sys_utc_ns"]
            for spec in self.stream_specs:
                lbl = spec["label"]
                header.append(f"{lbl}_time")
                ch_n = self.ch_counts.get(lbl, 0)
                for i in range(ch_n):
                    header.append(f"{lbl}_ch{i}")
            self.current_writer.writerow(header)
            self.current_basename = name
            rospy.loginfo("[MultiLSL] recording to %s", out_path)

    def run(self):
        try:
            rate = rospy.Rate(300)
            while not rospy.is_shutdown():
                for spec in self.stream_specs:
                    label = spec["label"]
                    inlet = self.inlets.get(label)
                    if inlet is None:
                        continue

                    sample, ts = inlet.pull_sample(timeout=0.0)
                    if sample is None:
                        continue

                    self.last_samples[label] = sample
                    self.last_ts[label] = ts

                    with self.lock:
                        if self.current_writer is None:
                            continue

                        # 1) ROS time
                        row = [rospy.get_time()]
                        # 2) system UTC (epoch) in ns – the closest we can get in this script
                        row.append(time.time_ns())

                        # 3) all streams’ LSL times + channels
                        for spec2 in self.stream_specs:
                            lbl2 = spec2["label"]
                            row.append(self.last_ts.get(lbl2, ""))
                            ch_n = self.ch_counts.get(lbl2, 0)
                            vals = list(self.last_samples.get(lbl2, []))
                            if len(vals) < ch_n:
                                vals += [""] * (ch_n - len(vals))

                            for i, v in enumerate(vals[:ch_n]):
                                # ✅ round neon_ch0 and neon_ch1 to 4 decimals
                                if lbl2 == "neon" and i in (0, 1) and isinstance(v, (int, float)):
                                    row.append(round(v, 3))
                                elif isinstance(v, (int, float)):
                                    row.append(v)
                                else:
                                    row.append(str(v))

                        self.current_writer.writerow(row)
                rate.sleep()
        except Exception:
            rospy.logerr("[MultiLSL] run crashed:\n%s", traceback.format_exc())
# ==========================================================
# main
# ==========================================================
def main():
    rospy.init_node('UCDsession')

    session = Session()
    rospy.Subscriber("/vx300s/joy", Joy, session.joy_cb)

    recorder = MultiLSLRecorder(save_dir=session._base_dir)
    rec_thread = threading.Thread(target=recorder.run)
    rec_thread.daemon = True
    rec_thread.start()

    rospy.on_shutdown(lambda: session._csv_basename_pub.publish(String(data="")))

    session.prompt()


if __name__ == "__main__":
    main()
