#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading
import random
import json
import re
import math
import time

from .pointclass import Gimbal 
ros_interrupt = False  # Global flag to pause/resume tracking

from OneEuroFilter import OneEuroFilter

config = {
    'freq': 10,       # Hz
    'mincutoff': 1.0,  # Hz
    'beta': 0.1,       
    'dcutoff': 1.0    
    }

f_x = OneEuroFilter(**config)
f_y = OneEuroFilter(**config)
f_z = OneEuroFilter(**config)

# ===================== Face Class =====================
class Face:
    def __init__(self):
        self.face_move_radius = 60
        self.target_tilt = 0
        self.target_yaw = 0
        self.current_tilt = 0
        self.current_yaw = 0
        self.gimbal = Gimbal()
        self.gimbal.open()
        self.gimbal.head_point(0, 0, 0)

    def _move_face(self):
        self.gimbal.head_point(0, self.current_yaw, self.current_tilt)

    def update_face_position(self):
        step = 0.15
        self.current_tilt += (self.target_tilt - self.current_tilt) * step
        self.current_yaw += (self.target_yaw - self.current_yaw) * step
        self._move_face()

    def look_at_3d_point(self, x, y, z):
        if z == 0:
            z = 0.01

        x = f_x(x, time.time())
        y = f_y(y, time.time())
        z = f_z(z, time.time())

        # Calculate tilt and yaw based on normalized coordinates
        yaw = math.atan( x / z )
        tilt = math.atan( y / z )
        dt = max(-1, min(1, yaw)) * self.face_move_radius
        db = max(-1, min(1, tilt)) * self.face_move_radius
        self.target_tilt = dt
        self.target_yaw = db

# ===================== ROS 2 Node =====================
class FaceNode(Node):
    def __init__(self):
        super().__init__('face_tracker_node')
        self.subscription = self.create_subscription(
            String,
            '/pzgaze/face_info',
            self.gaze_callback,
            10
        )
        self.latest_coords = (0.0, 0.0, 2.0)  # default position
        self.target_face_id = 0

    def gaze_callback(self, msg):
        try:
            face_data = self.parse_face_info(msg.data)
            if not face_data:
                return
            
            target_face = None
            for face in face_data:
                if face['face_id'] == self.target_face_id:
                    target_face = face
                    break

            cx = -target_face['center_x']
            cy = target_face['center_y']
            cz = target_face['center_z']
            
            self.latest_coords = (float(cx), float(cy), float(cz))
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse JSON from eth_gaze")
    
    def parse_face_info(self, data: str) -> list:
        faces = []
        
        face_count_match = re.search(r'faces_count:(\d+)', data)
        if not face_count_match:
            return faces
        
        face_count = int(face_count_match.group(1))
        
        for i in range(face_count):
            face_pattern = rf'face_{i}:([^|]+)'
            face_match = re.search(face_pattern, data)
            
            if face_match:
                face_info_str = face_match.group(1)
                face_info = self._parse_single_face(face_info_str, i)
                if face_info:
                    faces.append(face_info)
        
        return faces
    
    def _parse_single_face(self, face_str: str, face_id: int) -> dict:
        try:
            pitch_match = re.search(r'pitch=([-\d.]+)', face_str)
            yaw_match = re.search(r'yaw=([-\d.]+)', face_str)
            distance_match = re.search(r'distance=([-\d.]+)', face_str)
            center_match = re.search(r'center=\(([-\d.]+),([-\d.]+),([-\d.]+)\)', face_str)
            gaze_pitch_match = re.search(r'gaze_pitch=([-\d.]+)', face_str)
            gaze_yaw_match = re.search(r'gaze_yaw=([-\d.]+)', face_str)
            
            if not all([pitch_match, yaw_match, distance_match, center_match]):
                return None
            
            return {
                'face_id': face_id,
                'head_pitch': float(pitch_match.group(1)),
                'head_yaw': float(yaw_match.group(1)),
                'distance': float(distance_match.group(1)),
                'center_x': float(center_match.group(1)),
                'center_y': float(center_match.group(2)),
                'center_z': float(center_match.group(3)),
                'gaze_pitch': float(gaze_pitch_match.group(1)) if gaze_pitch_match else None,
                'gaze_yaw': float(gaze_yaw_match.group(1)) if gaze_yaw_match else None
            }
        except Exception as e:
            self.get_logger().error(f"Error parsing face info: {str(e)}")
            return None

    def get_latest_coords(self):
        return self.latest_coords
    
# =================== Tracking Setup ===================
def update_animation():
    x, y, z = ros_node.get_latest_coords()
    print(f"Tracking coordinates: x={x}, y={y}, z={z}")
    Face.look_at_3d_point(x, y, z)
    Face.update_face_position()

# ===================== Main Entry =====================

def main(args=None):
    global ros_node, face_tracker, ros_interrupt
    rclpy.init(args=args)

    try:
        while rclpy.ok():
            if not ros_interrupt:
                # === Reinitialize tracking ===
                ros_node = FaceNode()
                face_tracker = Face()

                # Spin ROS 2 in background thread
                spin_thread = threading.Thread(
                    target=rclpy.spin,
                    args=(ros_node,),
                    daemon=True
                )
                spin_thread.start()

                # Tracking loop until interrupted
                while rclpy.ok() and not ros_interrupt:
                    x, y, z = ros_node.get_latest_coords()
                    face_tracker.look_at_3d_point(x, y, z)
                    face_tracker.update_face_position()
                    time.sleep(0.02)  # ~50 Hz update rate

                # Stop and clean up ROS node when interrupted
                ros_node.destroy_node()

            else:
                # Paused — just wait until flag changes
                time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()