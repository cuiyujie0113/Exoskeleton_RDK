#!/usr/bin/python3

import rospy
import os
import h5py
import cv2
import argparse
import threading
import numpy as np
from std_msgs.msg import Float32MultiArray


class Episode_record:
    def __init__(self,task_name,eposide_num,with_motor=True,with_arm=False):

        self.record_T=rospy.Duration(1/25)
        self.with_motor=with_motor
        self.with_arm=with_arm

        self.task_dir=os.path.join(os.path.abspath("."),'data','hdf5',task_name)
        os.makedirs(self.task_dir,exist_ok=True)
        self.hdf5_file=os.path.join(self.task_dir,f"episode_{eposide_num}.hdf5")
        
        self.inhand_cam = cv2.VideoCapture("/dev/video2")
        print(self.inhand_cam)
        self.overview_cam = cv2.VideoCapture("/dev/video4")
        print(self.overview_cam)

        if not self.inhand_cam.isOpened():
            rospy.logerr("Cannot open inhand_camera")
        if not self.overview_cam.isOpened():
            rospy.logerr("Cannot open overview_camera")

        self.action=[]
        self.qpos=[]
        self.inhand_frames=[]
        self.overview_frames=[]
  
        # self.finger_sensor=None
        self.servo_pos_read=None
        self.servo_pos_cmd=None
        self.servo_read_sub=rospy.Subscriber("/control/hand/servo_read",Float32MultiArray,self.servo_read_callback)
        self.servo_cmd_sub=rospy.Subscriber("/control/hand/servo_cmd",Float32MultiArray,self.servo_cmd_callback)
        
        if self.with_motor:
            self.motor_vel_read=None
            self.motor_vel_cmd=None
            self.motor_read_sub=rospy.Subscriber("/control/hand/motor_read",Float32MultiArray,self.motor_read_callback)
            self.motor_cmd_sub=rospy.Subscriber("/control/hand/motor_cmd",Float32MultiArray,self.motor_cmd_callback)
        
        if self.with_arm:
            #TODO:JointState 
            self.arm_teacher_joints=None
            self.arm_follower_joints=None
            self.arm_teacher_sub=rospy.Subscriber("/control/arms/teacher_joint",Float32MultiArray,self.arm_teacher_callback)
            self.arm_follower_sub=rospy.Subscriber("/control/arms/follower_joint",Float32MultiArray,self.arm_follower_callback)

        self.inhand_lock=threading.Lock()
        self.overview_lock=threading.Lock()
        self.inhand_cam_event= threading.Event()
        self.overview_cam_event= threading.Event()
        self.timer_cam_thread_inhand = threading.Thread(target=self.inhand_cam_thread)
        self.timer_cam_thread_overview = threading.Thread(target=self.overview_cam_thread)
        self.save_end_event=threading.Event()
        self.timer_cam_thread_inhand.start()
        self.timer_cam_thread_overview.start()
        
    
    def __call__(self):
        self.timer=rospy.Timer(self.record_T, self.timer_callback)
        self.save_end_lock=threading.Lock()
        self.save_end_thread=threading.Thread(target=self.save_end,args=())
        self.save_end_thread.start()
        rospy.spin()

    def servo_read_callback(self,data):
        self.servo_pos_read=np.array(data.data, dtype=np.float32)
    
    def servo_cmd_callback(self,data):
        self.servo_pos_cmd=np.array(data.data, dtype=np.float32)

    def motor_read_callback(self,data):
        self.motor_vel_read=np.array(data.data, dtype=np.float32)

    def motor_cmd_callback(self,data):
        self.motor_vel_cmd=np.array(data.data, dtype=np.float32)

    def sensor_callback(self,data):
        self.finger_sensor = np.array(data.data, dtype=np.float32)
    
    def arm_teacher_callback(self,data):
        self.arm_teacher_joints = np.array(data.data, dtype=np.float32)

    def arm_follower_callback(self,data):
        self.arm_follower_joints = np.array(data.data, dtype=np.float32)  
    
    def timer_callback(self,event):
        self.inhand_cam_event.set()
        self.overview_cam_event.set() #low_dim data will be saved in this thread

    def inhand_cam_thread(self):
        while not self.save_end_event.is_set():
            self.inhand_cam_event.wait()
            self.inhand_cam_event.clear()

            with self.inhand_lock:
                ret, frame = self.inhand_cam.read()

                if not ret:
                    rospy.loginfo("inhand camera read error")
                    continue
                if ret:
                    encoded_frame=cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])[1]

            self.inhand_frames.append(encoded_frame.astype(np.uint8))
            print(f"inhand_frames={len(self.inhand_frames)}")
            print(self.inhand_frames[-1])

    def overview_cam_thread(self):
        while not self.save_end_event.is_set():
            self.overview_cam_event.wait()
            self.overview_cam_event.clear()

            with self.overview_lock:
                ret, frame = self.overview_cam.read()

                if not ret:
                    rospy.loginfo("overview camera read error")
                    continue
                if ret:
                    encoded_frame=cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])[1]

            self.overview_frames.append(encoded_frame.astype(np.uint8))
            print(f"overview_frames={len(self.overview_frames)}")
            print(self.overview_frames[-1])

            current_action=[]
            current_qpos=[]
            # arm_joints
            if self.with_arm:
                current_action.append(self.arm_teacher_joints)
                current_qpos.append(self.arm_follower_joints)
            # hand_servo
            current_action.append(self.servo_pos_cmd)
            current_qpos.append(self.servo_pos_read)
            # finger_motor
            if self.with_motor:
                current_action.append(self.motor_vel_cmd)
                current_qpos.append(self.motor_vel_read)

            # append current frame to episode record
            self.action.append(np.concatenate(current_action))
            self.qpos.append(np.concatenate(current_qpos))

    def zeropad_stack_arrays(self,array_list):
        max_len = max(len(arr) for arr in array_list)
        padded_arrays = [np.pad(arr, (0, max_len - len(arr)), 'constant') for arr in array_list]
        result=np.vstack(padded_arrays)
        return result.astype(np.uint8)

    def save_end(self):
        input("press Enter to stop recording")

        with self.save_end_lock:
            
            self.save_end_event.set()
            self.timer.shutdown()
            self.inhand_cam_event.set()
            self.overview_cam_event.set()

            
            self.inhand_cam.release() 
            self.overview_cam.release()

            self.action=np.vstack(self.action)
            self.qpos=np.vstack(self.qpos)
            self.inhand_frames=self.inhand_frames[len(self.inhand_frames)-len(self.overview_frames):]
            print(f"inhand_frames={len(self.inhand_frames)}")
            print(f"overview_frames={len(self.overview_frames)}")
            print(f"action_shape={self.action.shape}")
            print(f"qpos_shape={self.qpos.shape}")
            print(self.action[10])
            print(self.qpos[5])

            self.inhand_frames=self.zeropad_stack_arrays(self.inhand_frames)
            self.overview_frames=self.zeropad_stack_arrays(self.overview_frames)
            print(self.overview_frames.shape)
            print(self.inhand_frames.shape)

            rospy.loginfo("Start writing hdf5 file")
            with h5py.File(self.hdf5_file, 'w') as h5file:
                h5file.create_dataset('action', data=self.action)

                obser_grp = h5file.create_group('observations')
                obser_grp.create_dataset('qpos',data=self.qpos)

                images_grp=obser_grp.create_group("images")
                images_grp.create_dataset('inhand',data=self.inhand_frames)
                images_grp.create_dataset('overview',data=self.overview_frames)
            rospy.loginfo("Saving data finished")
            rospy.loginfo("Now you can quit")


TASK_NAME = "rotate"
EPISODE_NUM = 0
WITH_MOTOR = True
WITH_ARM = True
parser = argparse.ArgumentParser()
parser.add_argument('--task_name', type=str, default=TASK_NAME)
parser.add_argument('--episode_num', type=int, default=EPISODE_NUM)
parser.add_argument('--with_motor', type=bool, default=WITH_MOTOR)
parser.add_argument('--with_arm', type=bool, default=WITH_ARM)
            
if __name__=="__main__":
    # parse the command line params
    args = parser.parse_args()
    task_name = args.task_name
    episode_num = args.episode_num
    with_motor = args.with_motor
    with_arm = args.with_arm

    # init
    rospy.init_node('episode_record', anonymous=True)
    episode_record=Episode_record(task_name, episode_num, with_motor, with_arm)

    # wait keyboard input to start recording
    input("press Enter to start recording")
    episode_record()




            





            
            

             
    

        