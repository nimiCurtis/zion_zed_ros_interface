#!/usr/bin/env python3

"""
-TODO: 
"""



# Libraries
import hydra
from omegaconf import OmegaConf

from omegaconf import DictConfig
import rospy
import rosparam
import subprocess
import os
import json
import yaml
from zion_msgs.srv import RecordBag, RecordBagRequest, RecordBagResponse
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from RosBagRecorder import RosBagRecord
from rospy_message_converter import message_converter

PATH = os.path.dirname(__file__)
OmegaConf.register_resolver("path", lambda : PATH)


class ZedRecordManager():
    
    def __init__(self,cfg) -> None:
        
        topics_list_keys = cfg.recording.topics_to_rec
        
        topics = cfg.topics
        topics_list = [topics.get(k) for k in topics_list_keys]

        self.record_script = os.path.join(PATH,cfg.recording.script)           # use bash script from path in config
        self.record_folder = os.path.join(PATH,cfg.recording.bag_folder)       # use folder to store the bag from path in config
        self.recorded_cam_params_folder =os.path.join(PATH,cfg.recording.camera_params_folder)
        
        rospy.Subscriber('/tf_static', TFMessage, self._tf_static_callback)
        self.rosbag_record = RosBagRecord(topics_list=topics_list,
                                        record_script_path=self.record_script,
                                        record_folder=self.record_folder)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        self._is_runing = False

        
    def shutdownhook(self):
        rospy.logwarn(rospy.get_name() + ' was shutdown by user')
        self.ctrl_c = True
        
        if self._is_runing:
            self.rosbag_record.stop_recording_handler(record_runing=self._is_runing)
    
    def _tf_static_callback(self,msg:TFMessage):
        rospy.wait_for_message("/tf_static",TFMessage,timeout=3)
        self.tf_static = msg       
    
    def _record_callback(self,request:RecordBagRequest):
        frames_num = request.frames_num
        record_duration = request.duration 
        resp = RecordBagResponse()
        
        resp.success = False
        resp.print = "Didn't complete properly"
        
        if frames_num==-1 and record_duration!=-1:
            self._record_duration_depend(record_duration)
            resp.success = True
            resp.print = f"Reach {record_duration} secs. Bag Saved to {self.rosbag_record.record_folder}"


        elif frames_num!=-1 and record_duration==-1:
            self._record_num_depend(frames_num)
            resp.success = True
            resp.print = f"Reach {frames_num} images. Bag saved to {self.rosbag_record.record_folder} "
            
        else:
            self._record_non_stop()
            resp.success = True
            resp.print = f"Bag saved to {self.rosbag_record.record_folder} "
            
        return resp
        
    def _record_num_depend(self, max_frame):
        
        rospy.loginfo(f"Start recording process of {max_frame} images")
        
        self._frame_count = 0
        self.rosbag_record.start()
        self._is_runing = True
        
        while self._frame_count <= max_frame: 
            self.depth_img = rospy.wait_for_message("/zedm/zed_node/depth/depth_registered",Image)
            self._frame_count+=1
        
        self.rosbag_record.stop_recording_handler()
        self._is_runing = False

    def _record_duration_depend(self, duration):
        rospy.loginfo("Start recording process for {} secs")

        last_time = rospy.Time.now()
        current_time = rospy.Time.now()
        dt  = (current_time - last_time).to_sec()
        
        self.rosbag_record.start()
        self._is_runing = True
        
        
        while dt <= duration: 
            current_time = rospy.Time.now()
            dt  = (current_time - last_time).to_sec()
        
        self.rosbag_record.stop_recording_handler()
        self._is_runing = False
        
    def _record_non_stop(self):
        
        rospy.loginfo("Start recording process")
        
        self.rosbag_record.start()
        self._is_runing = True
    
    def record(self):
        
        try:
            if not self.ctrl_c:
                self.depth_img = rospy.wait_for_message("/zedm/zed_node/depth/depth_registered",Image,timeout=5)
                self.save_zed_params()
                self.save_camera_info()
                self.record_service = rospy.Service('/zion/record_node/record',RecordBag,self._record_callback)

            rospy.spin()
            
        except rospy.ROSException as e:
            rospy.logwarn("ZED node didn't start")
            self.rosbag_record.stop_recording_handler(record_runing=self._is_runing)

            
    def save_zed_params(self):
        # dump camera params
        rospy.loginfo("Saving camera params..")
        if not os.path.exists(self.recorded_cam_params_folder):
            os.mkdir(self.recorded_cam_params_folder)
        rosparam.dump_params(self.recorded_cam_params_folder+"/zedm.yaml",param="zedm")
    
    def save_camera_info(self):
        rospy.loginfo("Saving camera info and static TFs..")
        if not os.path.exists(self.recorded_cam_params_folder):
            os.mkdir(self.recorded_cam_params_folder)
            

        camera_info_left = rospy.wait_for_message("/zedm/zed_node/left/camera_info",CameraInfo,timeout=5)
        camera_info_right = rospy.wait_for_message("/zedm/zed_node/right/camera_info",CameraInfo,timeout=5)
        info_dic={}
        
        
        info_dic["tf_static"] = message_converter.convert_ros_message_to_dictionary(self.tf_static)
        info_dic["left_camera_info"] = message_converter.convert_ros_message_to_dictionary(camera_info_left)
        info_dic["right_camera_info"] = message_converter.convert_ros_message_to_dictionary(camera_info_right)
        file_path = os.path.join(self.recorded_cam_params_folder,"camera_info.yaml")
        with open(file_path, 'w') as file:
            documents = yaml.dump(info_dic, file)
        
# Use hydra for configuration managing
@hydra.main( version_base=None ,config_path="../../config/recorder_config", config_name = "record")
def main(cfg):

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        rospy.init_node('zion_recording_node')                # Init node
        rospy.loginfo(rospy.get_name() + ' start')
        
        zed_recorder_manager = ZedRecordManager(cfg)
        zed_recorder_manager.record()
        
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
