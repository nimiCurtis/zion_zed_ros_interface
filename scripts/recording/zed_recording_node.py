#!/usr/bin/env python3

# Standard library imports
import os
import yaml

# Related third party imports
import hydra
from omegaconf import OmegaConf

# ROS application/library specific imports
import rospy
import rosnode
import rosparam
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage
from RosBagRecorder import RosBagRecord
from rospy_message_converter import message_converter

PATH = os.path.dirname(__file__)
OmegaConf.register_new_resolver("path", lambda : PATH)

class ZedRecordManager(object):

    def __init__(self,cfg) -> None:
        
        rospy.loginfo("*** Parameters ***")
        rospy.loginfo(OmegaConf.to_yaml(cfg.recording, resolve=True))
        
        topics_list_keys = cfg.recording.topics_to_rec
        topics = cfg.topics
        topics_list = [topics.get(k) for k in topics_list_keys]

        self.record_script = os.path.join(PATH, cfg.recording.script)           # use bash script from path in config
        
        bag_folder = cfg.recording.bag_folder
        self.record_folder = os.path.join(PATH, bag_folder)       # use folder to store the bag from path in config

        self.recorded_cam_params_folder = os.path.join(self.record_folder, 'configs')        
        if not os.path.exists(self.recorded_cam_params_folder):
            os.makedirs(self.recorded_cam_params_folder)

        # self.recorded_cam_params_folder = os.path.join(PATH, cfg.recording.camera_params_folder)
        
        self.rosbag_record = RosBagRecord(topics_list=topics_list,
                                        record_script_path=self.record_script,
                                        record_folder=self.record_folder)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        self._is_recording = False
        self._ros_start_time = rospy.Time.now()
        self._ros_current_time = rospy.Time.now()
        rospy.loginfo("**************")

    def spin(self):

        zed_node = "/zedm/zed_node"
        # Use rosnode to get the list of running nodes
        running_nodes = rosnode.get_node_names()
        
        try:
            while (not (zed_node in running_nodes)) and not(self.ctrl_c) :
                rospy.loginfo_throttle(2,"{} is not runnning. Please check camera".format(zed_node))
                running_nodes = rosnode.get_node_names()

            if (not self.ctrl_c and (zed_node in running_nodes)):
                rospy.loginfo("{} is runnning properly.".format(zed_node))
                self._save_zed_params()
                self._save_camera_info()
                self.recording_service = rospy.Service('~record',SetBool,self._record_callback)

                while not rospy.is_shutdown():
                    if self._is_recording:
                        self._ros_current_time = rospy.Time.now()
                        dt = (self._ros_current_time - self._ros_start_time).to_sec()
                        rospy.loginfo_throttle(10,"Recording for {:.2f} [secs]".format(dt))

            rospy.spin()
        
        except rospy.ROSException as e:
            rospy.logwarn("ZED node didn't start")
            self.rosbag_record.stop_recording_handler(record_runing=self._is_recording)

    def shutdownhook(self):

        self.ctrl_c = True
        self.rosbag_record.stop_recording_handler(record_runing=self._is_recording)

        # Cleanup actions before exiting
        rospy.logwarn("Shutting down " + rospy.get_name())

    def _record_callback(self,request:SetBoolRequest):

        if request.data:
            if not self._is_recording:
                self.rosbag_record.start()
                self._is_recording = True
                self._ros_start_time = rospy.Time.now()
            else:
                return SetBoolResponse(success=False, message="Already recording.")

        elif not request.data:
            if self._is_recording:
                self.rosbag_record.stop_recording_handler(record_runing=self._is_recording)
                self._is_recording = False
                self._ros_current_time = rospy.Time.now()
                dt = (self._ros_current_time - self._ros_start_time).to_sec()
                rospy.loginfo("Recording stopped after {:.2f} seconds.".format(dt))
            else:
                return SetBoolResponse(success=False, message="Already not recording.")

        if request.data:
            response_message = "Received recording command, start recording."
        else:
            self._ros_current_time = rospy.Time.now()
            dt = (self._ros_current_time - self._ros_start_time).to_sec()
            response_message = "Received stop command, recording stopped after {:.2f} seconds.".format(dt)

        return SetBoolResponse(success=True, message=response_message)

    def _save_zed_params(self):
        # dump camera params
        rospy.loginfo("Saving camera params")
        if not os.path.exists(self.recorded_cam_params_folder):
            os.mkdir(self.recorded_cam_params_folder)
        rosparam.dump_params(self.recorded_cam_params_folder+"/zedm.yaml",param="zedm")

    def _save_camera_info(self):
        rospy.loginfo("Saving camera info and static TFs")
        if not os.path.exists(self.recorded_cam_params_folder):
            os.mkdir(self.recorded_cam_params_folder)

        camera_info_left = rospy.wait_for_message("/zedm/zed_node/left/camera_info",CameraInfo,timeout=5)
        camera_info_right = rospy.wait_for_message("/zedm/zed_node/right/camera_info",CameraInfo,timeout=5)
        info_dic={}

        tf_static_imu = rospy.wait_for_message("/tf_static",TFMessage,timeout=0.1)
        while len(tf_static_imu.transforms)!=1:
            tf_static_imu = rospy.wait_for_message("/tf_static",TFMessage,timeout=0.1)

        tf_static_base_to_optical = rospy.wait_for_message("/tf_static",TFMessage,timeout=0.1)
        while len(tf_static_base_to_optical.transforms)==1:
            tf_static_base_to_optical = rospy.wait_for_message("/tf_static",TFMessage,timeout=0.1)

        info_dic["tf_static_imu"] = message_converter.convert_ros_message_to_dictionary(tf_static_imu)
        info_dic["tf_static_base_to_optical"] = message_converter.convert_ros_message_to_dictionary(tf_static_base_to_optical)
        info_dic["left_camera_info"] = message_converter.convert_ros_message_to_dictionary(camera_info_left)
        info_dic["right_camera_info"] = message_converter.convert_ros_message_to_dictionary(camera_info_right)
        file_path = os.path.join(self.recorded_cam_params_folder,"camera_info.yaml")
        with open(file_path, 'w') as file:
            yaml.dump(info_dic, file)

# Use hydra for configuration managing
@hydra.main( version_base=None ,config_path="../../config/recorder_config", config_name = "record")
def main(cfg):

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        rospy.init_node('zed_recording_node')                # Init node
        rospy.loginfo('********** Starting node {} **********'.format(rospy.get_name()))
        zed_recorder_manager = ZedRecordManager(cfg)
        zed_recorder_manager.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
