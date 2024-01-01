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
from zion_msgs.srv import RecordBag, RecordBagRequest, RecordBagResponse
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage
from RosBagRecorder import RosBagRecord
from rospy_message_converter import message_converter

PATH = os.path.dirname(__file__)
OmegaConf.register_new_resolver("path", lambda : PATH)

class ZedRecordManager(object):

    def __init__(self,cfg) -> None:
        
        topics_list_keys = cfg.recording.topics_to_rec
        
        topics = cfg.topics
        topics_list = [topics.get(k) for k in topics_list_keys]

        self.record_script = os.path.join(PATH,cfg.recording.script)           # use bash script from path in config
        self.record_folder = os.path.join(PATH,cfg.recording.bag_folder)       # use folder to store the bag from path in config
        self.recorded_cam_params_folder =os.path.join(PATH,cfg.recording.camera_params_folder)
        
        self.rosbag_record = RosBagRecord(topics_list=topics_list,
                                        record_script_path=self.record_script,
                                        record_folder=self.record_folder)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        self._is_recording = False
        self._ros_start_time = rospy.Time.now()
        self._ros_current_time = rospy.Time.now()

    def record(self):

        zed_node = "/zedm/zed_node"
        # Use rosnode to get the list of running nodes
        running_nodes = rosnode.get_node_names()
        try:
            
            r = rospy.Rate(0.5)
            while (not (zed_node in running_nodes)) and not(self.ctrl_c) :
                rospy.loginfo("{} is not runnning. Please check camera".format(zed_node))
                running_nodes = rosnode.get_node_names()
                r.sleep()

            if (not self.ctrl_c and (zed_node in running_nodes)):
                rospy.loginfo("{} is runnning properly.".format(zed_node))
                self._save_zed_params()
                self._save_camera_info()
                self.record_service = rospy.Service('/zion/record_node/record',RecordBag,self._record_callback)

                feedback_rate = 10. # secs
                r = rospy.Rate(1/feedback_rate)  # Rate to print the duration every 1 second
                while not self.ctrl_c:
                    if self._is_recording:
                        self._ros_current_time = rospy.Time.now()
                        dt = (self._ros_current_time - self._ros_start_time).to_sec()
                        rospy.loginfo("Recording for {:.2f} [secs]".format(dt))
                        r.sleep()

            rospy.spin()

        except rospy.ROSException as e:
            rospy.logwarn("ZED node didn't start")
            self.rosbag_record.stop_recording_handler(record_runing=self._is_recording)


    def shutdownhook(self):

        self.ctrl_c = True
        
        if self._is_recording:
            self.rosbag_record.stop_recording_handler(record_runing=self._is_recording)

        # Cleanup actions before exiting
        rospy.signal_shutdown("Shutting down " + rospy.get_name())

    def _record_callback(self,request:RecordBagRequest):
        
        if request.record:
            if not self._is_recording:
                self.rosbag_record.start()
                self._is_recording = True
                self._ros_start_time = rospy.Time.now()
            else:
                return RecordBagResponse(success=False, message="Already recording.")

        elif not request.record:
            if self._is_recording:
                self.rosbag_record.stop_recording_handler(record_runing=self._is_recording)
                self._is_recording = False
                self._ros_current_time = rospy.Time.now()
                dt = (self._ros_current_time - self._ros_start_time).to_sec()
                rospy.loginfo("Recording stopped after {:.2f} seconds.".format(dt))
            else:
                return RecordBagResponse(success=False, message="Not recording.")

        if request.record:
            response_message = "Received recording command, start recording."
        else:
            self._ros_current_time = rospy.Time.now()
            dt = (self._ros_current_time - self._ros_start_time).to_sec()
            response_message = "Received stop command, recording stopped after {:.2f} seconds.".format(dt)

        return RecordBagResponse(success=True, message=response_message)

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
        rospy.init_node('zion_recording_node')                # Init node
        rospy.loginfo('********** Starting node {} **********'.format(rospy.get_name()))

        zed_recorder_manager = ZedRecordManager(cfg)
        zed_recorder_manager.record()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
