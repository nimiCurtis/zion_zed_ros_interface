#!/usr/bin/env python3

"""
-TODO: 
- better config file managment
"""



# Libraries
import hydra
from hydra.utils import get_original_cwd, to_absolute_path

from hydra import compose, initialize
from omegaconf import OmegaConf

from omegaconf import DictConfig
import rospy
import rosparam
import subprocess
import os

PATH = os.path.dirname(__file__)
OmegaConf.register_resolver("path", lambda : PATH)
                            
class RosbagRecord:
    """TBD...
        
        """    
    def __init__(self, cfg):
        """_summary_
        Args:
            cfg (DictConfig): configuration dictionary for recording process
        """        
        topics_list_keys = cfg.recording.topics_to_rec
        topics = cfg.topics
        topics_list = [topics.get(k) for k in topics_list_keys]


        self.record_script = os.path.join(PATH,cfg.recording.script)           # use bash script from path in config
        self.record_folder = os.path.join(PATH,cfg.recording.bag_folder)       # use folder to store the bag from path in config
        self.recorded_cam_params_folder =os.path.join(PATH,cfg.recording.camera_params_folder)

        rospy.on_shutdown(self.stop_recording_handler)      # when ros shuting down execute the handler 
        
        command = "source " + self.record_script +" "+  " ".join(topics_list) # build rosbag command depend on the topic list
        
        # execute bash script for recording using the subprocess.popen module
        self.p = subprocess.Popen(command, 
                                    stdin=subprocess.PIPE,
                                    cwd=self.record_folder,
                                    shell=True,
                                    executable='/bin/bash') 
        
        rospy.spin() # effectively go into an infinite loop until it receives a shutdown signal
        
        

    def terminate_ros_node(self, s):
        """This function terminate the ros node starting with the given argument
            Args:
                s (string): first word of the target node to kill
            """        
        # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        # get topics to kill from 'rosnode list' using shell command.
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split(b"\n"):            # iterate topics using split(b"\n") | 'b' for byte type
            str_decode = str.decode('utf8')             # decode to string
            if (str_decode.startswith(s)):              # if it starts with string which 's' stored >> kill it
                os.system("rosnode kill " + str_decode) # kill node

    def stop_recording_handler(self):
        """This function execute the terminate function when ros shutdown
            """        
        # kill node
        rospy.loginfo("Ctrl-c detected")
        self.terminate_ros_node("/record")
        rospy.loginfo("Bag saved")

        # dump params
        rospy.loginfo("Saving camera configurations..")
        os.mkdir(self.recorded_cam_params_folder)
        rosparam.dump_params(self.recorded_cam_params_folder+"/zedm.yaml",param="zedm")

# Use hydra for configuration managing
@hydra.main( version_base=None ,config_path="../../config/recorder_config", config_name = "record")
def main(cfg):
    rospy.init_node('zion_recording_node')                # Init node
    rospy.loginfo(rospy.get_name() + ' start')  
    print(os.path.dirname(__file__))

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        rosbag_record = RosbagRecord(cfg)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':


    main()
