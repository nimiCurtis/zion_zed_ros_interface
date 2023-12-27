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


class RosBagRecord:
    """TBD...
        
        """    
    def __init__(self, topics_list, record_script_path, record_folder):
        """_summary_
        Args:
            cfg (DictConfig): configuration dictionary for recording process
        """        
        self.record_folder = record_folder      # use folder to store the bag from path in config        
        self.command = "source " + record_script_path +" "+" ".join(topics_list) # build rosbag command depend on the topic list        

    def start(self):
        
        self.p = subprocess.Popen(self.command, 
                                    stdin=subprocess.PIPE,
                                    cwd=self.record_folder,
                                    shell=True,
                                    executable='/bin/bash') 

        # effectively go into an infinite loop until it receives a shutdown signal


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

    def stop_recording_handler(self,record_runing=True):
        """This function execute the terminate function when ros shutdown
            """        
        # kill node
        
        if record_runing:
            self.terminate_ros_node("/record")
            rospy.loginfo("Saving bag..")

        else:
            rospy.loginfo("Record didn't run. Not saving bag and camera params")
        


def main():
    pass


if __name__ == '__main__':

    main()
