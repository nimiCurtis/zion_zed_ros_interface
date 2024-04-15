// Copyright 2024 Nimrod Curtis

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// STD
#include <string>

// Custom
#include "zion_zed_ros_interface/ObjDetectPublisher.hpp"

namespace object_detect {

    ObjDetectConverter::ObjDetectConverter(ros::NodeHandle& nodeHandle)
        : nodeHandle_(nodeHandle),
        instance_id_(0),
        label_("Person")
    {   
        string node_name = ros::this_node::getName();
        ROS_INFO_STREAM("Starting " << node_name << "node." );
        
        // Read ros parameters
        if (!readParameters()) {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }
        

        // Check the topic of detection is alive
        auto message = ros::topic::waitForMessage<zed_interfaces::ObjectsStamped>(ObjDetSubTopic_,
                                        nodeHandle_, ros::Duration(10.0));
        while(ros::ok() && !message) {
            if (message) {
                ROS_INFO("Received Object message");
                break;  // Exit loop if message is received
            } else {
                ROS_WARN_STREAM("No message received on " << ObjDetSubTopic_ <<  " . Waiting...");
                auto message = ros::topic::waitForMessage<zed_interfaces::ObjectsStamped>(ObjDetSubTopic_,
                                        nodeHandle_, ros::Duration(3.0));
            }
        }

        // Subscribers
        obj_detect_subscriber_ = nodeHandle_.subscribe(ObjDetSubTopic_, 1,
                                        &ObjDetectConverter::objDetectCallback, this);
        
        // Publishers
        obj_detect_publisher_ = nodeHandle_.advertise<zed_interfaces::ObjectsStamped>(ObjDetPubTopic_, 10);

        // Services
        set_target_service_server_ = nodeHandle_.advertiseService(node_name +"/set_target_object",
                                                    &ObjDetectConverter::setTargetServiceCallback, this);

        ROS_INFO_STREAM("Successfully launched " << node_name);

    }

    ObjDetectConverter::~ObjDetectConverter()
    {
    }

    bool ObjDetectConverter::readParameters()
    {
        if (!nodeHandle_.getParam("topics/obj_detect_sub_topic", ObjDetSubTopic_)) return false;
        if (!nodeHandle_.getParam("topics/obj_detect_pub_topic", ObjDetPubTopic_)) return false;

        nodeHandle_.getParam("object/label", label_);
        nodeHandle_.getParam("object/instance_id", instance_id_);

        ROS_INFO_STREAM("******* Parameters *******");
        ROS_INFO_STREAM("* Topics:");
        ROS_INFO_STREAM("  * obj_detect_sub_topic: " << ObjDetSubTopic_);
        ROS_INFO_STREAM("  * obj_detect_pub_topic: " << ObjDetPubTopic_);
        ROS_INFO_STREAM("* Object:");
        ROS_INFO_STREAM("  * label " << label_);

        ROS_INFO_STREAM("  * Linear vel " << instance_id_);

        ROS_INFO_STREAM("**************************");
        return true;
    }


    void ObjDetectConverter::objDetectCallback(const zed_interfaces::ObjectsStamped& msg)
    {   
        zed_interfaces::ObjectsStamped obj_stamped_msg_;
        obj_stamped_msg_.header = msg.header;
        
        // Pull the specific instance from the objects list 
        for(auto obj = msg.objects.begin(); obj != msg.objects.end(); obj++){
            if((obj->label == label_) && (obj->instance_id==instance_id_)){
                obj_stamped_msg_.objects.push_back(*obj);
            }
        }

        // Publish
        obj_detect_publisher_.publish(obj_stamped_msg_);
    }

    bool ObjDetectConverter::setTargetServiceCallback(zion_msgs::set_target_objRequest& request,
                            zion_msgs::set_target_objResponse& response)
    {   

        if ( request.instance_id >=0 
            // && typeid(request.instance_id) == typeid(instance_id_)
            // && //typeid(request.label) == typeid(label_))
        ){
            label_ = request.label;
            instance_id_ = request.instance_id;
            response.info = "set target object to label: " + label_ + " | instance_id: " + to_string(instance_id_);
            response.result = true;
        } else {
            response.info = "fail. stay with object label: " + label_ + " | instance_id: " + to_string(instance_id_);
            response.result = false;
        }

        ROS_INFO_STREAM(response.info);
        return true;
    }


} /* namespace */