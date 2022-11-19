#!/usr/bin/env python3

import rospy
from dr_spaam_ros.dr_spaam_ros import DrSpaamROS
from dr_spaam_ros.tracker import PersonTracker


if __name__ == '__main__':
    rospy.init_node('dr_spaam_ros')
    try:
        person_tracker = PersonTracker()
        DrSpaamROS(person_tracker)
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
