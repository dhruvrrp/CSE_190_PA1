#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool
from cse_190_assi_1.msg import temperatureMessage

def callback(data):
    print "CALLBACK"
    print data

def robot():
    pub = rospy.Publisher('/temp_sensor/activation', Bool, queue_size=10)
    rospy.Subscriber("/temp_sensor/data", temperatureMessage, callback)

    rospy.init_node('robot', anonymous=True)
    rate = rospy.Rate(10)
    ar = True
    while not rospy.is_shutdown():
       # rospy.loginfo(ar)
        if ar: 
            pub.publish(ar)
            print "robot"
        rospy.Subscriber("/temp_sensor/data", temperatureMessage, callback)
        rate.sleep()
    rospy.spin()
   
        
def lir():

    rospy.Subscriber("/temp_sensor/data", temperatureMessage, callback)


if __name__ == '__main__':
    try:
        robot()
        
    except rospy.ROSInterruptException:
        pass
