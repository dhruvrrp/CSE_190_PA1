#!/usr/bin/env python
# license removed for brevity
import rospy
import random
from std_msgs.msg import Bool
from cse_190_assi_1.msg import temperatureMessage
from cse_190_assi_1.msg import RobotProbabilities
from read_config import read_config
from cse_190_assi_1.srv import requestTexture, moveService

from std_msgs.msg import String


from std_msgs.msg import Float32

class Robot():
    def __init__(self):

        self.config = read_config()
        rospy.init_node("robot")
        self.texture_requester = rospy.ServiceProxy(
                "requestTexture",
                requestTexture
        )
        self.move_service = rospy.ServiceProxy(
                "moveService",
                moveService
        )

        self.temperature_subscriber = rospy.Subscriber(
                "/temp_sensor/data",
                temperatureMessage,
                self.handle_temperature
        )
        self.temperature_activator = rospy.Publisher(
                "/temp_sensor/activation",
                Bool,
                queue_size = 10
        )
        self.temperature_data = rospy.Publisher(
                "/results/temperature_data",
                Float32,
                queue_size = 10
        )
        self.texture_data = rospy.Publisher(
                "/results/texture_data",
                String,
                queue_size = 10
        )
        self.probabilities = rospy.Publisher(
                "/results/probabilities",
                RobotProbabilities,
                queue_size = 10
        )

        self.move_list = self.config["move_list"]
        self.temp_message = temperatureMessage()
        self.sensor_on = False
        self.got_callback = True
        random.seed(self.config['seed'])
        self.rate = rospy.Rate(1)
        self.sensor_loop()


    def sensor_loop(self):

        while not rospy.is_shutdown():
            if self.sensor_on and self.got_callback:
                self.temperature_activator.publish(self.sensor_on)
                self.rate.sleep()


    def handle_temperature(self, message):

        self.got_callback = False
        text_response = texture_requester('temp')
        texture = text_response.data
        # PROBABILITY SHIT????? 
        
        rospy.wait_for_service('moveService')
        respo = self.move_service(self.move_list.pop(0))
 
        self.temperature_data.publish(message.data)       
        self.texture_data.publish(texture)
       # MORE PROBABILITY SHIT????????
        if len(self.move_list) == 0:
            rospy.signal_shutdown()

if __name__ == '__main__':
    try:
        Ro = Robot()
        
    except rospy.ROSInterruptException:
        pass
