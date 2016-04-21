#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool
from cse_190_assi_1.msg import temperatureMessage
from cse_190_assi_1.msg import RobotProbabilities

class Robot():
    def __init__(self):

        self.config = read_config()
        rospy.init_node("robot")
        self.texture_requester = rospy.ServiceProxy(
                "requestTexture",
                requestTexture
        )
        self.temperature_subscriber = rospy.Subscriber(
                "/temp_sensor/data",
                Bool,
                self.handle_temperature
        )
        self.temperature_activator = rospy.Publisher(
                "/temp_sensor/activation",
                temperatureMessage,
                queue_size = 10
        )
        self.temperature_data = rospy.Publisher(
                "/results/temperature_data",
                temperatureMessage,
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
        





if __name__ == '__main__':
    try:
        robot()
        
    except rospy.ROSInterruptException:
        pass
