#!/usr/bin/env python
# license removed for brevity
import math
import rospy
import random
from std_msgs.msg import Bool
from cse_190_assi_1.msg import temperatureMessage
from cse_190_assi_1.msg import RobotProbabilities
from read_config import read_config
from cse_190_assi_1.srv import requestTexture, moveService

from std_msgs.msg import String


from std_msgs.msg import Float32

from util import *

class Robot():
    def __init__(self):

        self.config = read_config()
        rospy.init_node("robot")
        self.pg_cols = len(self.config["pipe_map"][0])
        self.pg_rows = len(self.config["pipe_map"])
        value = 1.0 / (self.pg_cols * self.pg_rows)
        self.probGrid = [ [ value for __ in row ] for row in self.config["pipe_map"]]
        print_2d_floats(self.probGrid)
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
        self.sensor_on = True
        self.got_callback = True
        random.seed(self.config['seed'])
        self.rate = rospy.Rate(1)
        self.sensor_loop()


    def sensor_loop(self):

        while not rospy.is_shutdown():
            if self.sensor_on and self.got_callback:
                self.temperature_activator.publish(self.sensor_on)
                self.rate.sleep()

    def getPDF(x, mean, sd):
        xu = x - mean
        xu2 = xu * xu
        var = sd * sd

        to_exp = -xu2/(2 * var)
        lft = 1 / (sd * math.sqrt(2 * math.pi))

        out = lft * math.exp(to_exp)

        return out

    def pipeToTemp(e):
        if(e == 'H'):
            return 40
        elif(e == '-'):
            return 25
        elif(e == 'C'):
            return 20
        else:
            return 0

    def handle_temperature(self, message):

        self.got_callback = False
        text_response = self.texture_requester()
        texture = text_response.data
        # PROBABILITY SHIT?????

        for r in self.pg_rows:
            for c in self.pg_cols:
                mean_tmp = self.config["pipe_map"][r][c]
                probTemp = getPDF(message.data, pipeToTemp(mean_tmp), self.config["temp_noise_std_dev"])

        rospy.wait_for_service('moveService')
        respo = self.move_service(self.move_list.pop(0))

        self.temperature_data.publish(message.temperature)
        self.texture_data.publish(texture)
       # MORE PROBABILITY SHIT????????


        if len(self.move_list) == 0:
            rospy.signal_shutdown()

if __name__ == '__main__':
    try:
        Ro = Robot()

    except rospy.ROSInterruptException:
        pass
