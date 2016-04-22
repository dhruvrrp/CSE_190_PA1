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
        self.completion = rospy.Publisher(
                "/map_node/sim_complete",
                Bool,
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

    def getPDF(self, x, mean, sd):
        xu = x - mean
        xu2 = xu * xu
        var = sd * sd

        to_exp = -xu2/(2 * var)
        lft = 1 / (sd * math.sqrt(2 * math.pi))

        out = lft * math.exp(to_exp)

        return out

    def pipeToTemp(self, e):
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

        self.temperature_data.publish(message.temperature)
        self.texture_data.publish(texture)

        # PROBABILITY SHIT?????
        print_2d_floats(self.probGrid)
        if len(self.move_list) == 0:
#Should Sleep here
            self.completion.publish(True)
#Should Sleep and here too
            rospy.signal_shutdown("Done")

        for r in range(self.pg_rows):
            for c in range(self.pg_cols):
                probTex = self.config["prob_tex_correct"] if (texture == self.config["texture_map"]) else (1 - self.config["prob_tex_correct"])
                mean_tmp = self.config["pipe_map"][r][c]
                probTemp = self.getPDF(message.temperature, self.pipeToTemp(mean_tmp), self.config["temp_noise_std_dev"])
                self.probGrid[r][c] = self.probGrid[r][c] * probTex * probTemp

        rospy.wait_for_service('moveService')
        respo = self.move_service(self.move_list.pop(0))

        prob_move_correct = self.config["prob_move_correct"]
        tmpProb = [ [ value for __ in row ] for row in self.config["pipe_map"]]
        possible_moves = self.config["possible_moves"]

        for x in range(0, len(self.probGrid[0])):
            for y in range(0, len(self.probGrid)):
                for mv in possible_moves:
                    new_x = x + mv[0]
                    new_y = y + mv[1]
                    if new_x < 0:
                        new_x = len(self.probGrid[0])
                    if new_y < 0:
                        new_y = len(self.probGrid)
                    if new_x > len(self.probGrid[0]):
                        new_x = 0.0
                    if new_y > len(self.probGrid):
                        new_y = 0.0
                    if mv == cur_mov:
                        tmpProb[x][y] += pro_move_correct *
                                         self.probGrid[new_x][new_y]
                    else:
                        tmpProb[x][y] += ((1.0 - pro_mov_correct)/
                                         (len(possible_moves) - 1.0))

        self.probTable = RobotProbabilities()
        self.probTable.data = tmpProb

        self.probabilities.publish(self.probTable)



       # MORE PROBABILITY SHIT????????


if __name__ == '__main__':
    try:
        Ro = Robot()

    except rospy.ROSInterruptException:
        pass
