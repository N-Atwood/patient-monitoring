import rclpy
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Person
import time

class PatientSubscriber:
    def __init__(self):
        self.ang_thresh = 50
        self.max_ang = .2
        self.opt_dist = 1750
        self.dist_thresh = 250
        self.max_vel = 3.
        self.last_spotted_x = 0
        self.last_spotted_time = 0
        self.time_thresh = 0.2
        self.node = rclpy.create_node('patient_subscriber')
        self.subscription = self.node.create_subscription(
            Person,
            'person',
            self.person_callback,
            1)
        self.x_sign = 0

        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 1) # If the robot falls behind, we'd rather have it follow the newest command than old ones
        
    def person_callback(self,msg_in):
        msg_out = Twist()

        #If we can't see anyone, just start spinning
        if msg_in.x == -1 :
            if self.last_spotted_x == 1:
                print("rotate LEFT ***PATIENT OUT OF FRAME***")
                msg_out.angular.z = self.max_ang
                msg_out.linear.x = 0.

            if self.last_spotted_x == 2:
                print("rotate RIGHT ***PATIENT OUT OF FRAME***")
                msg_out.angular.z = (-1.) * self.max_ang
                msg_out.linear.x = 0.
        else:
            self.last_spotted_time = time.time()

            # Assuming left-handed coordinate system

            # If the person is too far to the left, turn left
            if msg_in.x < 212 - self.ang_thresh :
                print("rotate LEFT, msg_in.x==", msg_in.x)
                msg_out.angular.z = self.max_ang #self.max_ang * (0.5 - msg_in.x + self.ang_thresh)
                self.last_spotted_x = 1
                self.x_sign = 0
            # If the person is too far to the right, turn right
            elif msg_in.x > 212 + self.ang_thresh :
                print("rotate RIGHT, msg_in.x==", msg_in.x)
                msg_out.angular.z = (-1.) * self.max_ang #- self.max_ang * (0.5 - msg_in.x + self.ang_thresh)
                self.last_spotted_x = 2
                self.x_sign = 0
            # If the person is at an acceptable angle and too far or near, approach or retreat appropriately
            elif abs(msg_in.depth - self.opt_dist) > self.dist_thresh :
                print("FORWARD, msg_in.depth==", msg_in.depth)
                # Set magnitude of velocity from half max speed (at threshold) to max speed (double threshold)
                msg_out.linear.x = self.max_vel * (.5 + min(.5, (abs(msg_in.depth - self.opt_dist) - self.dist_thresh)/(2 * self.dist_thresh)))
                # Set direction of velocity (forward if too far, backward if too near)
                self.x_sign = (msg_in.depth - self.opt_dist)/abs(msg_in.depth - self.opt_dist)
                msg_out.linear.x *= self.x_sign
            # If the robot was already approaching or retreating, continue to do so until at the optimal distance.
            # This should reduce 'jitteriness' when someone is near the threshold. We *may* want to do the same thing
            # for angle to prevent the robot from rapidly switching between turning and linear motion if someone is near
            # the angle threshold and beyond the depth threshold.
            elif self.x_sign != 0 and (msg_in.depth - self.opt_dist)/abs(msg_in.depth - self.opt_dist) == self.x_sign:
                msg_out.linear.x = self.x_sign * 0.5 * self.max_vel
            # If no movement needs to be taken, just log that for future reference.
            else:
                self.x_sign = 0
            
            # TODO: Have another case for depth == 0 where velocity is determined based on dimensons
            # CONSIDER: Train a logistic regression model on the distances between various landmarks and depth when
            #           we have non-zero depth, and consult the model to determine depth when depth is zero. Could
            #           accumulate data during operation and train while charging, so training is a separate programme
            #           that doesn't impact runtime latency.

        self.publisher.publish(msg_out)

def main():
    rclpy.init()
    patient_sub = PatientSubscriber()
    rclpy.spin(patient_sub.node)

if __name__ == '__main__':
    main()
