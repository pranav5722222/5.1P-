import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped
from sensor_msgs.msg import Range

class SquareDriver:
    def __init__(self):
        # Set up class variables
        self.command = Twist2DStamped()
        self.tick_count = 0
        self.front_distance = 0

        # Initialize ROS node
        rospy.init_node('square_driver_node', anonymous=True)

        # Set up publishers and subscribers
        self.command_publisher = rospy.Publisher('/duckie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/duckie/fsm_node/mode', FSMState, self.state_callback, queue_size=1)
        rospy.Subscriber('/duckie/left_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)
        rospy.Subscriber('/duckie/front_center_tof_driver_node/range', Range, self.range_callback, queue_size=1)
        
    def state_callback(self, state_msg):
        rospy.loginfo("Current State: %s", state_msg.state)
        if state_msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.halt_robot()
        elif state_msg.state == "LANE_FOLLOWING":            
            rospy.sleep(1) # Wait a moment for the node to stabilize
            self.drive_square()

    def encoder_callback(self, encoder_msg):
        self.tick_count = encoder_msg.data

    def range_callback(self, range_msg):
        self.front_distance = range_msg.range

    # Halts the robot by sending zero velocities
    def halt_robot(self):
        self.command.header.stamp = rospy.Time.now()
        self.command.v = 0.0
        self.command.omega = 0.0
        self.command_publisher.publish(self.command)
 
    # Main execution loop to handle callbacks
    def execute(self):
        rospy.spin()

    # Drives the robot in a square path and stops
    def drive_square(self):
        for _ in range(4):
            self.drive_forward()
            self.turn_90_degrees()
        self.halt_robot()

    def drive_forward(self):
        self.command.header.stamp = rospy.Time.now()
        self.command.v = 1.0 # Forward speed
        self.command.omega = 0.0
        self.command_publisher.publish(self.command)
        rospy.loginfo("Moving forward!")
        rospy.sleep(1) # Duration for moving straight

    def turn_90_degrees(self):
        self.command.header.stamp = rospy.Time.now()
        self.command.v = 0.0 # No forward speed
        self.command.omega = 1.0
        self.command_publisher.publish(self.command)
        rospy.loginfo("Turning!")
        rospy.sleep(1) # Duration for turning
        self.halt_robot()

if __name__ == '__main__':
    try:
        square_driver = SquareDriver()
        square_driver.execute()
    except rospy.ROSInterruptException:
        pass
