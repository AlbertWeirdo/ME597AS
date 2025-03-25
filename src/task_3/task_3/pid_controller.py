import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt


class pid_speed_controller(Node):
    def __init__(self):
        super().__init__('pid_speed_controller')
        ## For hardware
        self.subscription=self.create_subscription(LaserScan, '/robot/scan', self.fetch_distance, 10)
        self.publishers_=self.create_publisher(Twist,'/robot/cmd_vel',10)
        ## For simulation
        # self.subscription=self.create_subscription(LaserScan, '/scan', self.fetch_distance, 10)
        # self.publishers_=self.create_publisher(Twist,'/cmd_vel',10)
        self.latest_msg=None
        self.target_distance=0.35
        self.dt=0.1
        # pid constants 
        # 0.6 0.005 0.3
        # 0.6 0.0015 0
        # 0.6 0.0015 0.15
        # 0.605 0.0024 0.187
        self.kp=0.605
        self.ki=0.00002
        self.kd=0.2


        # pid variables
        self.prevError=0.0
        self.integral=0.0

        self.create_timer(0.1, self.callback)

        self.error_history = []
        self.time_history = []
        self.start_time = self.get_clock().now().nanoseconds*1e-9
        # plot the error to time
        plt.ion()
        self.fig, self.ax=plt.subplots()
        self.ax.set_title("PID Error Over Time")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Error (m)")


    def fetch_distance(self,msg):
        self.latest_msg=msg

    def callback(self):
        if (self.latest_msg != None):
            self.get_logger().info(f'The robot is {self.latest_msg.ranges[0]} meters away from the wall')
            # self.get_logger.info(f'current angle: {self.latest_msg.angle_min}')
            self.PID_controller()
        return

    def PID_controller(self):
        error=self.latest_msg.ranges[0]-self.target_distance
        self.integral+=error*self.dt
        derivative=(error-self.prevError)/self.dt

        output=(self.kp*error)+(self.ki*self.integral)+(self.kd*derivative)
        # check if the output is within the limits
        print(f'output velocity : {output}')
        output=min(0.15, max(-0.15,output))
        print(f'output velocity : {output}')
        print(f'previous error {self.prevError}')
        
        # Store error and time for plotting
        current_time = self.get_clock().now().nanoseconds*1e-9 - self.start_time
        self.error_history.append(error)
        self.time_history.append(current_time)

        # Update the plot
        self.update_plot()

        
        self.prevError=error

        # publish output
        cmd=Twist()
        cmd.linear.x=output
        cmd.linear.y=0.0
        cmd.angular.z=0.0
        self.publishers_.publish(cmd)
    
    def update_plot(self):
        self.ax.clear()
        self.ax.plot(self.time_history, self.error_history, marker='o', linestyle='-')
        self.ax.set_title("PID Error Over Time")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Error (m)")
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()



def main():
    rclpy.init()
    pos_control=pid_speed_controller()

    rclpy.spin(pos_control)
    pos_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

        