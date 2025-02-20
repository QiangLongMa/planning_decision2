import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np

class SpeedVisualizer(Node):
    def __init__(self):
        super().__init__('speed_visualizer')

        # 订阅速度、误差和转角数据
        self.subscription = self.create_subscription(Float32, '/speed_topic', self.speed_callback, 10)
        self.subscription2 = self.create_subscription(Float32, '/error_topic', self.error_callback, 10)
        self.subscription3 = self.create_subscription(Float32MultiArray, '/steer_angle_topic', self.steer_angle_callback, 10)

        # 存储数据
        self.speeds = []
        self.errors = []
        self.steer_angles = []  # 原始转角数据
        self.filtered_steer_angles = []  # 滤波后的转角数据

        # 初始化图形
        self.fig, (self.ax_speed, self.ax_error, self.ax_steer) = plt.subplots(3, 1, figsize=(10, 8))
        
        # 绘制速度数据
        self.line_speed, = self.ax_speed.plot(self.speeds, color='g', label='Speed')
        self.ax_speed.set_ylim(0, 100)  # 根据实际速度范围调整
        self.ax_speed.set_title("Speed Visualization (km/h)")
        self.ax_speed.legend()

        # 绘制误差数据
        self.line_error, = self.ax_error.plot(self.errors, color='r', label='Error')
        self.ax_error.set_ylim(-2, 100)  # 根据实际误差范围调整
        self.ax_error.yaxis.set_major_locator(plt.MultipleLocator(20))
        self.ax_error.set_title("Error Visualization (cm)")
        self.ax_error.legend()

        # 绘制转角数据
        self.line_steer, = self.ax_steer.plot(self.steer_angles, color='g', label='Raw Steer Angle')
        self.line_filtered_steer, = self.ax_steer.plot(self.filtered_steer_angles, color='b', label='Filtered Steer Angle')
        self.ax_steer.set_ylim(-45, 45)  # 设置转角范围，假设是 -45 到 45 度
        self.ax_steer.set_title("Steering Angle Visualization (degrees)")
        self.ax_steer.legend()

        # 添加文本框以显示最新数据
        self.text_speed = self.ax_speed.text(0.05, 0.95, '', transform=self.ax_speed.transAxes, fontsize=10)
        self.text_error = self.ax_error.text(0.05, 0.95, '', transform=self.ax_error.transAxes, fontsize=10)
        self.text_steer = self.ax_steer.text(0.05, 0.95, '', transform=self.ax_steer.transAxes, fontsize=10)

        plt.ion()  # 开启交互模式
        plt.show()

        # 绑定关闭事件
        self.fig.canvas.mpl_connect('close_event', self.on_close)

    def speed_callback(self, msg):
        # 将速度从 m/s 转换为 km/h
        speed = msg.data * 3.6
        self.speeds.append(speed)
        self.line_speed.set_ydata(self.speeds)
        self.line_speed.set_xdata(range(len(self.speeds)))
        self.ax_speed.set_ylim(0, max(max(self.speeds), 20))  # 更新Y轴范围
        self.text_speed.set_text(f'Latest Speed: {speed:.2f} km/h')

        # 更新 X 轴范围，仅显示最新的 20 个数据点
       # self.ax_speed.set_xlim(max(0, len(self.speeds) - 20), len(self.speeds))
        
        self.ax_speed.relim()
        self.ax_speed.autoscale_view()
        plt.draw()
        plt.pause(0.01)

    def error_callback(self, msg):
        # 将误差从米转换为厘米
        error = msg.data * 100
        self.errors.append(error)
        self.line_error.set_ydata(self.errors)
        self.line_error.set_xdata(range(len(self.errors)))

        self.ax_error.set_ylim(min(min(self.errors, default=0), -200), max(max(self.errors, default=0), 200))
        self.text_error.set_text(f'Latest Error: {error:.2f} cm')

        # 更新 X 轴范围，仅显示最新的 20 个数据点
        #self.ax_error.set_xlim(max(0, len(self.errors) - 20), len(self.errors))

        self.ax_error.relim()
        self.ax_error.autoscale_view()
        plt.draw()
        plt.pause(0.01)

    def steer_angle_callback(self, msg):
        # 获取原始转角数据
        steer_angle = msg.data[0]  # 假设是一个包含单个角度的 Float32MultiArray
        self.steer_angles.append(steer_angle)
        filtered_steer = msg.data[1]
        self.filtered_steer_angles.append(filtered_steer)

        # 更新转角图形
        self.line_steer.set_ydata(self.steer_angles)
        self.line_steer.set_xdata(range(len(self.steer_angles)))
        self.line_filtered_steer.set_ydata(self.filtered_steer_angles)
        self.line_filtered_steer.set_xdata(range(len(self.filtered_steer_angles)))

        self.text_steer.set_text(f'Latest Steer Angle: {filtered_steer:.2f}°')

        # 更新 X 轴范围，仅显示最新的 20 个数据点
        #self.ax_steer.set_xlim(max(0, len(self.steer_angles) - 20), len(self.steer_angles))
        self.ax_steer.set_ylim(min(min(self.steer_angles) -50 , -500), max(max(self.steer_angles) + 50, 500))

        self.ax_steer.relim()
        self.ax_steer.autoscale_view()
        plt.draw()
        plt.pause(0.01)

    def on_close(self, event):
        # 在程序运行完之后放大转角数据图
        print("Program finished, zooming in on the last part of steer angle data...")
        
        # 显示放大后的转角图（最后的部分）
        self.ax_steer.set_xlim(max(0, len(self.steer_angles) - 100), len(self.steer_angles))  # 最后 100 个数据点
        self.ax_steer.set_ylim(min(self.steer_angles[-100:], default=-45), max(self.steer_angles[-100:], default=45))
        
        self.ax_steer.relim()
        self.ax_steer.autoscale_view()
        plt.draw()
        plt.pause(1)

        # 正常关闭 ROS2
        self.get_logger().info('关闭图形窗口')
        rclpy.shutdown()  # 正常关闭 ROS2
        plt.close(self.fig)  # 关闭 Matplotlib 图形

def main(args=None):
    rclpy.init(args=args)
    visualizer = SpeedVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()

if __name__ == '__main__':
    main()
