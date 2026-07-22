import mujoco.viewer
import ikpy.chain
import transforms3d as tf
import numpy as np
from collections import deque
import matplotlib.pyplot as plt


def viewer_init(viewer):
    """渲染器的摄像头视角初始化"""
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.lookat[:] = [0, 0.5, 0.5]
    viewer.cam.distance = 2.5
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -30


class ForcePlotter:
    """实时可视化接触力"""

    def __init__(self, update_interval=20):
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.update_interval = update_interval  # 更新间隔帧数
        self.frame_count = 0  # 帧计数器

    def plot_force_vector(self, force_vector):
        self.frame_count += 1
        if self.frame_count % self.update_interval != 0:
            return  # 跳过本次渲染

        self.ax.clear()

        origin = np.array([0, 0, 0])
        force_magnitude = np.linalg.norm(force_vector)
        force_direction = force_vector / force_magnitude if force_magnitude > 1e-6 else np.zeros(3)

        # 主箭头
        arrow_tip = force_direction * 1.5
        self.ax.quiver(*origin, *arrow_tip, color='r', arrow_length_ratio=0)

        # 蓝色箭头
        self.ax.quiver(*arrow_tip, *(0.5 * force_direction), color='b', arrow_length_ratio=0.5)

        # XY平面投影
        self.ax.plot([0, arrow_tip[0]], [0, arrow_tip[1]], [-2, -2], 'g--')

        # XZ平面投影
        self.ax.plot([0, 0], [2, 2], [0, arrow_tip[2]], 'm--')

        # 力大小指示条
        scaled_force = min(max(force_magnitude / 50, 0), 2)
        self.ax.plot([-2, -2], [2, 2], [0, scaled_force], 'c-')
        self.ax.text(-2, 2, scaled_force, f'Force: {force_magnitude:.1f}', color='c')

        # 坐标系设置
        self.ax.scatter(0, 0, 0, color='k', s=10)
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([-2, 2])
        self.ax.set_title(f'Force Direction')

        plt.draw()
        plt.pause(0.001)
        self.frame_count = 0  # 重置计数器


class ForceSensor:
    def __init__(self, model, data, window_size=100):
        self.model = model
        self.data = data
        self.window_size = window_size
        self.force_history = deque(maxlen=window_size)

    def filter(self):
        """获取并滑动平均滤波力传感器数据(传感器坐标系下)"""
        # 获取MjData中的传感器数据
        force_local_raw = self.data.sensordata[:3].copy() * -1

        # 添加新数据到滑动窗口
        self.force_history.append(force_local_raw)

        # 计算滑动平均
        filtered_force = np.mean(self.force_history, axis=0)

        return filtered_force


class JointSpaceTrajectory:
    """关节空间坐标系下的线性插值轨迹"""

    def __init__(self, start_joints, end_joints, steps):
        self.start_joints = np.array(start_joints)
        self.end_joints = np.array(end_joints)
        self.steps = steps
        self.step = (self.end_joints - self.start_joints) / self.steps
        self.trajectory = self._generate_trajectory()
        self.waypoint = self.start_joints

    def _generate_trajectory(self):
        for i in range(self.steps + 1):
            yield self.start_joints + self.step * i
        # 确保最后精确到达目标关节值
        yield self.end_joints

    def get_next_waypoint(self, qpos):
        # 检查当前的关节值是否已经接近目标路径点。若是，则更新下一个目标路径点；若否，则保持当前目标路径点不变。
        if np.allclose(qpos, self.waypoint, atol=0.02):
            try:
                self.waypoint = next(self.trajectory)
                return self.waypoint
            except StopIteration:
                pass
        return self.waypoint


def main():
    model = mujoco.MjModel.from_xml_path('model/universal_robots_ur5e/scene.xml')
    data = mujoco.MjData(model)
    my_chain = ikpy.chain.Chain.from_urdf_file("model/ur5e.urdf",
                                               active_links_mask=[False, False] + [True] * 6 + [False])

    start_joints = np.array([-1.57, -1.34, 2.65, -1.3, 1.55, 0])  # 对应机械臂初始位姿[-0.13, 0.3, 0.1, 3.14, 0, 1.57]
    data.qpos[:6] = start_joints  # 确保渲染一开始机械臂便处于起始位置，而非MJCF中的默认位置

    # 定义两个目标点：A位置（接近红色立方体）、B位置（推动立方体）
    target_A_pos = [-0.13, 0.5, 0.1]   # A位置：靠近立方体但不接触
    target_B_pos = [-0.13, 0.65, 0.1]  # B位置：推动立方体
    ee_euler = [3.14, 0, 1.57]
    ref_pos = [0, 0, -1.57, -1.34, 2.65, -1.3, 1.55, 0, 0]
    ee_orientation = tf.euler.euler2mat(*ee_euler)

    # IK求解A位置的关节角
    joint_angles_A = my_chain.inverse_kinematics(target_A_pos, ee_orientation, "all", initial_position=ref_pos)
    target_A_joints = joint_angles_A[2:-1]

    # IK求解B位置的关节角
    joint_angles_B = my_chain.inverse_kinematics(target_B_pos, ee_orientation, "all", initial_position=ref_pos)
    target_B_joints = joint_angles_B[2:-1]

    # 创建两阶段轨迹：初始→A→B
    trajectory_to_A = JointSpaceTrajectory(start_joints, target_A_joints, steps=100)
    trajectory_to_B = None  # 将在到达A点后创建
    current_phase = "to_A"  # 当前阶段

    force_sensor = ForceSensor(model, data)
    force_plotter = ForcePlotter()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer_init(viewer)

        # 添加目标位置可视化标记
        # 在场景中添加两个半透明球体标记A和B位置
        print(f"目标A位置: {target_A_pos}")
        print(f"目标B位置: {target_B_pos}")
        print("阶段1: 移动到A位置（接近立方体）")

        while viewer.is_running():
            # 根据当前阶段选择轨迹
            if current_phase == "to_A":
                waypoint = trajectory_to_A.get_next_waypoint(data.qpos[:6])
                data.ctrl[:6] = waypoint

                # 检查是否到达A点
                if np.allclose(data.qpos[:6], target_A_joints, atol=0.02):
                    if trajectory_to_B is None:
                        print("已到达A位置！")
                        print("阶段2: 移动到B位置（推动立方体）")
                        # 创建从当前位置到B的轨迹
                        trajectory_to_B = JointSpaceTrajectory(data.qpos[:6], target_B_joints, steps=150)
                        current_phase = "to_B"

            elif current_phase == "to_B":
                waypoint = trajectory_to_B.get_next_waypoint(data.qpos[:6])
                data.ctrl[:6] = waypoint

                # 检查是否到达B点
                if np.allclose(data.qpos[:6], target_B_joints, atol=0.02):
                    if current_phase == "to_B":
                        print("已到达B位置！任务完成")
                        current_phase = "completed"

            filtered_force = force_sensor.filter()
            force_plotter.plot_force_vector(filtered_force)

            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()