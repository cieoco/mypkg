#!/usr/bin/env python3
"""
在 ROS 2 + Docker 裡跑的 HTTP 伺服器 + 機械手臂控制節點

功能：
  1. 建立 ArmCommander 節點，透過 JointTrajectory 控制六軸手臂
  2. 啟動一個 FastAPI HTTP 伺服器，監聽 /command 這個路徑
  3. Windows 語音程式會送 POST /command，JSON 內容類似：{"cmd": "home"}
  4. 收到後依 cmd 呼叫對應的動作（move_home / move_pose1 / step_up / step_down）
"""

import threading

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# -------- FastAPI / Uvicorn 部分 --------
from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn


# =========================
# ROS 2 節點：ArmCommander
# =========================
class ArmCommander(Node):
    """
    ArmCommander：
      - 負責發 JointTrajectory 給 /joint_trajectory_controller/joint_trajectory
      - 提供幾個基本動作：home, pose1, step_up, step_down
    """
    def __init__(self):
        super().__init__('arm_commander_http')

        # 建立 Publisher
        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # 六個關節名稱（順序要跟控制器設定一致）
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
        ]

        # 紀錄目前姿勢（假設一開始全 0）
        self.current_positions = [0.0] * 6

    # ------------------------------------------------
    # 發送 JointTrajectory 的共用函式
    # ------------------------------------------------
    def send_trajectory(self, positions, duration_sec=2):
        """
        positions: 長度 6 的 list，代表六個關節目標角度（rad）
        duration_sec: 幾秒內走到這個姿勢
        """
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start.sec = duration_sec
        point.time_from_start.nanosec = 0

        traj.points.append(point)

        self.pub.publish(traj)
        self.get_logger().info(f'發送動作: {positions}')
        self.current_positions = positions.copy()

    # ------------------------------------------------
    # 幾個基本動作
    # ------------------------------------------------
    def move_home(self):
        """HOME 姿勢：全部關節設為 0"""
        self.get_logger().info("執行動作：HOME")
        self.send_trajectory([0.0] * 6, duration_sec=2)

    def move_pose1(self):
        """示範姿勢 Pose1"""
        self.get_logger().info("執行動作：POSE1")
        self.send_trajectory([0.0, -0.5, 0.5, 0.0, 0.0, 0.0], duration_sec=2)

    def step_up(self, delta=0.1):
        """微調往上：joint2--, joint3++"""
        self.get_logger().info("執行動作：STEP_UP")
        p = self.current_positions.copy()
        p[1] -= delta
        p[2] += delta
        self.send_trajectory(p, duration_sec=1)

    def step_down(self, delta=0.1):
        """微調往下：joint2++, joint3--"""
        self.get_logger().info("執行動作：STEP_DOWN")
        p = self.current_positions.copy()
        p[1] += delta
        p[2] -= delta
        self.send_trajectory(p, duration_sec=1)


# =========================
# FastAPI HTTP 伺服器部分
# =========================

app = FastAPI()

# 全域變數，讓 FastAPI 的 handler 可以拿到 ROS 節點
arm_node: ArmCommander | None = None  # 如果 Python 版本太舊，可以改成不標註型別


class Command(BaseModel):
    cmd: str


@app.post("/command")
def handle_command(command: Command):
    """
    接收來自 Windows 語音程式的指令：
      {"cmd": "home"} / {"cmd": "pose1"} / {"cmd": "up"} / {"cmd": "down"} / {"cmd": "quit"}
    """
    global arm_node
    if arm_node is None:
        return {"status": "error", "message": "ArmCommander 節點尚未就緒"}

    cmd = command.cmd
    arm_node.get_logger().info(f"收到 HTTP 指令: {cmd}")

    if cmd == "home":
        arm_node.move_home()
        return {"status": "ok", "action": "home"}

    if cmd == "pose1":
        arm_node.move_pose1()
        return {"status": "ok", "action": "pose1"}

    if cmd == "up":
        arm_node.step_up()
        return {"status": "ok", "action": "step_up"}

    if cmd == "down":
        arm_node.step_down()
        return {"status": "ok", "action": "step_down"}

    if cmd == "quit":
        # 不直接 shutdown，由你自己決定要不要讓這個指令關掉 ROS
        arm_node.get_logger().info("收到 quit 指令（目前只記錄，不關閉 ROS）")
        return {"status": "ok", "action": "quit_received"}

    return {"status": "error", "message": f"未知指令: {cmd}"}


def start_http_server_in_thread(host: str = "0.0.0.0", port: int = 8000):
    """
    用 threading 的方式啟動 Uvicorn，不會卡住 rclpy.spin()
    """
    def run():
        uvicorn.run(app, host=host, port=port, log_level="info")

    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t


# =========================
# main()
# =========================
def main():
    global arm_node

    rclpy.init()
    arm_node = ArmCommander()
    arm_node.get_logger().info("ArmCommander 節點啟動。")

    # 啟動 HTTP 伺服器
    start_http_server_in_thread(host="0.0.0.0", port=8000)
    arm_node.get_logger().info("HTTP 伺服器已啟動，監聽 /command。")

    # ROS 2 事件迴圈
    rclpy.spin(arm_node)

    arm_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()