#!/usr/bin/env python3
"""
在 ROS 2 + Docker 裡跑的 HTTP 伺服器 + 機械手臂控制節點

功能：
  1. 建立 ArmCommander 節點，透過 Action (FollowJointTrajectory) 控制六軸手臂
  2. 啟動一個 FastAPI HTTP 伺服器，監聽 /command 這個路徑
  3. Windows 語音程式會送 POST /command，JSON 內容類似：{"cmd": "home"}
  4. 收到後依 cmd 呼叫對應的動作（move_home / move_pose1 / step_up / step_down）
"""
import threading
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn

# =========================
# ROS 2 節點：ArmCommander (使用 Action Client)
# =========================
class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander_http_action')

        # 建立 Action Client，連接到 follow_joint_trajectory 這個 Action
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        # 六個關節名稱
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6',
        ]

        # 紀錄目前姿勢（假設一開始全 0）
        self.current_positions = [0.0] * 6
        self.get_logger().info("等待 Action 伺服器 '/joint_trajectory_controller/follow_joint_trajectory'...")
        self.action_client.wait_for_server()
        self.get_logger().info("Action 伺服器已連接。")

    # ------------------------------------------------
    # 發送 Action Goal 的共用函式
    # ------------------------------------------------
    def send_goal(self, positions, duration_sec=2):
        if not self.action_client.server_is_ready():
            self.get_logger().error("Action 伺服器未就緒，無法發送指令。")
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"發送 Action Goal: {positions}")
        
        # 異步發送 Goal，並註冊回呼函式來處理結果
        # 這不會阻塞 HTTP 伺服器
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        # 更新當前位置紀錄 (樂觀更新)
        self.current_positions = positions.copy()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目標被伺服器拒絕')
            return

        self.get_logger().info('目標已被接受，等待執行結果...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('動作成功完成！')
        else:
            self.get_logger().error(f'動作失敗，錯誤碼: {result.error_code}, {result.error_string}')

    # ------------------------------------------------
    # 幾個基本動作 (現在呼叫 send_goal)
    # ------------------------------------------------
    def move_home(self):
        self.get_logger().info("執行動作：HOME")
        self.send_goal([0.0] * 6, duration_sec=3)

    def move_pose1(self):
        self.get_logger().info("執行動作：POSE1")
        self.send_goal([0.0, -0.5, 0.5, 0.0, 0.0, 0.0], duration_sec=2)

    def step_up(self, delta=0.1):
        self.get_logger().info("執行動作：STEP_UP")
        p = self.current_positions.copy()
        p[1] -= delta
        p[2] += delta
        self.send_goal(p, duration_sec=1)

    def step_down(self, delta=0.1):
        self.get_logger().info("執行動作：STEP_DOWN")
        p = self.current_positions.copy()
        p[1] += delta
        p[2] -= delta
        self.send_goal(p, duration_sec=1)

# =========================
# FastAPI HTTP 伺服器部分 (與之前相同)
# =========================
app = FastAPI()
arm_node: ArmCommander | None = None

class Command(BaseModel):
    cmd: str

@app.post("/command")
def handle_command(command: Command):
    global arm_node
    if arm_node is None:
        return {"status": "error", "message": "ArmCommander 節點尚未就緒"}

    cmd = command.cmd
    arm_node.get_logger().info(f"收到 HTTP 指令: {cmd}")

    if cmd == "home":
        arm_node.move_home()
        return {"status": "ok", "action": "home"}
    elif cmd == "pose1":
        arm_node.move_pose1()
        return {"status": "ok", "action": "pose1"}
    elif cmd == "up":
        arm_node.step_up()
        return {"status": "ok", "action": "step_up"}
    elif cmd == "down":
        arm_node.step_down()
        return {"status": "ok", "action": "step_down"}
    elif cmd == "quit":
        arm_node.get_logger().info("收到 quit 指令（目前只記錄，不關閉 ROS）")
        return {"status": "ok", "action": "quit_received"}
    else:
        return {"status": "error", "message": f"未知指令: {cmd}"}

def start_http_server_in_thread(host: str = "0.0.0.0", port: int = 8000):
    def run():
        uvicorn.run(app, host=host, port=port, log_level="info")
    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t

# =========================
# main() (與之前相同)
# =========================
def main():
    global arm_node
    rclpy.init()
    try:
        arm_node = ArmCommander()
        arm_node.get_logger().info("ArmCommander (Action Client) 節點啟動。")
        start_http_server_in_thread(host="0.0.0.0", port=8000)
        arm_node.get_logger().info("HTTP 伺服器已啟動於 http://0.0.0.0:8000，監聽 /command。")
        rclpy.spin(arm_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if 'arm_node' in locals() and arm_node:
            arm_node.get_logger().error(f"發生未預期錯誤: {e}")
    finally:
        if 'arm_node' in locals() and arm_node:
            arm_node.destroy_node()
        rclpy.shutdown()
        print("ROS 節點與伺服器已關閉。")

if __name__ == "__main__":
    main()
