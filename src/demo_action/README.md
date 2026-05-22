# 1. 退出 conda，必须用系统 Python

```bash
conda deactivate
source install/setup.bash
```

# 编译

```bash
colcon build --packages-select demo_action --symlink-install

```

## 测试服务器和客户端

### 启动服务器

ros2 run demo_action count_server

### 验证服务器：启动后应该会看到服务器等待请求的消息

ros2 action list
应该看到：/count_to_n

### 启动客户端

ros2 run demo_action count_client

## 查看action

ros2 action info /count_to_n

## 实时查看Feedback

ros2 action echo /count_to_n feedback

## 手动发送Goal

ros2 action send_goal /count_to_n demo_action/action/CountToN "{target_number: 5}"

## 先发送 Goal，加上 --feedback 参数实时看进度

ros2 action send_goal /count_to_n demo_action/action/CountToN "{target_number: 10}" --feedback

## 中途取消

ros2 action cancel /count_to_n

# 或者终端1 启动 Lifecycle Server

ros2 run demo_action count_server_lifecycle
