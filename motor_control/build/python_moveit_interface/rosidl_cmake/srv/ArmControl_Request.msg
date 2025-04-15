string task_name           # 指定命名姿態，例如 put_1、home 等
bool trigger_grab_flow     # 執行抓取流程
bool trigger_place_flow    # 執行放置流程
string target_zone         # 指定抓取或放置的區域（例如 grab_a、put_left）
geometry_msgs/Pose target_pose  # 可選：給定實際三維目標位置
string named_pose          # 可選：移動至指定 named pose
bool gripper_close         # 控制夾爪關閉/開啟

