# 验证记录

日期：2026-10-30。运行方式：Python 标准库 unittest 与命令行闭环仿真。

## 已执行

| 测试 | 判定依据 |
| --- | --- |
| 直线里程计 | 等轮速下位移满足 rωt，侧移与转角为零 |
| 圆弧积分 | 单轮静止时位姿满足解析圆弧解 |
| 轮速限幅 | 最大轮速不超限，左右轮比例保持 |
| PID 抗积分饱和 | 持续饱和不累积积分，目标归零后恢复 |
| 指令看门狗 | 超时后输出零，后续新指令不能自动解除锁存 |
| 安全异常 | 急停、近障、NaN 指令、未来时间戳分别停机 |
| 四航点任务 | 完成 4 个航点且误差不大于 0.05 m |
| 障碍注入 | 第 2 s 注入后状态 stopped，输出与轮速为零 |
| 无效配置与超时 | 非法参数拒绝执行，过短任务预算返回 timeout |

本地 9 项测试通过。CI 已配置 Python 3.10 与 3.12；远端是否通过以 GitHub Actions 实际结果为准。

## 复现命令

```bash
python -m unittest discover -s tests -v
python -m alpha_lab --config config/robot.json --mission config/mission.json --output outputs/baseline
python -m alpha_lab --obstacle-at 2 --output outputs/obstacle
```

仓库 `examples/` 保存首次运行的 CSV、JSON 与 SVG，时间步长与参数均可核查。程序没有随机项，相同参数下结果应可复现。

## 验证边界

未验证实机控制、电机温升、传感器噪声、轮胎打滑、坡道、定位漂移、通信抖动和动态障碍。当前安全门仅作控制逻辑演示，没有安全等级认证；理想制动不能用于推算实体机器人停止距离。路线跟踪不包含地图、SLAM 或障碍绕行，终点朝向不受约束。
