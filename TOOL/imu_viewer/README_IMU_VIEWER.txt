SYUBEA AF1000X IMU Viewer (PC)

1) Install
- python -m pip install -r requirements.txt

2) Run
- python imu_viewer.pyw

3) Usage
- Select COM port
- Click Connect (sends 1510 to start streaming)
- Click Disconnect (sends 1511 to stop)

Output format expected:
IMU <roll> <pitch> <yaw> <vbat>

4) 3D Attitude View
- 오른쪽 3D 축에 자세(roll/pitch/yaw)가 표시됩니다.
- 좌표축: X(빨강), Y(초록), Z(파랑)
- 큐브 모델이 함께 회전하여 자세를 표시합니다.
- 오버레이에 롤/피치/요 및 배터리 전압이 표시됩니다.

