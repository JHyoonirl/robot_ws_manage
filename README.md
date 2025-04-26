## 개발 환경 set up
---
1) ros2 jazzy 기준 코드 구축
2) ubuntu 24.04 LTS
3) cyclone dds 사용

## Bashrc 파일에 설정 추가해야 하는 부분
---
### bash 파일에 설정해야 하는 것

source /opt/ros/jazzy/setup.bash
source ~/robot_ws_manage/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

alias cb='cd ~/robot_ws_manage && colcon build --symlink-install && . ~/robot_ws_manage/install/local_setup.bash'
alias cs='cd ~/robot_ws_manage/src'
alias rt='ros2 topic list'
alias testpub='ros2 run demo_nodes_py talker'
alias testsub='ros2 run demo_nodes_py listener'

2)
pkg update를 위한 설정
apt install setuptools==58.2.0
