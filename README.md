1) {your_ws}/src에서 git clone https://github.com/hjonghyeok/stop_line.git
2) cd ~/{your_ws} && catkin_make
3) cd src/stopline/scripts && sudo chmod +x *
4) {your_ws}/src/stopline/scripts에 stopline_node.py 수정
   - line 11 "/home/jh/catkin_ws/src/stopline/scripts/driving.mp4" => "/home/{name}/{your_ws}/src/stopline/scripts/driving.mp4"
   - 카메라로 하고 싶으면 "/home/jh/catkin_ws/src/stopline/scripts/driving.mp4" => 0

5) rosrun stopline stopline_node

6) roi 설정은 16번째 줄 X,Y,W,H 수정 
   - X, Y => 시작 좌표
   - W, H => 가로, 세로 길이
