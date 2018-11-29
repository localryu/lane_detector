# 2018 lane detector using opencv only

## branches
* master: KASA competition lane detector (continuous lane)
* pams: PAMS competition lane detector (dotted lane)

## About main codes(classes)
* LinePointDetector: 차선 검출을 위해 차선 위의 점을 찾는 클래스
* LaneDetector: LinePointDetector를 이용하여 차선을 검출하는 클래스
* LaneDetectorNode: LaneDetector를 이용한 ROS 노드 클래스 
* main: LaneDetectorNode 클래스 객체를 생성하여 노드를 생성하는 메인 함수 
* *Dependency: etc/Class Diagram.png 참고*

## About launch files
* opencv_team_gtcam: prosilica gt 드라이버 노드 + lane_detector 노드 + serial_example 노드(rc car) or kuuve_control 노드(scale platform)
* opencv_team_webcam: usb_cam 드라이버 노드 + lane_detector 노드 + serial_example 노드(rc car) or kuuve_control 노드(scale platform)
* test_with_gtcam: prosilica gt 드라이버 노드 + lane_detector 노드
* test_with_webcam: usb_cam 드라이버 노드 + lane_detector 노드

## Debug mode printlog parsing
* debug mode로 실행시 print log들이 publish된다. 이때 `rostopic echo /lane_detector/printlog | sed 's/\\n/\n/g'`을 사용하면 로그를 실시간으로 볼수있다
