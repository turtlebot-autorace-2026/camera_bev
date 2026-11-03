# camera_bev — 원본 광각 영상에서 직접 BEV

팀 공유용 목적·설치 전제·빠른 시작·파일 구조는 [README.md](README.md)를 먼저 읽는다.
이 문서는 보정 원리, 시야 제한과 검증 근거를 상세히 설명한다.

`/camera_raw_16_9`의 JPEG 1280×720에 기존 camera_ros 내부 보정값과 바닥 평면을 적용한다.
**2026-10-30 기본 설정은 바닥 체커보드의 실제20mm 칸으로 보정한 지면 변환이다.**
사용자는 종이가 평평하며 칸을 직접 측정했다고 확인했다. 검흰 칸의 내부 교점10×7개를
사용했으며 종이 여백이나 테두리를 차선 경계로 간주하지 않는다. 체커보드가 차선을 꽉
채운다는 가정도 없다. 높이135mm·하향32.5°는 좌표 원점을 잡는 참고값이며 정밀 자세 추정값이 아니다.
카메라 원점·로봇 좌표와 보드 밖의 거리 정확도는 아직 검증되지 않았다.

## 빌드와 바로 보기

소스 위치: `~/turtlebot3_ws/src/camera_bev`. 기존 camera_ros나 다른 패키지를 빌드하지 않고
`build_bev/`, `install_bev/`, `log_bev/`를 별도로 사용한다.

```bash
PYTHONNOUSERSITE=1 bash ~/turtlebot3_ws/src/camera_bev/tools/build.sh
bash ~/turtlebot3_ws/src/camera_bev/tools/preview.sh --bag ~/bags/normal
```

배포판은 `BEV_ROS_DISTRO`, 현재 `ROS_DISTRO`, 설치된 Galactic, Humble 순으로 선택한다.
명시하려면 앞에 `BEV_ROS_DISTRO=galactic` 또는 `BEV_ROS_DISTRO=humble`을 붙인다.
빌드의 `PYTHONNOUSERSITE=1`은 사용자 설치 setuptools와 colcon의 충돌을 피하기 위한
명령 단위 설정이다. 확인한 버전과 전제는 [README.md](README.md)의 빠른 시작을 참고한다.
필수 시스템 의존성은 ROS의 rclpy/sensor_msgs/launch_ros와
`python3-opencv python3-numpy python3-yaml`이다. 브리지는 별도 `foxglove_bridge` 패키지다.
`preview.sh --no-bridge`는 BEV와 bag만 실행한다. 이 명령은 **소스 디렉터리용**이다.

Foxglove에서 **Open connection → Foxglove WebSocket → `ws://이_컴퓨터_IP:8765`**로 연결한다.
같은 컴퓨터에서는 `ws://localhost:8765`다. Image 패널에 아래 토픽을 추가한다.
네트워크 대역폭을 줄이려면 색 영상의 `/compressed` 토픽을 선택한다.
`foxglove/bev.layout.json`은 6개 영상 패널의 가져오기용 배치다. Foxglove의
Layouts → Import from file로 연다. 새 공개 스키마 형식은 `bev.layout.v1.json`도 제공한다.
클라이언트 버전에 따라 가져오기 형식이 다르면 아래 표의 토픽을 Image 패널에 직접 선택한다.
레이아웃은 공식 Image 설정 필드와 대조했으며 사용자의 Foxglove GUI에서 가져오기는 미검증이다.

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/camera_raw_16_9` | CompressedImage | 촬영 원본 |
| `/bev/image` | Image/bgr8 | 직접 변환한 BEV, 검출용 |
| `/bev/debug` | Image/bgr8 | 체커보드 스케일 격자·근사 축·변환 경계 |
| `/bev/valid_mask` | Image/mono8 | 흰색=기하적으로 변환 가능, 검정=무효 |
| `/bev/image_rect` | Image/bgr8 | 넓은 시야의 렌즈 보정 비교영상 |
| `/bev/rect_valid_mask` | Image/mono8 | 위 보정영상의 유효 영역 |
| `/bev/camera_info_rect` | CameraInfo | 보정영상 전용 K, D=0 |
| `/bev/status` | String/JSON | 설정, 처리 개수·시간, 오류 개수 |

색 영상 3개는 `/bev/image/compressed`, `/bev/debug/compressed`,
`/bev/image_rect/compressed`도 제공한다. 토픽에 구독자가 있을 때 해당 영상을 생성·발행한다.
일반 BEV 영상에는 안내 글씨가 없고 debug에만 `BOARD SCALE / APPROX ORIGIN`을 표시한다.
BEV에 원본 CameraInfo를 붙이지 않는다.

미리보기는 ROS domain 84, localhost DDS를 사용한다. 외부 Foxglove에는 WebSocket만 연다.
bag에서 **`/camera_raw_16_9`만** 반복 재생하고 `/clock`을 생성한다. `/cmd_vel`과 `/odom`은
재발행하지 않는다. 브리지는 해당 영상과 `/bev/*`만 노출하고 clientPublish/services 기능을
활성화하지 않는다. `Ctrl-C`로 이 실행에서 시작한 프로세스만 종료한다.
`--once`, `--rate 0.5`, `--domain 85`, `--port 8766`도 지원한다.
로그와 소유 PID는 `~/bags/reviews/bev_preview_<시각>/processes.json`에 남는다.

## 변환과 좌표

BEV 픽셀 중심 → 바닥의 XY → 보정된 지면 호모그래피 → 정규화 카메라 광선 → 렌즈 왜곡을 포함한 raw 좌표 →
`cv2.remap` 한 번으로 색을 읽는다. 중간 alpha=0 보정·crop·640×480 축소를 거치지 않는다.
넓은 `image_rect`는 독립된 진단 분기다. 원본의 해상도가 다르면 오류로 거부한다.

바닥 좌표는 X 전방, Y 왼쪽, Z 위쪽이고 화면은 위가 전방, 왼쪽이 +Y다.
기본 원점은 카메라 수직 아래의 바닥을 근사한 것이며 로봇 중심이 아니다.
내부 주점 광선의 지면 위치를 참고 높이/하향각의 h/tan(pitch)에 맞춰 고정했다.
`camera_ground_estimated` 프레임을 사용하고 실측되지 않은 base_link TF를 만들지 않는다.
입력 stamp를 그대로 보존한다. 원본 frame_id가 비어 있으면 진단 영상에
`camera_optical_frame`을 사용한다.
입력 QoS는 best-effort/depth 2, 출력은 reliable/depth 5다. 출력 큐 크기는 읽기 전용 시작
파라미터 `output_qos_depth`로 조정한다. 느린 구독자에 대해 무한히 누적하지 않는다.

기본 범위는 체커보드로 맞춘 스케일로 X 0~1.0 m, Y −0.5~+0.5 m, 0.002 m/px의 500×500 영상이다.
현재 프로파일의 높이·각도 필드는 고정 참고값이다. 출력 범위/해상도를 바꿔도 보정된 지면 변환은 유지한다.
검은 영역은 채우지 않는다. 마스크는 **기하적 지원 영역**일 뿐 장애물·가림·노면 판별이나
차선 검출 신뢰도가 아니다. 사람·신발처럼 바닥 위로 솟은 물체는 평면 투영에서 늘어진다.

## 보정값과 가장자리 제한

`config/camera_1280x720.yaml`은 팀 camera_ros의 기존 5계수 plumb_bob 보정값을 그대로 옮겼다.
fisheye 모델이 아니다. 원본 pickle SHA256:
`c22cd925234e39a8fa587a9d9076f73fb4edcfb2b8e8110b406568a3f75a1da8`.
43장/RMS 1.617815 px는 저장된 값이며 이번에 재계산한 결과가 아니다.

이 다항식은 매우 큰 광선 각도에서 접혀 영상이 중복될 수 있다. 방사 왜곡만의 한계는
정규화 반경 약 1.26855지만 접선 왜곡도 고려해야 한다. 구현은 왜곡 Jacobian의 양의
고유값을 보장하는 중심 원판 안에서 기본 반경 1.15까지만 사용한다. 잘못된 점이 화면 안으로
다시 들어오는 것을 차단한다. 이 보수적인 영역은 일부 쓸 수 있는 가장자리도 제외할 수 있다.
raw 맨 아래 모든 픽셀의 복원까지 보장하지 않는다. 현재 모델로 표현되지 않는 주변부는
새로운 가장자리 보정 검증 없이 억지로 채우지 않는다.

이 보정값에 `getOptimalNewCameraMatrix(alpha=1)`을 적용하면 fx가 약 49.5 px로 매우 작아져
실용적인 보정 시야를 만들기 어렵다. 진단 영상은 focal=540 px의 별도 캔버스와 마스크를 쓴다.
이는 물리 카메라의 보정값 변경이 아니라 **출력 가상 카메라 선택**이다.

수식/API 기준: [OpenCV 4.2 calibration/projectPoints](https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html),
[OpenCV 4.2 remap](https://docs.opencv.org/4.2.0/da/d54/group__imgproc__transform.html).

## 체커보드 보정과 설정 교체

`config/estimated.yaml`은 현재 기본값이며 `config/ground_stationary_20261030.yaml`과 같다.
`ground_projection.ground_to_normalized`가 지면XY→왜곡 전 정규화 광선의3×3행렬이다.
기존 K/D를 다시 적용해 raw를 한 번만 remap한다. 왜곡된 raw에 단일 호모그래피를 직접 적용하지 않는다.
내부 보정의 SHA 지문이 다르거나 `ground.*` 참고값만 바꾸면 시작을 거부한다.
이 모드의 `ground.*` ROS 파라미터는 읽기 전용이다. 바닥 변환을 바꾸려면 보정을 다시 수행한다.
`output.*`/`rectification.*`는 별도로 조절 가능하지만 차선 노드도 사용하면 같은 설정으로 재시작한다.

보정은 기존 K/D로 만든 임시 고해상도 BEV에서 교점을 검출하고 이를 **원본 raw 좌표로 되돌린 뒤**
정규화 광선과20mm 격자 사이 관계를 적합했다. 초기 높이·각도를 정답으로 강제하지 않는다.
자세만 적합한 solvePnP 결과는 높이 약173mm/하향36.1°·재투영RMS3.38px로 보고값과 달라
실제 장착값으로 채택하지 않았다. 현재 행렬은 경험적 지면 보정이며 새 내부 보정/정밀 외부 보정을 뜻하지 않는다.
방법 근거: [OpenCV4.2 평면 호모그래피](https://docs.opencv.org/4.2.0/d9/dab/tutorial_homography.html).

정지영상17장을9장 적합/8장 검증으로 나눴다. 검증영상에서 한 칸 중앙값은
가로19.89mm/세로19.95mm(이전17.05/21.30mm)다. 일부 교점을 제외해 검증한 오차p95는1.43mm다.
**같은 정지 자세의 반복 관측 일관성**이며 전체 지면의 실제 정확도가1.43mm라는 뜻은 아니다.
보드 위/아래의 노출된 차선에서 독립 확인한 안쪽 폭은 가까운 쪽237mm/먼 쪽243mm로,
사용자 보고260mm와17~23mm 차이가 남는다. 종이 여백 구간을 제외했으며 차선 폭은 적합에 사용하지 않았다.
따라서 보드를260mm로 늘리거나 체커보드 전체 폭=차로 폭으로 강제하지 않는다.

재현(이 도구의 검출영역/차선검증 ROI는 이 정지영상용):

```bash
source /opt/ros/humble/setup.bash  # 실물은 galactic
source ~/turtlebot3_ws/install_bev/local_setup.bash
python3 ~/turtlebot3_ws/src/camera_bev/tools/calibrate_ground.py \
  --bag ~/bags/stationary_camera_2 \
  --calibration ~/turtlebot3_ws/src/camera_bev/config/camera_1280x720.yaml \
  --initial-settings ~/turtlebot3_ws/src/camera_bev/config/initial_20261028.yaml \
  --square-m 0.020 --height-m 0.135 --pitch-deg 32.5 \
  --output ~/bags/reviews/new_ground_fit
```

새 output 폴더에 profile.yaml·검증JSON·대응점·전후영상을 저장한다. 이 도구가 기본 설정을 자동 덮어쓰지는 않는다.
검토한 설정은 `preview.sh --bag ~/bags/stationary_camera_2 --settings /절대경로/profile.yaml`로 선택한다.
원래 높이120mm/하향31.6° 모델로 복원하려면 `--settings ~/turtlebot3_ws/src/camera_bev/config/initial_20261028.yaml`을 사용한다.
그 이전 모델에서는 height/pitch/roll/yaw 변경이 물리 투영을 다시 계산한다.

live 카메라에는 `ros2 launch camera_bev bev.launch.py use_sim_time:=false`로 연결한다.
카메라·제어기는 시작하지 않는다. 같은 렌즈·초점·1280×720 설정과 카메라 장착 상태를 유지해야 한다.
추후 다른 바닥 위치의 체크보드/실측 거리, 카메라-로봇 위치로 보드 밖 정확도와 원점을 검증한다.

## 검증 재현과 호환성

```bash
source /opt/ros/humble/setup.bash  # 실물에서는 galactic
source ~/turtlebot3_ws/install_bev/local_setup.bash
cd ~/turtlebot3_ws/src/camera_bev
python3 -m pytest -q
python3 tools/evaluate_bag.py --bag ~/bags/normal --output ~/bags/reviews/my_bev_check
```

오프라인 평가는 SQLite를 읽기 전용으로 열고 전체 JPEG 디코딩·BEV·진단 변환·성능·지원 영역을
검사하며 영상/CSV/JSON을 별도 폴더에 저장한다. ROS 노드나 속도 발행자를 생성하지 않는다.
원본 bag 해시를 전후 비교한다. 재생 영상은 평균 fps로 저장되므로 원본 시간 간격의 흔들림은
재현하지 않고 각 프레임에 bag 시각을 표시한다.

배포 목표는 Ubuntu 20.04 / ROS 2 Galactic / Python 3.8 / OpenCV 4.2다.
Python 3.8 문법과 해당 시기의 ROS/OpenCV API를 사용하며, 실제 실행 검증 환경은
Ubuntu 22.04 / Humble / Python 3.10 / OpenCV 4.5.4다. **실물 Galactic 런타임 검증은 별도다.**
정밀 거리 정확도, 실제 로봇 CPU의 처리율, 자동 차선 검출·조향은 이번 단계의 검증 대상이 아니다.

현재 개발 환경의 foxglove_bridge 3.6.0은 `foxglove.sdk.v1` WebSocket 프로토콜로 수신을
검증했다. 실물의 브리지/클라이언트 버전 조합은 따로 확인한다. BEV 노드 자체는 브리지에
의존하지 않으며 ROS 표준 메시지를 사용한다.

초기 모델의 과거 검증 근거: `~/bags/reviews/normal_bev_20261028/REPORT_KO.html`, `evaluation.json`,
`ros_check_depth5/result.json`, `bridge_check/result.json`, `compatibility.json`.
초기각 재추정은 `tools/estimate_initial_pitch.py --bag ~/bags/normal --output /별도/폴더`로
재현한다. 초기 직선의 수동 관심영역·색상 범위를 사용하는 조사 도구이며 다른 bag의 일반적인
자동 장착각 추정이나 차선 검출기로 사용하지 않는다. 특징점 overlay를 함께 확인한다.

2026-10-30 보정 근거: `~/bags/reviews/ground_calibration_20261030/`의
`final_fit/calibration.json`, `final_fit/comparison.png`, `tests.xml`, `ros_check/result.json`.
이전 normal bag의 차선 유효 비율은 이전 투영 결과이며 새 보정 후 성능으로 인용하지 않는다.
