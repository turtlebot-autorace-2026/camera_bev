# camera_bev

광각 카메라의 원본 영상을 바닥 평면의 **BEV(Bird's-Eye View, 위에서 내려다본 영상)**로 변환하는 ROS 2 Python 패키지다. 차선 검출에 사용할 영상과 유효 마스크를 만들고, Foxglove에서 원본·왜곡 보정·BEV를 비교할 수 있도록 토픽을 제공한다.

현재 개발 환경에서의 정확한 패키지 경로:

```text
/home/ubuntu/turtlebot3_ws/src/camera_bev
```

팀원은 자신의 ROS 작업 공간의 `src/camera_bev`에 패키지를 배치하면 된다. 아래 빌드·미리보기 스크립트는 **`<workspace>/src/camera_bev` 배치**를 기준으로 작업 공간을 찾는다. 명령 예시는 패키지 루트에서 실행한다.

## 목적과 처리 범위

- 기존 카메라 내부 행렬 K와 왜곡 계수 D, 지면 변환을 합성해 raw 영상에서 한 번의 `cv2.remap`으로 BEV를 생성한다.
- 중간 보정 영상의 crop을 거치지 않고, 현재 렌즈 모델이 지원하는 영역에서 원본 픽셀을 사용한다.
- BEV·격자 표시·렌즈 보정 비교영상·유효 마스크·처리 상태를 발행한다.
- bag 재생으로 결과를 확인하고 체커보드 지면 보정을 재현하는 도구를 제공한다.

차선 검출·조향은 별도 `lane_following` 패키지의 역할이다. 이 패키지는 모터 명령을 생성하지 않는다. 미리보기 도구는 bag의 `/camera_raw_16_9`만 재생하고 `/cmd_vel`·`/odom`은 재발행하지 않는다.

## 실행 환경과 준비물

| 구분 | 환경 |
|---|---|
| 배포 목표 | Ubuntu 20.04 / ROS 2 Galactic / Python 3.8 / OpenCV 4.2 |
| 실행 검증 환경 | Ubuntu 22.04 / ROS 2 Humble / Python 3.10 / OpenCV 4.5.4 |

Python 3.8 문법과 대상 API 호환성을 검토했으며, **실물 Galactic 환경의 빌드·실행은 아직 검증하지 않았다.**

필요한 구성:

- ROS 2 및 `colcon`, `ament_python`.
- `rclpy`, `ament_index_python`, `rcl_interfaces`, `sensor_msgs`, `std_msgs`, `launch`, `launch_ros`.
- NumPy, OpenCV, PyYAML. ROS 패키지 의존성은 [package.xml](package.xml)을 기준으로 한다.
- bag 미리보기: `ros2 bag` CLI와 SQLite3 저장소 플러그인.
- Foxglove 미리보기: 별도로 설치한 `foxglove_bridge`와 Foxglove 클라이언트.
- 검사 실행: `pytest`.

ROS 배포판에 맞는 시스템 Python을 사용한다. `pip install`만으로 ROS 의존성이나 이 패키지의 share 파일 설치가 완료되는 것은 아니다.

입력은 `sensor_msgs/msg/CompressedImage` 타입의 **JPEG 1280×720** 영상이다. 기본 토픽은 `/camera_raw_16_9`이며, 다른 해상도는 자동 축소·확대하지 않고 거부한다. `sensor_msgs/msg/Image` 입력을 직접 받는 노드는 아니다.

## 빠른 시작: bag을 Foxglove로 확인

아래 예시는 Humble 개발 환경이다. 로봇에서 실행할 때는 설치된 배포판에 맞게 `humble`을 `galactic`으로 바꾼다.

```bash
# 현재 개발 PC의 패키지 위치. 팀원은 자신의 checkout 위치로 이동한다.
cd /home/ubuntu/turtlebot3_ws/src/camera_bev

# camera_bev만 별도 빌드한다.
PYTHONNOUSERSITE=1 BEV_ROS_DISTRO=humble bash tools/build.sh

# 정지 체커보드 bag을 반복 재생한다. --bag은 실제 데이터 경로로 바꾼다.
BEV_ROS_DISTRO=humble bash tools/preview.sh \
  --bag /home/ubuntu/bags/stationary_camera_2 \
  --port 8766
```

예시의 `8766`은 기존 bridge와 포트가 겹치는 일을 줄이기 위해 선택했다. 도구의 기본 포트는 `8765`다. 원본 bag은 패키지에 포함하지 않으므로 팀원에게 별도 전달해야 한다. bag 경로는 `metadata.yaml`과 `.db3` 파일이 들어 있는 디렉터리다.

빌드 명령의 `PYTHONNOUSERSITE=1`은 해당 명령에서 사용자 홈의 Python 패키지 대신 시스템 패키지를 사용하게 한다. 현재 PC에서는 사용자 설치 `setuptools 84.2.1`이 colcon의 `develop --uninstall` 옵션을 거부했고, 시스템 `setuptools 59.6.0`으로 빌드가 성공했다. 이 설정은 시스템 설치나 다른 터미널의 환경을 변경하지 않는다. 위 명령은 시스템에 colcon과 ROS 의존성이 설치된 환경을 전제로 한다.

Foxglove에서 **Open connection → Foxglove WebSocket**을 선택하고 다음 주소로 연결한다.

- 같은 PC: `ws://localhost:8766`
- 다른 PC: `ws://<ROS를 실행하는 PC의 IP>:8766`

Image 패널에서 `/bev/debug/compressed`를 선택하면 격자와 함께 확인할 수 있다. 원본은 `/camera_raw_16_9`, 글씨 없는 BEV는 `/bev/image/compressed`다. [foxglove/bev.layout.json](foxglove/bev.layout.json)을 가져오거나, 클라이언트 형식에 따라 [bev.layout.v1.json](foxglove/bev.layout.v1.json)을 사용한다. 레이아웃 가져오기가 맞지 않으면 토픽을 직접 선택한다.

종료는 실행한 터미널의 **Ctrl+C**다. 이 도구가 시작한 BEV 노드·bag player·bridge를 함께 정리한다. `--once`를 추가하면 한 번 재생한 뒤 자동 종료한다.

### 미리보기 옵션

| 옵션 | 기본값 / 동작 |
|---|---|
| `--bag PATH` | 사용자 홈의 `bags/normal` |
| `--settings FILE` | 지정하지 않으면 설치된 패키지의 `config/estimated.yaml` |
| `--rate NUMBER` | `1.0`, 재생 배속 |
| `--once` | 지정하면 단발 재생, 생략하면 반복 재생 |
| `--domain NUMBER` | `84`, 이 도구에서는 1~232 허용 |
| `--port NUMBER` | `8765`, Foxglove WebSocket 포트 |
| `--no-bridge` | bridge를 시작하지 않고 BEV와 bag만 실행 |
| `--output PATH` | 로그·프로세스 정보를 기록할 **새 디렉터리** |

미리보기의 DDS 환경은 지정한 `ROS_DOMAIN_ID`와 `ROS_LOCALHOST_ONLY=1`이다. 외부 Foxglove는 WebSocket으로 접속한다. 기존 bridge를 `--no-bridge`로 재사용하려면 해당 bridge도 같은 domain 및 호환되는 DDS 설정에 있어야 한다. 포트가 사용 중이면 다른 포트를 선택한다.

배포판 선택 순서는 `BEV_ROS_DISTRO` → 현재 `ROS_DISTRO` → 설치된 Galactic → Humble이다. 서로 다른 배포판의 환경이 섞이지 않도록 새 터미널에서 배포판을 명시하는 것을 권장한다.

## 실제 카메라 또는 별도 재생기에 연결

BEV 노드만 실행할 때는 다음과 같이 한다. 아래 예시는 현재 개발 PC 경로이며 팀원의 workspace 위치에 맞게 바꾼다.

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/turtlebot3_ws/install_bev/local_setup.bash

ros2 launch camera_bev bev.launch.py \
  input_topic:=/camera_raw_16_9 \
  use_sim_time:=false
```

카메라 드라이버와 Foxglove bridge는 별도로 실행하고, 카메라·BEV·bridge의 ROS domain과 DDS 설정을 맞춘다. 이 launch는 카메라·bag·bridge를 자동 실행하지 않는다. `/clock`을 발행하는 별도 bag 재생기에 연결한다면 `use_sim_time:=true`를 사용한다.

launch에서 `calibration_file:=/절대경로/camera.yaml`, `settings_file:=/절대경로/ground.yaml`도 지정할 수 있다. 설정 파일 형식은 아래의 패키지 YAML과 같아야 한다.

## 입력·출력 토픽

| 토픽 | 타입 / 내용 |
|---|---|
| `/camera_raw_16_9` | 입력 `CompressedImage`, JPEG 1280×720 |
| `/bev/image` | 출력 `Image/bgr8`, BEV |
| `/bev/debug` | 출력 `Image/bgr8`, BEV 위 격자·축·유효 경계 |
| `/bev/valid_mask` | 출력 `Image/mono8`, 변환 가능한 영역 255 / 무효 영역 0 |
| `/bev/image_rect` | 출력 `Image/bgr8`, 렌즈 왜곡만 보정한 비교영상 |
| `/bev/rect_valid_mask` | 출력 `Image/mono8`, 비교영상의 유효 영역 |
| `/bev/camera_info_rect` | 출력 `CameraInfo`, 비교영상 전용 내부 행렬과 D=0 |
| `/bev/status` | 출력 `String` JSON, 적용 설정·처리 수·오류 수·처리 시간 |

색 영상 세 종류에는 `/compressed` 토픽도 있다. 구독 중인 영상만 발행하므로 Foxglove 또는 다른 구독기를 연결해야 실제 영상 메시지가 나온다. 입력 stamp를 보존한다. `CameraInfo`는 렌즈 보정 비교영상에 해당하며 BEV 자체에 붙이는 카메라 정보가 아니다.

기본 BEV는 **500×500, 2mm/px**, 지면 범위는 X 0~1m / Y −0.5~+0.5m다. X는 전방, Y는 왼쪽이며 영상 위쪽이 전방이다. 프레임은 `camera_ground_estimated`이고 로봇의 `base_link`까지의 TF는 제공하지 않는다.

## 보정 설정과 적용 방법

| 파일 | 역할 |
|---|---|
| [config/camera_1280x720.yaml](config/camera_1280x720.yaml) | 기존 camera_ros의 내부 행렬 K·왜곡 계수 D를 옮긴 파일 |
| [config/estimated.yaml](config/estimated.yaml) | **현재 기본 지면 변환과 출력 설정** |
| [config/ground_stationary_20261030.yaml](config/ground_stationary_20261030.yaml) | 2026-10-30 체커보드 보정 설정의 보관본 |
| [config/initial_20261028.yaml](config/initial_20261028.yaml) | 이전 높이120mm/하향31.6° 가정의 물리 자세 모델 |

2026-11-03 기준 기본 설정과 체커보드 보관본은 같다. `estimated.yaml`이라는 이름이 보정 전 설정이라는 뜻은 아니다. K/D가 YAML에 포함되어 있어 BEV 노드가 원본 camera_ros pickle을 실행 중 직접 읽을 필요는 없다.

현재 `ground_projection`은 지면XY에서 왜곡 전 정규화 카메라 광선으로의 변환 행렬이다. 실측20mm 칸으로 스케일을 맞췄고, 높이135mm·하향32.5°는 근사 원점을 잡는 참고값이다. **이 모드에서 `ground.*`는 고정 참고값**이므로 높이·각도만 수정하면 거부된다. 렌즈 보정값이 달라져도 행렬의 보정 지문과 일치하지 않으면 거부된다.

`output.*`는 BEV 범위·해상도, `rectification.*`는 렌즈 보정 비교영상의 캔버스 설정이다. 설정을 복사해 수정하고 `--settings`로 선택할 수 있다. 차선 노드까지 사용한다면 두 노드에 동일한 지면·출력 설정을 적용해야 한다.

이전 투영으로 비교하려면:

```bash
BEV_ROS_DISTRO=humble bash tools/preview.sh \
  --bag /home/ubuntu/bags/normal \
  --settings config/initial_20261028.yaml \
  --port 8766
```

기본 내부 보정은 특정 광각 카메라의 1280×720 설정에 해당한다. 다른 카메라·초점·해상도·장착 상태에 그대로 적용하지 않는다. 현재 모델의 가장자리 제한과 자세한 수식·검증은 [README_KO.md](README_KO.md)를 참고한다.

## 체커보드 보정 재현

`tools/calibrate_ground.py`는 제공된 `stationary_camera_2` 영상의 검출 영역과 차선 확인 영역에 맞춘 도구다. 다른 위치·종류의 보드를 자동 처리하는 일반 캘리브레이션 도구는 아니다.

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/turtlebot3_ws/install_bev/local_setup.bash

python3 tools/calibrate_ground.py \
  --bag /home/ubuntu/bags/stationary_camera_2 \
  --calibration config/camera_1280x720.yaml \
  --initial-settings config/initial_20261028.yaml \
  --columns 10 --rows 7 --square-m 0.020 \
  --height-m 0.135 --pitch-deg 32.5 \
  --output /home/ubuntu/bags/reviews/new_ground_fit
```

출력 경로에는 아직 존재하지 않는 새 폴더를 지정한다.

- `profile.yaml`: 생성한 지면 변환 설정.
- `calibration.json`: 격자 치수·오차·차선 폭 검증 수치.
- `correspondences.npz`: 검출한 대응점 데이터.
- `before.png` / `after.png` / `comparison.png`: 보정 전후 영상.
- `corners.png` / `lane_validation_edges.png`: 사용한 교점·차선 경계 확인 영상.

생성 결과를 확인한 다음 미리보기에 적용한다. 이 도구는 패키지 기본 설정을 자동으로 덮어쓰지 않는다.

```bash
BEV_ROS_DISTRO=humble bash tools/preview.sh \
  --bag /home/ubuntu/bags/stationary_camera_2 \
  --settings /home/ubuntu/bags/reviews/new_ground_fit/profile.yaml \
  --port 8766
```

보정에는 검흰 칸의 내부 교점과 실측 간격을 사용한다. 종이 테두리·여백이나 체커보드 전체 폭을 차로 폭으로 취급하지 않는다.

## 패키지 및 데이터 구조

```text
camera_bev/
├── README.md                     # 팀 공유용 시작 문서
├── README_KO.md                  # 보정 원리·제약·검증 상세
├── .gitignore
├── package.xml / setup.py / setup.cfg
├── resource/camera_bev           # ament 패키지 등록용 파일
├── camera_bev/
│   ├── __init__.py
│   ├── geometry.py              # 렌즈 모델·지면 변환·remap·마스크
│   └── node.py                  # ROS 구독·발행·파라미터
├── config/                      # 내부 보정과 현재/과거 지면 설정
├── launch/bev.launch.py          # BEV 노드만 실행
├── foxglove/                    # 패널 배치 JSON
├── tools/
│   ├── build.sh                 # 분리 빌드
│   ├── preview.sh / preview.py  # bag + BEV + bridge 실행 관리
│   ├── calibrate_ground.py      # 정지 체커보드 보정 재현
│   ├── evaluate_bag.py          # 오프라인 전체 영상 처리·결과 저장
│   ├── estimate_initial_pitch.py # 이전 초기 직선 기반 각도 조사
│   ├── verify_ros.py            # 실제 ROS 출력 검증
│   └── verify_bridge.py         # Foxglove WebSocket 수신 검증
└── test/                        # 기하·ROS·보정 행렬 검사
```

빌드 산출물은 workspace의 `build_bev/`, `install_bev/`, `log_bev/`에 생성된다. 코드·설정은 `src/camera_bev`에서 관리한다. 단독 BEV 사용에는 별도 `lane_following` 패키지가 필요하지 않다.

현재 개발 PC에서는 데이터와 결과를 소스 밖에 보관한다.

| 경로 | 내용 |
|---|---|
| `/home/ubuntu/bags/archives` | 전달받은 원본 ZIP |
| `/home/ubuntu/bags/normal` | 정상 방향 주행 bag |
| `/home/ubuntu/bags/stationary_camera_2` | 정지 체커보드 bag |
| `/home/ubuntu/bags/catalog.json` | 데이터 출처·파일 해시·측정 정보 |
| `/home/ubuntu/bags/reviews/ground_calibration_20261030/final_fit` | 최신 지면 보정 산출물 |
| `/home/ubuntu/bags/reviews/normal_bev_20261028` | 초기 BEV 평가 결과·보고서·영상 |

이 경로들은 개발 PC의 위치이며 팀원의 PC에 자동 생성되거나 데이터가 내려받아지는 것은 아니다. **패키지 공유에는 소스·설정·문서를 포함하고, 재현용 bag과 필요한 결과 파일은 별도로 전달한다.** 패키지에 대용량 데이터를 복사하지 않고 `--bag`·`--output` 경로를 사용한다. `.gitignore`는 캐시·빌드·로그·원본 bag·생성 동영상을 제외하며 보정 YAML·코드·문서·Foxglove JSON은 포함하도록 구성했다.

## 검증과 현재 한계

패키지 검사:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/turtlebot3_ws/install_bev/local_setup.bash
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q test
```

오프라인 주행 bag 평가:

```bash
python3 tools/evaluate_bag.py \
  --bag /home/ubuntu/bags/normal \
  --output /home/ubuntu/bags/reviews/new_bev_evaluation
```

평가 결과는 원본 bag과 다른 새 폴더에 저장한다. 오프라인 평가 도구는 ROS 노드를 생성하지 않는다. `verify_ros.py`와 `verify_bridge.py`는 실제 통신 확인용 별도 도구다.

2026-10-30 검증 기록은 BEV 검사27개와 차선 연동 검사41개, 정지 bag127장 전체 ROS 출력, Foxglove WebSocket 수신을 포함한다. 패키지 단독 검사 명령이68개 전체 검사를 실행하는 것은 아니다.

현재 확인된 한계:

- 보드 칸 길이 중앙값은 가로19.89mm·세로19.95mm다. 같은 정지 자세의 반복 관측 결과이며 전체 지면의 독립 정확도 보장은 아니다.
- 보드 밖 차선 안쪽 폭은 약237~243mm로 사용자 보고260mm와 차이가 남는다. 카메라 기준 원점과 로봇 기준 좌표도 추가 검증이 필요하다.
- 유효 마스크는 렌즈 모델과 영상 범위 안에서 변환 가능한 영역이다. 장애물·가림·차선 신뢰도를 나타내지 않는다. 바닥 위로 솟은 물체는 평면 투영에서 늘어난다.
- 체커보드 보정 전 평가 결과와 새 기본 설정의 평가 결과를 구분해야 한다. 실차 조향·주행 성능을 검증한 패키지는 아니다.

## 자주 확인할 문제

| 증상 | 확인할 항목 |
|---|---|
| `Package 'camera_bev' not found` | 빌드 후 현재 터미널에서 해당 workspace의 `install_bev/local_setup.bash`를 source했는지 확인 |
| 빌드 중 `option --uninstall not recognized` | 사용자 설치 setuptools와 colcon의 호환성 확인. 위 빠른 시작처럼 `PYTHONNOUSERSITE=1`을 빌드 명령에 적용해 시스템 패키지 사용 |
| Foxglove 연결 실패 | 실행 포트·PC IP·bridge 실행 여부 확인. 기존 포트가 사용 중이면 `--port` 변경 |
| 연결되지만 영상 없음 | 입력 토픽·CompressedImage 타입·ROS domain 확인. Image 패널에서 영상 토픽을 구독했는지 확인 |
| 입력 영상 거부 | JPEG 디코딩 가능 여부와1280×720 해상도 확인. `/bev/status`·노드 로그 확인 |
| 보정 지문/고정 참고값 오류 | K/D와 지면 프로파일이 같은 보정 쌍인지 확인. `ground.*`만 변경한 설정은 사용하지 않음 |
| output 폴더가 이미 존재한다는 오류 | 미리보기·보정 결과 저장에 사용하지 않은 새 디렉터리 지정 |

상세 보정 근거와 이전 설정의 동작은 [README_KO.md](README_KO.md)에 정리되어 있다.
