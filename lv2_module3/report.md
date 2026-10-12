## 실험 1 — 채도(S) 하한값에 따른 Mask와 검출 결과 변화

### 1. 실험 목적

HSV 채도(S) 하한값을 120에서 60으로 낮췄을 때 Mask와 검출 결과가 어떻게 달라지는지 비교하고, 관찰 결과를 바탕으로 설정값 선택의 근거를 설명한다.

### 2. 실험 조건

| 설정 항목 | A — 기준 조건 | B — 비교 조건 |
|---|---|---|
| hsv_lower | [93, 120, 35] | [93, 60, 35] |
| hsv_upper | [130, 255, 255] | [130, 255, 255] |
| min_area_ratio | 0.002 | 0.002 |
| publish_debug_image | true | true |

- 사용 저장소: `SpartaPA/Lv2_EYE4_Assignment`
- 사용 브랜치: `dev/sanghwa`
- 소스 커밋: 별도로 기록한 `source_commit.txt` 참조
- 실행 노드: `/perception_node`
- 입력 영상: RealSense D435의 실시간 컬러 영상
- 카메라 설정: 640 × 480, 30 FPS
- 변경 변수: `hsv_lower`의 S 하한값
- 통제 계획: 나머지 처리 설정, 카메라 위치, 대상 위치, 배경 및 조명 조건을 동일하게 유지한다.
- 노출 및 화이트밸런스의 자동/고정 여부: 아직 확인되지 않음
- 실험 한계: 실시간 카메라를 사용하므로 A와 B의 입력 프레임이 완전히 같지는 않다.
- 실행 범위: 카메라와 인지 노드만 사용하며 모터 제어는 실행하지 않는다.

### 3. 수신 토픽

| 구분 | 토픽 | 메시지 타입 |
|---|---|---|
| Raw Image | `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| Mask | `/perception/mask` | `sensor_msgs/msg/Image` |
| 검출 결과 영상 | `/perception/debug_image` | `sensor_msgs/msg/Image` |
| 수치 검출 결과 | `/target` | `geometry_msgs/msg/PointStamped` |

### 4. 실험 전 예측 — 직접 작성

#### 4.1 Mask의 변화

- 대상 영역의 예상 변화: 파란색 물체(대상 영역) 옆에 하늘색 물체를 두었으므로 대상 영역은 확장될 것
- 배경 영역의 예상 변화: 배경 영역(검정)은 증가할 것
- 위와 같이 예상한 이유: s 값을 낮추는 것은 채도 범위를 넓히므로'더 연한 색'을 감지하게 됨

#### 4.2 검출 결과의 변화

- 대상 검출 여부에 대한 예측: 일정 크기 이상의 면적이 검출될 것이므로 여전히 검출될 것임
- 검출 위치 및 선택되는 대상의 변화에 대한 예측: 하늘색 물체가 파란색 물체보다 더 큰 면적을 차지하므로 검출되는 물체가 하늘색 물체가 될 것임
- 검출 안정성에 대한 예측: 충분한 면적이 검출될 것이므로 안정적일 것임
- 위와 같이 예상한 이유: s 값을 낮추는 것은 검출 영역을 넓히는 것이므로 검출되는 색의 범위를 증가시기 때 

### 5. 비교 및 설정값 선택 기준

| 비교 항목 | 관찰할 내용 | 내가 설정한 판단 기준 |
|---|---|---|
| 대상의 Mask 포함 정도 | 대상 중 Mask에 포함되는 영역과 누락되는 영역 | 파란색 물체는 다 포함 |
| 배경의 불필요한 포함 정도 | 배경 중 Mask에 포함되는 영역 | | 하늘색 물체가 면적의 절반 이상 포함 |
| 검출 대상의 정확성 | 검출 결과 영상이 의도한 대상을 선택하는지 | | 의도한 대상 미선택 |
| 검출의 안정성 | 관찰 중 검출이 끊기거나 다른 대상으로 바뀌는지 | | 검출된 대상 유지 |

### 6. 예측 작성 시점

- 작성일: 2026-10-12
- A 조건: 실시간 영상, Mask 및 검출 결과 영상의 정상 표시를 확인한 이후 작성
- B 조건: 이번 정식 비교의 실행 결과는 아직 기록되지 않음
- 사전 탐색: 기존 코드에서 S 하한값을 60으로 변경한 이력이 있음
- 이번 비교: Mask를 제공하는 동일한 구현으로 A와 B를 다시 실행하고 증거를 수집함

### 7. 실제 결과 — 실험 후 직접 작성

- 관찰 대상 및 배경 설명: s 값 하한 변경

| 비교 항목 | A — S 하한 120 | B — S 하한 60 |
|---|---|---|
| 대상의 Mask 포함 정도 | 포함 | 포함|
| 배경의 불필요한 포함 정도 | 다소 | 많이|
| 검출 대상의 정확성 | 정확| 부정확|
| 검출의 안정성 | 안정| 안정|

### 8. 해석 및 결론 — 직접 작성문

- 예측과 일치한 부분: 물체 대상의 변경
- 예측과 달랐던 부분: 없음
- 차이가 발생한 원인에 대한 해석: 더 넓은 범위 채도 감지
- 최종 선택한 설정값: s 하한 120
- 선택 근거: 의도한 대상만 선택
- 결과 해석의 한계: 다른 빛 환경에서도 잘 검출하는지 미검토

### 9. 증거 자료

A와 B의 자료는 각각 해당 실험 결과 폴더에 저장한다.

| 파일 | 내용 |
|---|---|
| `tracker.yaml` | 해당 조건에 사용한 설정 파일 |
| `applied_parameters.yaml` | 실행 중인 노드의 파라미터 |
| `node_info.txt` | 노드의 구독 및 발행 정보 |
| `topics.txt` | ROS 2 토픽 목록과 메시지 타입 |
| `detection.txt` | 10초 동안 수신한 `/target` 메시지 |
| `capture_start.txt` | 수치 결과 수집 시작 시각 |
| `capture_end.txt` | 수치 결과 수집 종료 시각 |
| `views.png` | Raw Image, Mask 및 검출 결과 영상의 화면 캡처 |
| `observation.md` | 해당 조건의 관찰 기록 |


#### 실험 1 A/B 증거 링크

| 자료 | A — S 하한 120 | B — S 하한 60 |
|---|---|---|
| views.png | [A: views.png](results/experiment1_mask_20261012_180339/A/views.png) | [B: views.png](results/experiment1_mask_20261012_180339/B/views.png) |
| tracker.yaml | [A: tracker.yaml](results/experiment1_mask_20261012_180339/A/tracker.yaml) | [B: tracker.yaml](results/experiment1_mask_20261012_180339/B/tracker.yaml) |
| applied_parameters.yaml | [A: applied_parameters.yaml](results/experiment1_mask_20261012_180339/A/applied_parameters.yaml) | [B: applied_parameters.yaml](results/experiment1_mask_20261012_180339/B/applied_parameters.yaml) |
| detection.txt | [A: detection.txt](results/experiment1_mask_20261012_180339/A/detection.txt) | [B: detection.txt](results/experiment1_mask_20261012_180339/B/detection.txt) |
