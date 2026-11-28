# 좌전륜 장착 단독 시뮬레이션 (`wheel_mount_sim/`)

컨베이어 높이(0.50 m) 작업대에 누워 있는 CN7 바퀴를 **Kawasaki RS080N**이 **SCHUNK PHL-W 63-080-S** 트레드 클램프로
집어 세우고, 앞 왼쪽 허브에 끼운 뒤 놓는 단독 시험이다. 전체 셀(`wheel_cell/`)·다른 단독 시험(`map_fasten/`)과
섞이지 않도록 이 폴더에서만 관리한다. 공용 파일은 읽기만 한다(`assets/vehicle/CN7_Wheel_205-55R16.usd`).

## 실행 (저장소 루트에서)

2026-11-28부터 셀 장착은 `mount_control.py`를 쓴다(`cell_mount_run.py`는 1단계 기록용으로 그대로 둔다).
**Isaac Sim은 한 번에 하나만 돌린다.** 동시에 돌리면 같은 시드라도 결과가 달라진다(TS-22).

```bash
~/isaacsim/python.sh wheel_mount_sim/mount_control.py --seed 42 --no-video --out wheel_mount_sim/output/cell/run1        # 앞바퀴 좌우
~/isaacsim/python.sh wheel_mount_sim/mount_control.py --seed 1 --car --car-dynamic --no-video --out .../car1           # A 앞바퀴 → 이동 → B 뒷바퀴
MOUNT_CAMS=zoom_left,zoom_right ~/isaacsim/python.sh wheel_mount_sim/mount_control.py --replay-video .../run1 --t0 17 --t1 31   # 확대 영상
./wheel_mount_sim/ros2_nodes/run_ros2_test.sh wheel_mount_sim/output/cell/ros2_1 --seed 1                              # ROS 2 (2단계)
```

```bash
./wheel_mount_sim/tools/fetch_assets.sh                          # 처음 한 번: RS080N USD 받기 (커밋하지 않음)
~/isaacsim/python.sh wheel_mount_sim/run.py --no-video           # 빠른 확인 (~5분)
~/isaacsim/python.sh wheel_mount_sim/run.py --fast --no-video    # 공정 시간 단축 프로필 (바퀴 1개 22.3 s)
~/isaacsim/python.sh wheel_mount_sim/run.py                      # 영상(GIF) 포함
~/isaacsim/python.sh wheel_mount_sim/run.py --gui --fast --no-video   # Isaac Sim 창에서 실시간으로 보기 (끝나도 창 유지)
~/isaacsim/python.sh wheel_mount_sim/run.py --mu-shoe .35 .28    # 슈-트레드 마찰 민감도
~/isaacsim/python.sh wheel_mount_sim/run.py --mu-hub .30 .25     # 휠-허브 마찰 민감도
```

조립 셀 맵(`scene/`)에서 좌우 장착로봇 함께 돌리기:

```bash
~/isaacsim/python.sh wheel_mount_sim/cell_mount_run.py --no-video --out wheel_mount_sim/output/cell/run1   # 계산 (~45 s)
~/isaacsim/python.sh wheel_mount_sim/cell_mount_run.py --replay wheel_mount_sim/output/cell/run1           # 창에서 실제 속도(1.0x)로 재생, 반복
~/isaacsim/python.sh wheel_mount_sim/cell_mount_run.py --gui        # 계산하면서 보기: 이 PC에서는 실제의 약 0.4-0.7배 속도
```

공정 연결(`--process`, 2026-11-28): AMR이 게이트에서 차를 싣고 A에 도킹(클램프·리프트) → 좌우 전륜 장착 → B 전진(걸린 휠 유지 측정) → 좌우 후륜 장착 → C → 출차. 체결은 물리가 없어 시험용 완료 신호로 넘어간다. 셀 전체를 기록하므로 `--process --replay 실행폴더`로 처음부터 재생한다. 2026-11-28 기준 공정 모드에서는 파일럿 끝 9 mm 걸림과 놓은 뒤 빠짐이 재현돼 전륜 단계에서 멈추는 경우가 있다(차 고정 모드는 좌우 mounted). 시연용 `--keep-going`은 휠이 덜 끼워져도 끝까지 진행한다.

```bash
~/isaacsim/python.sh wheel_mount_sim/cell_mount_run.py --process --no-video --out wheel_mount_sim/output/cell/proc1
~/isaacsim/python.sh wheel_mount_sim/cell_mount_run.py --process --replay wheel_mount_sim/output/cell/proc1
```

셀은 바퀴 4개의 트레드 충돌체(각 545개) 때문에 물리 계산만으로 실제 시간의 약 0.9배가 걸린다.
그래서 모든 실행이 움직이는 물체 30개의 위치를 초당 30번 `replay.npz`에 기록하고, `--replay`는 물리 없이
벽시계에 맞춰 그 기록을 그린다. 결과(접촉·힘)는 계산 실행의 `summary.json`이 기준이다.

### 장면 USD

`wheel_mount_sim/scene/wheel_mount.usda`는 `run.py`가 만드는 장면을 그대로 내보낸 파일이다(시드 1: 바퀴·허브 각도 고정).
Isaac Sim에서 **File → Open**으로 열어 형상·충돌체·관절·재질을 확인할 수 있다. 로봇 동작(IK·힘 제어)은 `run.py`에만 있으므로,
파일을 열고 재생하면 로봇은 처음 자세를 유지한 채 물리만 돈다. 장면을 바꾼 뒤에는 다시 내보낸다.

```bash
~/isaacsim/python.sh wheel_mount_sim/run.py --export-usd wheel_mount_sim/scene/wheel_mount.usda   # 시드는 --seed
```

결과: `wheel_mount_sim/output/<시각>/` — `log.csv`(1/60 s), `summary.json`(이벤트·최대 접촉력), `wide.gif`·`hub.gif`.
판정 줄은 `EVENT ...`와 `RESULT ...`로 출력된다.

## 파일

| 경로 | 내용 |
|---|---|
| `params.py` | 모든 물리값과 근거(카탈로그/순정/문헌/계산/셀 모델/추정). `python3 params.py > PARAMS_table.md` |
| `PARAMS_table.md` | 위 표 |
| `scene.py` | 작업대, 허브 코너(서스펜션·베어링·파일럿·스터드·로터·캘리퍼), 휠하우스·펜더, 바퀴 장착면·배럴, 링 타이어, 로봇, 그리퍼 |
| `run.py` | 동작 순서: 측정 → 집기 → 들기 → 세우기 → 허브 축 진입 → 비전 정렬 → 힘 제어 삽입 → 놓기 → 후퇴 |
| `geom.py` | USD/PhysX 작성 도구 |
| `scene/wheel_mount.usda` | 장면을 내보낸 USD (Isaac Sim에서 열어 보기용, `--export-usd`로 다시 생성) |
| `tools/` | 에셋 받기, USD 구조 조사 도구 |
| `docs/troubleshooting/` | 트러블슈팅 기록과 이미지(노션 「트러블슈팅 → 장착로봇팔」 원본) |
| `cell_mount.py`, `cell_check.py/.sh` | 조립 셀 라이브 물리에 장착로봇·장착 물리를 넣는 모듈과 셀 안 대기 검사 ([`docs/CELL_INTEGRATION.md`](docs/CELL_INTEGRATION.md)) |
| `cell_mount_run.py` | 조립 셀 맵에서 우리 부분만(AMR 이송 제외): 좌우 장착로봇이 벨트 끝 타이어를 집어 앞 허브에 끼움 |
| `mount_control.py` | 위 제어를 `CellEnv`(셀·PhysX·기록)와 `MountSide`(로봇 1대, 단계별 동작)로 나눈 모듈. 분리 직후 1단계와 이벤트까지 같음을 확인했고(TS-22), 그 뒤 집기·걸림 복구 수정이 들어감. `--car`(A 앞바퀴 → 차량 이동 → B 뒷바퀴), `--car-dynamic`, `--replay-video` |
| `cell_mount_ros2.py`, `cell_mount_ros2.sh` | 2단계: Isaac Sim 쪽 로봇 제어기(서보·240 Hz 힘 제어·그리퍼)와 가짜 AMR. ROS 2로 단계 명령을 받고 `/Parking`을 보냄 |
| `ros2_nodes/mount_sequencer.py` | ROS 2 순서 노드(시스템 Jazzy): `/Parking`을 받으면 바퀴마다 단계를 명령하고 `/Mount`로 결과를 냄 |
| `ros2_config.py` | 토픽 이름·메시지 형식(임시, 한 곳에 모음) |
| `ros2_nodes/run_ros2_test.sh` | 제어기 + 순서 노드 + `/Mount`·`/Parking` 기록을 한 번에 돌리는 시험 |
| `docs/STUD_SPEC.md` | 허브 스터드·너트 규격, 장착 쪽과 체결 쪽 형상 비교 |
| `cell_layout.py`, `tools/extend_belt_end.py`, `cell_feed_check.py` | 벨트 끝 적재식 공급(연장 벨트, V 끝 스토퍼: `scene/overrides/factory.usda`), 그 검사 |
| `docs/GOALS.md` | **목표와 합격 기준** (공정 시간, 단계별 수치 범위와 근거) |

## 현재 상태 (2026-11-27)

| 단계 | 결과 |
|---|---|
| 집기·들기 | 그리퍼 안 미끄러짐 0.02 mm |
| 세우기·운반 | 휠하우스·캘리퍼·펜더 충돌 0 |
| 허브 앞 비전 정렬 | 반경 0.03 mm, 기울기 0.009°, 위상 0.0° |
| 스터드 진입 | 통과 |
| 파일럿 안착(틈 0.086 mm) | 안착 0.32 mm, 130 N (F/T 힘 제어) |
| 놓기·후퇴 | **장착 유지** (간격 1.4 mm, 기울기 1.1°) — `RESULT mounted` |

## 모델링 요약

- **그리퍼**: 2조 평행 트레드 클램프(혼다 US9757828, 미쓰비시 US4942659 방식). 슈는 수평 아래 10°
  (누운 바퀴를 조일 때 tan θ ≤ μ/안전율, 휠하우스 아치 밖). 두 조는 랙·피니언처럼 미믹 관절로 대칭 동기화.
- **타이어**: 림에 스프링으로 연결된 강체 링(반경 180 / 원주 80 / 횡 65 N/mm, 실측 평균 문헌값).
- **허브**: 차를 든 상태(리바운드 스토퍼), 휠 레이트 23 N/mm(승차 진동수 1.3 Hz로 계산), 사전 예압 베어링 끌림 0.75 N·m.
- **충돌**: 장착면은 들로네 삼각형(셀과 같은 방식) + 허브 쪽 보어 모따기, 파일럿은 144각형 프리즘(원통 기본 도형은 가짜 접촉), 스터드는 원통. 접촉 오프셋은 틈·스텝 이동량·놓을 때 기울기로 계산.
- **제어**: Lula IK + 순기구학 뉴턴 정밀화, 로봇 실제-모델 편차 보정, 플랜지 F/T 센서 기반 힘 제어(축 145 N, 옆 25 N/mm).
