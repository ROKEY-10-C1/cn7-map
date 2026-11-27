# map_wheel — 타이어 장착 단독 장면 (한 코너)

전체 셀(`wheel_cell/`)에서 **타이어를 들어 올려 차 허브 볼트에 걸어 놓는 동작만** 떼어 낸 작은 장면입니다. 걸어 놓고 손을 떼면 체결(너트)은 다른 담당이 합니다.

| 들어 있는 것 | 내용 |
|---|---|
| 차 | CN7 1대, 도크 위치 그대로(월드 원점 = 도크), 노란 받침 2개 위에 고정. FL 허브는 비어 있고 나머지 세 바퀴는 끼워진 상태 |
| 로봇 | 두산 H2515 1대, 레일 없이 고정 받침 (1.0, 1.7) |
| 손 | `wheel_cell/gripper.py` 설계 모양: 아래 홈 롤러 2개(5·7시) + 바깥쪽 핀 암·센터 핀 |
| 타이어 받침대 | (1.9, 1.7), 타이어를 세워 둠(장착면 중심 높이 1.10 m), 롤러 2개(4·8시) |

좌표·치수는 셀과 같아서(허브 높이 `process.HUB_Z` 0.94 m, `fitment.half_track`), 여기서 맞춘 동작을 셀로 옮길 수 있습니다.

## 동작 (21 s, 키프레임)

| 시각 (s) | 동작 |
|---|---|
| 0 | 대기 (차 바깥쪽) |
| 3 → 4.5 | 받침대 밑으로 → 그리퍼 롤러가 타이어 아래에 닿음 |
| 5 → 5.5 | 3 cm 들어 올림 (이때부터 손에 붙음) |
| 7 → 10 | 위로 빠져 허브 쪽으로 |
| 12 | 허브 앞 10 cm에서 축 정렬 |
| 12 → 14 | 5 cm/s로 볼트에 걸기 (직선) |
| 15 → 15.5 | 놓기: 핀 빼고 손 2 cm 내림 → 바퀴는 볼트에 걸린 채 남음 |
| 17 → 19.5 | 바깥으로 빠짐 → 대기 |

- 직선 구간은 1/12 s마다 IK(`wheel_cell/robot.py` `Arm`, 툴 자세 고정)로 풀어 툴이 직선으로 갑니다. 관절 속도 한계를 넘으면 빌드가 멈춥니다.
- **지금은 키프레임 미리보기**입니다. 물리(PhysX 접촉, 허브 볼트 끼움, 그리퍼 롤러·핀 구동)는 다음 단계입니다.

## 실행

전체 셀(`wheel_cell/run.sh`)과 따로, 이 장면만 엽니다.

```bash
./map_wheel/run.sh                  # 장면이 없으면 빌드 → Isaac Sim 창에서 반복 재생 (카메라 Bench)
./map_wheel/run.sh --camera Hub     # Hub(허브 정면) | Side | Bench(전체)
./map_wheel/run.sh --once           # 한 번만 재생
./map_wheel/run.sh --rebuild        # build.py로 장면 다시 만든 뒤 열기
./map_wheel/run.sh --smoke-frames 120   # 창 없이 120번 갱신 후 종료 (확인용)
```
- Isaac Sim 위치는 `ISAAC_SIM_ROOT` (기본 `~/isaacsim`). Isaac Sim 5.1.0에서 확인 (2026-11-27, smoke 120 frames 통과)
- 이미 Isaac Sim 창이 열려 있으면 File > Open `map_wheel/scene/mount_bench.usda` 후 Play로도 됩니다.
- `play.py`는 장면만 열고 재생 → `wheel_cell` 코드를 쓰지 않음. `build.py`만 `wheel_cell/`의 치수·IK(`process`, `fitment`, `robot.Arm`)를 가져옴
- 하위 레이어: `scene/shared/asset_library.usdc`, `materials.usdc` (main에 있음, `wheel_cell/build.sh`가 만든 것)
- 장면은 `.usda` 텍스트라 Isaac 없이 usd-core로도 빌드됩니다.

## 다음

- [ ] 물리: 바퀴 강체 + 허브·볼트(`wheel_cell/corner.py`), 그리퍼 롤러·핀(`gripper.py`) 구동, 관절 드라이브로 같은 궤적 → 놓은 뒤 바퀴가 볼트에 걸린 채 버티는지
- [ ] 허브 위치 보정: AMR이 보내는 `/amr/dock_pose`만큼 허브 목표를 옮김 (`wheel_part/INTERFACE.md`)
- [ ] 비전: D455로 허브 볼트 각도 인식 → 바퀴 돌려 맞춤 (여유 약 0.7°)
- [ ] 체결 담당에게 넘김 신호 (`wheel_part/INTERFACE.md` "장착 → 체결 넘김")
