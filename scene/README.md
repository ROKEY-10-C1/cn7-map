# scene — 분할 작업용 장면

Isaac Sim에서 **`wheel_cell.usda`**를 여세요. 이 루트 파일이 아래 레이어를 모두 불러옵니다.
레이어 하나만 따로 열면 공용 메시(`/AssetLibrary`)와 재질 참조가 풀리지 않으니, 항상 루트로 여세요.

```
scene/
├── wheel_cell.usda            # 루트 (빌드가 다시 씀)
├── overrides/                 # ✏️ 손으로 편집하는 레이어 — 빌드가 절대 덮어쓰지 않음
│   ├── environment.usda       #   조명·카메라
│   ├── factory.usda           #   건물·바닥·타이어 공급 라인·로봇팔 레인·펜스·대기구역·출고구역
│   ├── orders.usda            #   차체·AMR·로봇·공급 휠
│   └── shared.usda            #   재질·공용 메시
├── shared/                    # 🔁 생성됨: asset_library.usdc, materials.usdc
├── environment/               # 🔁 생성됨: lighting_cameras.usdc
├── factory/                   # 🔁 생성됨: building, floor, conveyor, mount_robots, fasten_robots, safety, vehicle_buffers, truck
├── orders/                    # 🔁 생성됨: CN7_001 … CN7_004
├── concrete.png
└── production.sqlite3         # 시연 이벤트 로그
```

## 레이어 순서 (위가 강함)

`overrides/*` → `shared/*` → `environment/*` → `factory/*` → `orders/*`

override 레이어에 쓴 값이 생성된 값을 이깁니다. 빌드를 다시 해도 override는 그대로 남아요.
(확인함: `overrides/factory.usda`에서 `TruckCab`을 숨긴 뒤 다시 빌드해도 숨김이 유지됨)

## 나눠서 작업하는 방법

1. 담당 영역을 정합니다(예: A=factory, B=orders/로봇, C=environment).
2. Isaac Sim **Layer** 패널에서 자기 `overrides/<영역>.usda`를 **Authoring Layer**(편집 대상)로 지정합니다.
3. 그 상태에서 옮기거나 숨기거나 속성을 바꾸면 그 override 파일에만 저장됩니다.
4. 각자 자기 override 파일만 커밋하면 git 충돌이 나지 않습니다.

| 하고 싶은 일 | 어디서 |
|---|---|
| 배치 조정, 숨기기, 재질·조명 바꾸기 | `overrides/<영역>.usda` |
| 형상·공정 로직·물리 값 바꾸기 | 코드 `map/wheel_cell/*.py` 수정 후 `./map/wheel_cell/build.sh` |
| 새 에셋 추가 | `overrides/<영역>.usda`에 reference로 추가 |

⚠️ `shared/`, `environment/`, `factory/`, `orders/` 아래의 `.usdc`나 `wheel_cell.usda`를 직접 고치면, 다음 빌드 때 사라집니다.

## 변경 기록

### 2026-11-27 바닥 확장 · AMR 주행 구역 표시 (`factory/floor.usdc`, `factory/building.usdc`, `shared/materials.usdc`)
- 출고 쪽(남쪽) 바닥을 8 m 늘림: 바닥 y -18 ~ +10. 슬래브·줄눈·끝 벽 포함
- `/World/AMRZone`: AMR이 차를 싣고 다니는 구역을 바닥에 표시(통로 띠, 칸 줄 띠, 통로 양 끝 회전 원). 페인트만 있고 충돌체 없음
- Isaac Sim 5.1.0 `build.sh`로 생성(crate 0.9). 자세한 내용: `docs/PROGRESS.md` 3절

### 2026-11-26 차량 버퍼 받침대 (`factory/vehicle_buffers.usdc`)
AMR+행거가 칸에 세워 둔 차 밑으로 옆 이동해 들어갈 수 있게 고쳤습니다. 흐름과 칸 순서는 [`../docs/PROCESS_SCENARIO.md`](../docs/PROCESS_SCENARIO.md) 1단계를 보세요.

| 항목 | 이전 | 이후 |
|---|---|---|
| 입고 칸 받침대 | 바퀴 자리 4곳 `Vehicle_i/Skid_*` (높이 0.44 m) | 차체 앞뒤 끝 4곳 `Stand_i_*` (차 기준 x ±2.05, y ±0.68, 0.2 × 0.2 m) |
| 출고 칸 | 타이어로 바닥에 섬 | 입고 칸과 같은 받침대 위 (타이어는 떠 있음) |
| 칸의 차 높이 (행거 오프셋) | 입고 -0.56, 출고 -0.62 | 양쪽 -0.09 (`BUFFER_CAR_Z`), 차체 바닥 0.918 m |
| 받침대 윗면 | — | 앞 0.971, 뒤 0.941 m (차체 바닥에 닿음) |

- 확인(이 PC, usd-core 26.11로 생성): 빈 AMR+행거(진입 높이 -0.12)가 통로 x -7에서 칸 3까지 옆 이동할 때 장면 형상과 겹치지 않음. 차를 실은 AMR(-0.04)은 빈 칸 받침대 위를 4.7 cm 띄워 지나감.
- 이 레이어는 usd-core로 생성해 crate 0.8 형식입니다(Isaac 생성본은 0.9). 나머지 레이어는 내용이 같아 되돌렸습니다.
- 키프레임 시연은 그대로라 주문 차량이 칸이 아니라 통로 끝에서 실린 채 나타납니다.
- amr_part 2D 맵 (2026-11-27 이름 변경, 전 amr_test): `usd_to_map.py ... --z-min 0.05 --z-max 0.70 --exclude RightVehicleBuffer/Vehicle_` (칸의 차체·허브는 빠지고 받침대만 남음)

### 2026-11-26 차량 버퍼 물리 검증 (`check_physics.sh --buffer-probe 0|1|2`)
라이브 PhysX로 AMR+행거가 입고 칸의 차를 꺼내는 과정을 시험했습니다(받침대는 충돌체, 차는 물리 바디). 고친 것:

| 항목 | 이전 | 이후 | 근거 |
|---|---|---|---|
| 받침대 윗면 | 앞 0.971, 뒤 0.941 m (mm 반올림) | 앞 0.9715, 뒤 0.9405 m | 차체 최저점과 틈 0 (이전: 앞 0.5 mm 뜸, 뒤 0.5 mm 파고듦) |
| 진입 높이 `AMR_LIFT_ENTRY` | -0.12 | -0.16 | -0.12에서 행거 옆 위치결정 블록이 차 사이드실에 2 mm 걸려 AMR이 칸 1.41 m 앞에서 멈춤. -0.14에서는 패드가 휠 아치 아래 끝과 5 mm |
| 행거 위치결정 블록 | 고정 | 팝업식(공압, 60 mm 스트로크): 진입 때 내리고 들어 올린 뒤 올림 | 고정 블록은 어느 진입 높이에서도 사이드실·바닥 판과 겹침 |
| 리프트 스트로크 | 160 mm | 200 mm (행거 오프셋 -0.18 ~ +0.02) | KMP 1500 사양서 "Lift table height max. 200 mm" |
| 차체 충돌 형상 | 바닥 판·몸체 상자 2개 | + 사이드실·범퍼 모서리 블록(차체 메시에서 0.1 m 간격으로 잘라 냄, 휠 아치 구간 제외) | 받침대가 닿는 곳이 사이드실 끝이라 상자만으로는 차가 받침대 위 31 cm 떠 있었음 |

결과: 칸 1·2·3 모두 진입 → 들어 올림(받침대에서 뜸) → 빈 칸 받침대 위를 지나 통로로 복귀, 의도하지 않은 접촉 0건. 이송 높이 -0.043 m(하중 처짐 3 mm 포함).

## 실행

```bash
cd ~/project
./map/wheel_cell/build.sh                      # 생성 레이어 다시 만들기 (overrides 유지)
./map/wheel_cell/run.sh --replay               # 키프레임 시연
./map/wheel_cell/run.sh --assembly --seed 42   # 라이브 물리 (Play)
```
