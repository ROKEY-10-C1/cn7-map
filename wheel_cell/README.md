# CN7 AMR 휠 조립 셀

HMGICS 셀 생산 방식을 참고한 Isaac Sim 5.1 물리 환경입니다. 환경은 한 가지 모델로만 동작합니다(켜고 끄는 대체 모델 없음).

- **휠 공급 (방법 1)**: 타이어는 눕힌 채 스테이션 컨베이어로 들어와 집는 구역에서 멈춥니다. 광센서 S1에서 감속, 120° V 스토퍼에 닿아 S2가 감지하면 정지, 센터링 암이 가운데로 맞춘 뒤 리프트 바가 들어 올려 `pick_ready`가 됩니다. 광센서는 PhysX 광선 검사입니다.
- **장착 로봇 그리퍼**: 홈 파인 구동 롤러 2개(5시·7시)와 접히는 센터 핀(콜릿)이 장착 로봇 플랜지에 달려 있습니다. 로봇과 그리퍼 제어는 아직 없습니다(로봇은 시작 자세 유지).
- **차체 모서리**: 허브는 베어링 위의 별도 바디입니다(임의 정지 각도, 끌림 1.2 N·m, 반경 유격). 너클은 서스펜션으로 차체에 달려 들어 올린 차처럼 리바운드 스토퍼에 얹혀 있습니다. 파일럿·스터드 공차와 진입 모따기가 있습니다.
- **타이어**: 림과 트레드 링이 별도 바디이고 반경·횡·비틀림 스프링으로 연결됩니다.
- **측정**: 스테이션은 휠 볼트 패턴과 허브 스터드의 각도를 카메라 모델(30 Hz, 1프레임 지연, 잡음)로 측정해 제어에 제공합니다.

수치는 `cell_config.py`, 실측이 필요한 값은 `calibration.json`에 있습니다.

## 실행

```bash
cd "$HOME/project"
./map/wheel_cell/run.sh --wheel-detail --seed 42
```

**CN7 | Live PhysX** 창에서 **Play**를 누릅니다. 기본은 주문 한 대의 전체 흐름입니다: AMR이 입고 버퍼 1번 칸의 차 밑으로 들어가 들어 올리고(위치결정 블록 팝업), 통로 끝에서 0°로 돌아 게이트로 가서 게이트 높이로 내린 뒤 도킹합니다(약 71초에 도킹). 게이트부터만 보려면 `--start gate`(차를 실은 채 게이트에서 출발, 약 19초에 도킹)를 붙입니다. `--assembly`는 셀 전체, `--hub-detail`은 허브, `--top`은 상부 카메라를 엽니다. `--seed`를 생략하면 휠의 초기 회전각과 허브 정지 각도·공차를 새로 뽑습니다.

- 장면은 `map/scene/wheel_cell.usda`(루트)와 영역별 레이어로 나뉩니다. 라이브 실행에서 참조하는 모델과 별도의 키프레임 시연을 담습니다. 나눠서 작업하는 방법은 [`../scene/README.md`](../scene/README.md)를 보세요. `run.sh --replay` 또는 Isaac Sim의 기본 타임라인에서 시연을 재생할 수 있습니다.
- `output/wheel_cell_physics.usdc`는 강체, 재료, 관절과 초기 자세를 포함합니다. Isaac Sim에서 직접 열면 초기 상태의 물리 실험과 자체 롤러 구동을 볼 수 있습니다. AMR 주행·파지·삽입 명령은 위 라이브 실행에서 작동합니다.
- 기본 렌더링은 1080p RTX이며 `--quality cinematic`은 1440p 패스 트레이싱입니다. 실행기의 기본 VRAM 감시 기준은 8 GiB입니다. USD를 옮길 때는 `concrete.png`를 같은 폴더에 둡니다.

## 적용한 데이터

| 대상 | 물리에 반영한 값과 근거 |
|---|---|
| 아반떼 차체 | CN7 페이스리프트 G1.6 IVT 16인치를 기준으로 전장 4,710 mm, 전폭 1,825 mm, 축거 2,720 mm, 전후 윤거 1,585 / 1,596 mm를 적용했습니다. 카탈로그 공차중량 1,265 kg에서 아래 휠 네 개(17.1 kg × 4)를 뺀 1,196.6 kg을 차체 질량으로 계산했습니다(`physical_data.BODY_MASS`). PhysX 설정은 아래 [차체 PhysX 설정](#차체-physx-설정-라이브-셀)에 있습니다. [현대차 카탈로그](https://www.hyundai.com/contents/repn-car/catalog/the-new-avante-catalog.pdf) |
| 휠 | 205/55 R16, 6.5J × 16, 이론 외경 631.9 mm를 적용했습니다. Continental Elantra 적용 품번 `15570360000`의 8.0 kg 타이어와 CN7 순정 16×6.5J ET46 림(52910-AB200/AA230) 9.1 kg을 합쳐 17.1 kg으로 모델링했습니다. 림 무게는 wheel-size.com 한 곳의 값으로 검증되지 않았고, 이전 비교품(Konig 7.9 kg)보다 무거워 하중 쪽으로 보수적인 값입니다. 관성은 타이어와 림의 환형 단면에서 추정해 USD에 명시했습니다. [현대차 매뉴얼](https://ownersmanual.hyundai.com/full_webhelp/CN7/2025/en_GN/idf379ca53e75.html) · [Continental 제품 가이드](https://www.continental-tires.com/content/dam/conti-tires-cms/continental/market-content/ca/download-pdfs/product-guide/CT2024-TireGuide_v6b.pdf.coredownload.pdf) · [Konig Lockout](https://konigwheels.com/products/konig-lockout) |
| 허브 | PCD 5×114.3 mm, 센터 보어 Ø67.10 mm, M12×1.5는 설계 기준입니다. 파일럿 Ø66.94 mm, 돌출 12 mm, 휠 오프셋 +46 mm, 스터드 돌출 30 mm, 홀 Ø13.4 mm는 실측 전 가정입니다. 이 치수의 보어–파일럿 반경 여유 0.08 mm를 충돌 형상에 그대로 적용했습니다. |
| 컨베이어 | Interroll EC5000 35 W, 49:1 형식의 Ø50 mm, 튜브 두께 1.5 mm, 정격 2.42 N·m 및 최고 반송 속도 0.37 m/s를 적용했습니다. 롤러는 튜브 질량·관성을 가진 강체로 회전합니다. 속도 제어 이득과 롤러 외피 마찰은 추정값입니다. 가이드 라이너의 동마찰은 UHMW-PE 자료 범위 안에서 0.22를 적용했습니다. [Interroll 데이터시트](https://www.interroll.com/fileadmin/user_upload/Downloads__PDF_/Drives___Controls/DC_Platform/EC5000_Catalog_PDF/EN/EC5000_50mm_IP66_EN.pdf) · [TIVAR 1000 EC 데이터시트](https://www.mcam.com/mam/38052/Nylaplas-PE-TIVAR%C2%AE%201000%20EC%20UHMW-PE_en_US.pdf) |
| 집는 구역 | V 스토퍼 판은 UHMW, 120°, 완충 스프링 40 N/mm(추정). 구역 롤러 0.32 m/s, S1 이후 0.08 m/s. 센터링 암은 공압 150 N(추정), 리프트 바 0.1 m/s. 상부 카메라는 D455(1.93 mm 렌즈) 기준입니다. |
| AMR·승강 | KUKA KMP 1500 omniMove의 차대 711 kg, 리프트 144 kg, 정격 적재 1,500 kg, 직진 1 m/s, 횡주행 0.56 m/s, 승강 0.05 m/s를 기준으로 설정했습니다. 차체는 고무 지지 패드와 위치결정 블록 위에 놓여 하중을 전달합니다. 이동은 제한된 등가 추진력으로 계산합니다. 메카넘 롤러별 미끄러짐과 배터리·모터 전달계는 생략했습니다. [KUKA 사양서](https://www.kuka.com/-/media/kuka-downloads/files/87f2706ce77c4318877932fb36f6002d/kuka_kmp1500_en.pdf?hash=90820FB7DA3F2999B45EFC28EF98A626&rev=c434eb6522c4445e8b65c5e1fdde221c) |
| 로봇·툴 | 장착·체결 모두 고정식 Kawasaki RS080N(좌우 각 1대, 가반하중 80 kg)입니다. URDF·링크 메시는 [`map_fasten/assets/`](../map_fasten/assets/)(Kawasaki `khi_robot` `678dd3f`, 링크 질량 합 약 555 kg)를 씁니다. 장착 공구는 SCHUNK PHL-W 63-080-S(18.7 kg, [`wheel_mount_sim/params.py`](../wheel_mount_sim/params.py)), 체결 공구는 map_fasten의 컴팩트 5축 헤드(플랜지~소켓 끝 235 mm, [`map_fasten/README.md`](../map_fasten/README.md))이고, 설계 기준 사양은 QST50-150CT 5축([`HANDOVER.md`](HANDOVER.md))입니다. 라이브 물리의 로봇 모델은 아직 RS080N으로 옮기지 않았습니다. |

### 차체 PhysX 설정 (라이브 셀)

`cell.py`가 만드는 `/World/Live/CN7` 기준입니다. 키프레임 장면 `scene/wheel_cell.usda`에는 강체·충돌이 없고 라이브 실행에서만 붙습니다. 에셋 파일 `assets/vehicle/Hyundai_Avante_CN7_Body.usd`의 물리(외판 볼록 껍질)와는 다른 형상입니다.

| 항목 | 값 | 위치·근거 |
|---|---|---|
| 강체 | 동적 강체 하나 (kinematic 아님), 1,196.6 kg | `cell.py`, `physical_data.BODY_MASS` |
| 무게중심·관성 | 지정하지 않음 → PhysX가 충돌 형상에서 계산 (실차 값으로 교정 안 됨) | 실측 필요 |
| 솔버 | 위치 반복 64, 속도 반복 8, 물리 240 Hz | `cell_config.py` `physics` |
| 충돌 형상 | `Undertray` 상자 3.70 × 1.16 × 0.23 m (z 1.10, 바닥 판 대용), `Body` 상자 4.56 × 1.57 × 0.36 m (z 1.55), 사이드실·범퍼 모서리 블록(차체 메시를 0.1 m 간격으로 잘라 만든 높이 0.10 m 블록, 휠 아치 구간 제외) | `cell.py`, `vehicle.body_underside` |
| 접촉 | contact offset 2 mm, rest offset 0. 충돌체는 보이지 않음 | `sim_util.collision` |
| 재질 | 차체 충돌체에는 물리 재질 없음 → PhysX 기본 마찰. 차를 받치는 행거 패드 4개는 Rubber(0.82) | `cell.py` |
| 접촉 보고 | 켜짐 (받침대·행거 간섭 기록) | `cell.py`, `vehicle.BufferRun.observe` |
| 코너 4곳 | 너클 11 kg이 서스펜션 조인트로 차체에 연결: 상하 22 N/mm, 감쇠 1,800 N·s/m, 처짐 0~90 mm, 예압 6 mm(리바운드 스토퍼에 걸린 상태), 차축 방향 900 N/mm·±4 mm. 허브 6.5 kg은 베어링 조인트로 자유 회전(끌림 1.2 N·m, 반경 유격 0.03 mm) | `corner.py`, `cell_config.py` `suspension`·`hub` (모두 추정) |
| 휠 | 림과 트레드 링이 별도 강체, 반경·횡·비틀림 스프링으로 연결. 컨베이어 집는 구역까지 물리로 이동하며, 로봇 제어가 없어 허브 장착은 아직 물리로 일어나지 않음 | `station.py` |

타이어·현가계·베어링 값은 추정이며 `calibration.json`에 실측 방법과 함께 있습니다. 휠 너트의 현대차 공칭 체결 범위는 107–127 N·m입니다. 너트 체결(5축 멀티 너트러너로 휠당 1회에 5개 동시 런다운·최종 토크)·체결 토크 기록·출차·불량 배출 주문은 키프레임 시연에 포함되며, 나사산 축력이나 공구 반력은 물리 검증 범위 밖입니다.

## 검증 및 생성

```bash
./map/wheel_cell/build.sh
./map/wheel_cell/check_physics.sh --seed 42
./map/wheel_cell/check_physics.sh --supply-probe left
./map/wheel_cell/check_physics.sh --supply-probe right
./map/wheel_cell/check_physics.sh --interference-probe
./map/wheel_cell/check_physics.sh --insertion-probe nominal
./map/wheel_cell/check_physics.sh --insertion-probe skew
./map/wheel_cell/check_physics.sh --export-setup "$PWD/map/wheel_cell/output/wheel_cell_physics.usdc"
```

부분 실행: `check_physics.sh --start gate`(게이트부터), `--buffer-probe 0|1|2`(버퍼에서 꺼내 통로로 나오기까지만), `--until 초`(그 시각까지만). 기본 검사는 기준 질량·관성, 중복 바디, AMR 도킹, USD와 PhysX 자세 일치, 네 타이어가 집는 위치(흐름 방향 ±3 mm, 폭 방향 ±2 mm)에 `pick_ready`로 섰는지, 로봇과 집는 구역 장비에 다른 물체가 닿지 않았는지 확인합니다. `--until 초`는 그 시각까지만 돌리고 고장 여부만 봅니다. `--supply-probe left|right`는 1F 공급점에서 투입한 휠 한 개를 B1의 해당 픽업 위치까지 물리 구동하고 `pick_ready`를 확인합니다. `--layout-probe`는 적재 외형의 통과 여유, 인덱싱 인터록, 세 정차점 및 전방 출차를 검증합니다. 자동 타이어 투입과 후륜 공급 전환은 운영 제어에서 연결해야 합니다.

`--insertion-probe`는 차체를 고정한 시험대에서 FL 휠을 힘 제한(145 N) 슬라이드로 허브에 밀어 넣습니다. `nominal`은 장착면 안착을, `skew`는 4 mm 반경 어긋남에서 스터드가 삽입을 막는지를 확인합니다.

회귀 시험: `python3 map/wheel_cell/regress.py` (전체), `--area cycle|seeds|insertion|supply|rails`, `--changed`(바뀐 파일 기준).

다시 빌드할 때 `ISAAC_SIM_ROOT`와 `--asset-root`, `--tool-usd`로 로컬 설치 및 자산 경로를 지정할 수 있습니다. RS080N 링크 메시는 `map_fasten/assets/RS080N.usdc`에서 읽습니다. [Kawasaki 자산 라이선스](../map_fasten/assets/LICENSE-Kawasaki.txt)

현재 실행 환경에서 검증한 물리는 CPU PhysX입니다. RTX 화면과 VRAM 사용량은 별도 장비 측정이 필요합니다.

## KMP 1500 URDF (ROS 2 · Isaac URDF Importer용)

`assets/kmp1500.urdf`는 사양서 크기의 KUKA KMP 1500입니다. `physical_data.py` 값으로 `kmp1500_urdf.py`가 만들므로, 고칠 때는 스크립트를 수정하고 `python3 map/wheel_cell/kmp1500_urdf.py`로 다시 생성합니다.

| 항목 | 값 | 근거 |
|---|---|---|
| 외형 | 2,000 × 800 × 470 mm (스캐너 포함) | 사양서 "Dimensions (L × W × H)" |
| 질량 | 차대 711 kg + 리프트 144 kg = 855 kg | 사양서 "Weight", "Lifting system – Weight +144 kg" |
| 리프트 | `lift_joint` 직동 0–200 mm, 50 mm/s | 사양서 "Lift table height max. 200 mm", "Lifting speed max. 50 mm/s" |
| 바퀴 | Ø310 mm × 4, x ±0.72 m, y ±0.31 m | 지름은 사양서 "Wheel diameter 310 mm". 개수·위치는 사양서에 없음 → `build.py` `amr_housing()`과 같은 추정값 |
| 스캐너 기종 | **SICK S300** × 2: 270°, 각도 분해능 0.5°, 응답 80 ms, 측정 거리 30 m, 102 × 152 × 106 mm, 1.2 kg | S300 수치는 SICK S300 Standard 데이터시트(S30B-2011BA, 품번 1026820). KMP 1500 사양서는 "Integrated safety laser scanners"만 언급하고 기종은 공개되지 않음 → 같은 KUKA KMP/KMR 계열에 쓰인 S300을 채택(아래 출처) |
| 스캐너 배치 | `laser_front_left` (0.92, 0.32, 0.20 m, 45°), `laser_rear_right` (−0.92, −0.32, 0.20 m, −135°) | **대각선 두 모서리**: KMP 베이스가 S300 2개를 대각선으로 단다는 자료를 따름(270° × 2로 360°). 정확한 좌표·높이는 추정(2,000 × 800 mm 외곽 안) |
| IMU | `imu_link` (0, 0, 0.25 m), 자이로 z축으로 회전 보정 | KUKA 자료에 없음. 바퀴 오도메트리 보정용으로 추가한 설계 선택, 기종·위치 추정 |
| 외형 메시 | `assets/kmp1500_meshes/` 스커트·팬·상부 셸 STL + 상태 표시등 | 장면 `amr_housing()`의 외곽선·높이를 그대로 사양서 외곽(2,000 × 800 × 470 mm)에 맞춤. 모양은 제품 사진을 보고 근사한 것이며 KUKA CAD가 아님 |
| 리프트 테이블 판, 지상고, 바퀴 질량 | 1.50 × 0.60 × 0.03 m (상부 셸 안), 40 mm, 20 kg | 사양서에 없음 → 추정 |
| 속도 (주석) | 직진 1 m/s, 대각·횡 0.56 m/s | 사양서 "Velocity straight ahead", "Velocity diagonally and sideways" |

출처: KUKA, *KMP 1500* 제품 사양서(영문, © 2022), 2026-11-26 확인. [PDF](https://www.kuka.com/-/media/kuka-downloads/files/87f2706ce77c4318877932fb36f6002d/kuka_kmp1500_en.pdf?hash=90820FB7DA3F2999B45EFC28EF98A626&rev=c434eb6522c4445e8b65c5e1fdde221c) — 같은 사양서가 위 "AMR·승강" 행의 출처입니다. 사양서에는 치수 도면과 CAD 데이터가 없습니다.

센서 출처 (2026-11-26 확인):
- SICK, *S30B-2011BA* (S300 Standard) 데이터시트, 시트 날짜 2026-10-10. [PDF](https://www.sick.com/media/pdf/5/45/845/dataSheet_S30B-2011BA_1026820_en.pdf) — "Scanning angle 270°", "Angular resolution 0.5°", "Response time 80 ms", "Distance measuring range 30 m", "Dimensions 102 mm x 152 mm x 106 mm", "Weight 1.2 kg"
- SICK 블로그, *Safety and more – SICK provides protection and navigation data for KUKA's KMR iiwa* — KUKA omniMove 대형 플랫폼에 SICK 제품 통합, KMR iiwa에 S300 2개, 스캐너 데이터를 KUKA 내비게이션이 사용. [링크](https://www.sick.com/at/en/safety-and-more-sick-provides-protection-and-navigation-data-for-kukas-kmr-iiwa/w/blog-safety-and-more-sick-provides-protection-and-navigation-data-for-kukas-kmr-iiwa)
- *Mobile Robots using ROS2* (Scribd 문서) — KMP 베이스: 메카넘 바퀴 4개, SICK S300 2개 대각선 배치. [링크](https://www.scribd.com/document/796485986/Mobile-Robots-using-ROS2)
- SICK KUKA 솔루션 페이지 — microScan3 Pro, S300 Mini 등재(신형 KMP 1500은 microScan3일 수 있음, 미확인). [링크](https://sick.com/cn/en/w/kuka)

장면의 `amr_housing()`은 아직 스캐너를 앞뒤 가운데에 그립니다. 장면 빌드(Isaac 필요)를 바꿀 때 대각선 배치로 맞춥니다.

메시 경로는 URDF 기준 상대 경로(`kmp1500_meshes/*.stl`)라 Isaac URDF Importer는 그대로 읽습니다. RViz는 `file://` 절대 경로가 필요하므로 불러올 때 바꿔 줍니다. 리프트는 `/joint_states`의 `lift_joint` 값(0–0.2 m)으로 올립니다.

좌표계는 REP 105를 따릅니다(`base_link`는 바닥의 차대 중심, x 전방). omniMove 메카넘 롤러는 모델링하지 않았으므로 전방향 주행은 바퀴 관절이 아닌 holonomic 베이스 제어로 구동합니다. 차체 운반용 행어(`Hanger`)는 셀 장비라서 포함하지 않았습니다.
