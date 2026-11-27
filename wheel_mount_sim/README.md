# 좌전륜 장착 단독 시뮬레이션 (`wheel_mount_sim/`)

컨베이어 높이(0.50 m) 작업대에 누워 있는 CN7 바퀴를 **Kawasaki RS080N**이 **SCHUNK PHL-W 63-080-S** 트레드 클램프로
집어 세우고, 앞 왼쪽 허브에 끼운 뒤 놓는 단독 시험이다. 전체 셀(`wheel_cell/`)·다른 단독 시험(`map_fasten/`)과
섞이지 않도록 이 폴더에서만 관리한다. 공용 파일은 읽기만 한다(`assets/vehicle/CN7_Wheel_205-55R16.usd`).

## 실행 (저장소 루트에서)

```bash
./wheel_mount_sim/tools/fetch_assets.sh                          # 처음 한 번: RS080N USD 받기 (커밋하지 않음)
~/isaacsim/python.sh wheel_mount_sim/run.py --no-video           # 빠른 확인 (~5분)
~/isaacsim/python.sh wheel_mount_sim/run.py --fast --no-video    # 공정 시간 단축 프로필 (바퀴 1개 22.3 s)
~/isaacsim/python.sh wheel_mount_sim/run.py                      # 영상(GIF) 포함
~/isaacsim/python.sh wheel_mount_sim/run.py --gui --fast --no-video   # Isaac Sim 창에서 실시간으로 보기 (끝나도 창 유지)
~/isaacsim/python.sh wheel_mount_sim/run.py --mu-shoe .35 .28    # 슈-트레드 마찰 민감도
~/isaacsim/python.sh wheel_mount_sim/run.py --mu-hub .30 .25     # 휠-허브 마찰 민감도
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
| `tools/` | 에셋 받기, USD 구조 조사 도구 |
| `docs/troubleshooting/` | 트러블슈팅 기록과 이미지(노션 「트러블슈팅 → 장착로봇팔」 원본) |
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
