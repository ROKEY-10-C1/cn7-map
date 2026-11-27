# CN7 휠 조립 셀

Isaac Sim에서 CN7 차체 이송, 휠 장착, 너트 체결을 개발하고 검증하는 저장소다. 장착과 체결은 각 단독 시험으로 먼저 확인하고, 전체 공정은 조립 셀에서 검증한다.

## 프로젝트 구조

| 경로 | 역할 |
|---|---|
| [`map_wheel/`](map_wheel/README.md) | 좌전륜을 허브 스터드에 거는 단독 장착 장면 |
| [`map_fasten/`](map_fasten/README.md) | 너트 체결 구체화. Kawasaki RS080N과 5축 헤드의 단독 시험 |
| [`wheel_cell/`](wheel_cell/README.md) | 전체 조립 셀 생성·실행·물리 검증과 공용 제어 코드 |
| [`wheel_cell/HANDOVER.md`](wheel_cell/HANDOVER.md) | 장착 완료 휠의 허브 기준 상태, 스터드 데이터와 체결기 설계 사양 |
| [`scene/`](scene/README.md) | 전체 조립 셀의 USD 시작 장면과 공장·재질·주문별 레이어 |
| [`assets/`](assets/README.md) | 차체, 휠, AMR, 로봇 등 공용 USD 모델 |
| [`docs/`](docs/PROGRESS.md) | 공정 검증 기록과 참고 이미지 |

## Isaac Sim 실행

저장소 루트에서 실행한다. Isaac Sim 설치 경로가 다르면 해당 설치의 `python.sh`를 사용한다.

```bash
# 전체 조립 셀
./wheel_cell/run.sh

# 좌전륜 장착 단독 시험
./map_wheel/run.sh

# 좌전륜 너트 체결 단독 시험
~/isaacsim/python.sh -B map_fasten/fasten_pose_check.py
```

## 커밋 메시지

사람과 LLM 모두 **영어 접두사 뒤에 무엇을 바꿨는지 설명하는 한국어 문장**을 쓴다. 서로 다른 목적의 변경은 커밋을 나누고, 커밋 전에 스테이징된 파일을 확인한다.

| 접두사 | 사용 시점 | 예시 |
|---|---|---|
| `feat:` | 기능이나 시험 진입점을 추가할 때 | `feat: RS080N 기반 너트 체결 시험을 추가한다` |
| `docs:` | 실행 안내, 설계 자료, 작업 규칙을 정리할 때 | `docs: 조립 셀 구조와 커밋 규칙을 정리한다` |
| `fix:` | 기존 동작의 결함을 수정할 때 | `fix: 체결기 접근 중 차체와의 간섭을 줄인다` |

LLM 작업을 시작할 때 저장소 루트의 [AGENTS.md](AGENTS.md)를 읽도록 안내한다. 작업을 맡길 때는 변경할 폴더와 완료 기준을 함께 적는다.
