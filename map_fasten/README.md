# 너트 체결 구체화

CN7 좌전륜의 5개 너트를 고정식 Kawasaki RS080N으로 정렬하고 체결하는 Isaac Sim 단독 시험이다. 제어는 Python Standalone에서 PhysX 관절 드라이브로 전달한다.

## 실행

저장소 루트에서 실행한다.

```bash
# GUI에서 체결 시퀀스를 실행
~/isaacsim/python.sh -B map_fasten/fasten_pose_check.py

# GUI 없이 전체 체결 시퀀스를 검증
~/isaacsim/python.sh -B map_fasten/fasten_pose_check.py --headless

# 미리 맞물린 너트에서 나사면 접촉에 의한 축 방향 이동을 검증
~/isaacsim/python.sh -B map_fasten/fasten_pose_check.py --check-thread-contact
```

**CN7 Fastening** 창에서 **Pause**로 멈추고, **Play**로 이어간다. **Restart**는 너트 5개가 소켓에 장착된 초기 상태로 돌아가 다시 실행한다. 기본 타임라인의 **Stop → Play**도 사용할 수 있다.

## 파일 구조

| 위치 | 역할 |
|---|---|
| [`fasten_pose_check.py`](fasten_pose_check.py) | `TaskController.run()`이 준비 → 접근 → 스터드 정렬 → 회전 → 안착 → 소켓 분리 → 후퇴를 실행한다. 같은 파일에서 에셋 생성과 나사면 시험도 실행한다. |
| [`scene/fasten_pose_check.usda`](scene/fasten_pose_check.usda) | RS080N·CN7 차체·좌전륜의 시험 배치, 베이스 고정, TCP를 정의한다. |
| [`scene/fasten_head.usdc`](scene/fasten_head.usdc) | 헤드 생성 결과다. link_6의 어댑터·본체·하우징과 독립 로터·너트·관절을 정의한다. |
| [`assets/`](assets/) | RS080N의 USD와 IK용 URDF, Kawasaki 라이선스다. |
| [`images/`](images/) | 체결 시험 그림이다. |

## 최신 main 인계 사양 반영

- `634054e`의 고정식 **Kawasaki RS080N**과 **QST50-150CT 5축** 구성을 반영했다. RS080N의 링크 메시와 관절 축·한계는 Kawasaki ROS 에셋을 사용했다. 링크 질량은 URDF 값을 사용했고 관절 드라이브와 공구의 질량 분포는 시뮬레이션용 근사다. [Kawasaki 원본](https://github.com/Kawasaki-Robotics/khi_robot/tree/678dd3fbc4f75fd76c1fd5277116d0edc6528f13/khi_rs_description)
- 컴팩트 시험 헤드는 PCD 114.3 mm와 72° 간격을 유지한다. 기존 QST50-150CT 구성의 전체 길이 464 mm와 하우징 외경 51 mm를 각각 235 mm와 40 mm로 줄였다. 고정 공구 질량은 6.8 kg으로 근사했고, 독립 스트로크·D405·반력 핀은 모델링 범위에서 제외했다.
- 전체 셀(`wheel_cell`)의 체결 RS080N 네 동작(좌우 × 앞뒤)도 이 235 mm 헤드와 80 mm 준비 거리를 쓴다. 셀 배치와 시간표는 [진행 기록](../docs/PROGRESS.md)에 있다.
- `6f838b8`과 `2d1e510`의 인계 자료는 휠이 허브에 걸린 뒤 기울어질 수 있음을 보여 준다. 체결 방향은 허브 축에서, 정렬점은 스터드 메시에서 읽는다. [장착 결과 인계](../wheel_cell/HANDOVER.md)

## 현재 시험 범위

시험 장면은 조립된 좌전륜을 고정한 상태에서 시작한다. 준비 접근은 베이스 회전과 팔 접기 두 구간으로 나눈다. 로그의 TCP 도착 오차와 너트별 정렬·안착 오차는 이 고정 장면에서 측정한다. 전체 차체 간섭과 인계 JSON의 공차·기울기 및 실시간 장착 연동은 별도 검증이 필요하다.

현재 체결 시퀀스는 회전량과 피치로 로봇 이송량을 계산하고 안착 후 너트를 조인트로 고정한다. 접촉 시험은 처음부터 암나사와 스터드가 맞물려 있다. 스핀들 50 mm 플로팅, 최초 나사 물림, D405 인식, 반력 핀 작동, 최종 토크와 축력은 설계 구체화 항목이다.

고정 시험 장면에서 TCP 준비 위치 오차는 0.46 mm, 스터드 진입의 최대 오차는 0.12 mm였다. 너트 5개가 안착하고 공구가 약 102 mm 후퇴한 뒤 너트가 휠에 남았다. [소켓에 너트 장착](images/loaded_sockets.png) · [공구 후퇴 후 너트 안착](images/seated_nuts.png)

## 모델 다시 생성

컴팩트 헤드를 실행하기 전에 `--build-head`로 헤드 USD를 다시 생성한다. 생성기는 초기 관절의 연결 대상과 월드 기준점, TCP 위치를 검증한 후 USD를 교체한다.

RS080N USD는 저장소에 포함되어 있다. 수정 후 다시 만들 경우 Kawasaki 원본 저장소의 해당 버전 STL 폴더를 사용한다.

```bash
~/isaacsim/python.sh -B map_fasten/fasten_pose_check.py --build-robot <Kawasaki-STL-폴더>
~/isaacsim/python.sh -B map_fasten/fasten_pose_check.py --build-head
```
