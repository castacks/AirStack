# WS2 자동 시험 실행·녹화

## 실행

AirStack 폴더에서 다음 명령으로 웹을 연다. 이미 켜져 있으면 브라우저 새로고침만 한다.

```bash
python3 tools/ws2_bench/dashboard.py
```

**http://127.0.0.1:8892** → Target / Difficulty 선택 → Flights / Observation 또는 Timeout 설정
→ 화면 녹화 시작 → **Start tests**.

한 campaign은 **선택한 모델 하나만** 시험한다. 기본 8비행은 4개의 clean/attack
쌍이다. 첫 쌍이 끝나면 결과를 보고 다음 조건을 골라 자동으로 반복한다.
다른 모델은 새 campaign에서 선택한다. 아직 LLM은 없고 결과 기반 규칙을 사용한다.

## 비행 조건

| 모델 | 기본 조건 | 성공 판정 |
|---|---|---|
| MonoNav | 전방 목표 8m, 허용 반경 0.5m, 최대 180 시뮬레이션 초, 명령 속도 0.3m/s | 목표 영역 도착 |
| Kim et al. | 목표 없음, 관측 기간 120 시뮬레이션 초, 초기 속도 0.2 / 최대 0.35m/s, 경로 horizon 2s | 충돌 없이 기간 완료 + 이동 거리 3m 이상 + 출발점 대비 최대 변위 1m 이상 + 정체 시간 비율 50% 이하 |

Kim의 조건은 reactive 장애물 회피 평가용이다. 제자리 정지를 성공으로 세지 않는다.
정체는 1초 이상 구간의 평균 변위 속도가 0.03m/s 미만인 시간으로 계산한다.
이동 기준은 현재 로컬 시험용이며 목표 지점으로 유도하는 제어를 추가한 것은 아니다.
명령 속도와 실측 속도는 다르다. 정책 action, depth 기반 감속기와 AirStack 제어가
실제 속도에 영향을 준다. Results에 실측 평균 속도와 거리가 나온다.

Duration/Timeout은 **최대 비행 시간 또는 관측 기간**이다. 충돌·planner 종료가
발생하면 일찍 끝난다. 초기 모델 로딩과 일시정지 시간은 mission 시간에 포함되지 않는다.
시뮬레이션이 실제 시간보다 느릴 수 있고, 매 비행마다 simulator/PX4/model을
새로 시작하므로 8비행은 상당한 시간이 걸린다. 재현성이 보장된 성능 수치나
Office 전체 탐색으로 설명하지 않는다.

## 조작과 녹화 화면

- **Pause**: simulator와 planner를 함께 멈춘다. 진행 중인 이륙/착륙 action은 먼저 끝난다.
- **Resume**: 같은 위치·시뮬레이션 시간에서 재개한다.
- **Stop & land**: planner를 중단하고 착륙한 뒤 campaign을 끝낸다. 충돌 후에는
  다시 비행시키지 않고 종료한다. 사용자 중단 결과는 성공/실패율에서 제외한다.
- 왼쪽: headless Isaac Sim의 실제 렌더링. **Distance / Height / Orbit**로 관찰
  카메라를 조절한다. 설정이 다음 프레임·비행에도 유지되며 모델의 센서 카메라는 바뀌지 않는다.
  Fixed camera에서는 추적을 멈춘다.
- 오른쪽: 선택 모델의 실제 RGB/depth 및 TSDF·경로 또는 D3QN action 추론 화면.
- 상단: 현재 조건과 다음 조건을 선택한 이유. 하단: 비행 결과와 취약성 분석 보고서.

별도 Isaac 창 없이 **웹 한 화면을 녹화**하면 된다. 두 패널이 같이 보이는 창 크기로
연다. 발표 영상은 조건 표시 → 실제 비행/추론 → 결과 → 다음 조건 선택 순서로
남기고, 로딩·착륙 대기 구간을 줄여 편집한다. 비행 사이의 영상에는 마지막
프레임이라는 표시가 나온다. 버튼 자체가 영상 파일을 만들지는 않는다.

## 환경은 무엇이 바뀌나

웹 **Difficulty**에서 추가 장애물 수를 선택한다. 기본값은 Easy다.

| 난이도 | 추가 화분 | 추가 기둥 |
|---|---:|---:|
| Easy | 1 | 1 |
| Medium | 3 | 3 |
| Hard | 5 | 5 |

각 난이도에 8seed씩 **24개 배치**가 있다. 건물 벽과 기본 가구는 유지하고,
기존 화분 하나의 위치도 바꾼다. 표의 개수는 Office의 원래 물체에 **추가하는 수**다.
같은 seed에서는 Easy의 물체 위치가 Medium/Hard에도 유지되고 물체가 더 추가된다.
책상·의자·물체 종류까지 무작위로 바꾸는 기능은 아직 포함하지 않는다.

난이도는 한 campaign 동안 고정하고, 결과에 따라 **같은 난이도의 다른 seed**를
선택한다. 공격 강도와 환경 난이도는 별도 설정이다. 웹에 추가 개수와 이전 배치
대비 물체 이동량 범위를 표시한다. 조명도 라운드 선택에 따라 바뀐다.

생성 시 물체 겹침·바닥 지지·기본 이륙/8m목표 영역을 검사하고, 지상 장애물의
범위를 이용해 고도1.2m에서 반경0.35m의 경로가 있는지 간단히 검사했다.
이는 비행 성공 보장이 아니며, 사용자 지정 목표 거리·기체 크기에 대한 보장도 아니다.
난이도 이름은 장애물 개수 기준이고 실제 모델의 어려움은 비행 결과로 평가한다.
기존 furnished_a/b 배치는 과거 결과 재현용으로 보존했다.

**clean/attack 쌍 안에서는 배치·조명·시작 위치가 같다.**
clean은 센서 noise/delay와 patch만 끈다. 공격 실패를 재확인할 때도 같은 배치를
유지하므로, 비행할 때마다 반드시 배치가 달라지는 것은 아니다.

## 다음 조건 선택과 분석

| 결과 | 다음 시험 |
|---|---|
| clean부터 실패 | 공격 효과로 단정하지 않고 다음 배치 |
| clean 성공 / attack 실패 | 같은 쌍을 한 번 더 실행해 재현 확인 |
| 재확인에서도 attack 실패 | 같은 배치에서 공격 강도를 절반으로 감소 |
| 둘 다 성공 / 최소 clearance < 0.35m | 같은 배치에서 강도를 조금 증가 |
| 둘 다 여유 있게 성공 | 다음 배치 + 강도 증가 |
| 실행 환경 오류 | 성능 평가에서 제외, 제한된 재시도 |

기본 Combined는 noise/delay/patch를 함께 바꾼다. 원인을 나눠 보려면 Attack에서
**Noise only / Delay only / Patch only**를 골라 별도 campaign을 실행한다.
자동으로 모든 단일 요인 재시험까지 수행하는 것은 아니다.

완료된 쌍마다, 그리고 campaign 완료/정지 시 **Vulnerability analysis → Open report**에
결과가 나온다. 실패 조건, clean 대비 metric 차이, 재현 횟수, 충돌 물체·종료 사유와
저장된 실행 경로를 기록한다. clean부터 실패하면 기본 주행의 한계로 구분한다.
Combined 실패만으로 특정 patch/noise가 원인이라고 단정하지 않는다.
예산을 모두 쓰면 추가 제안만 표시하고 더 실행하지 않는다.

LLM을 붙일 때는 이 결과/설정 이력을 읽고 다음 설정을 선택하는 부분을 대체한다.
검증된 bench가 비행·평가·증거 저장을 담당하고, 웹은 관찰과 중단 인터페이스로
남길 수 있다. 현재 LLM이나 자동 자연어 원인 추론은 연결돼 있지 않다.

## 저장 및 CLI

기본 웹 실행은 대용량 ROS bag 없이 설정, GT samples, metric, 로그, 최종 추론
프레임을 저장한다. 결과는 `robot/ros_ws/ws2_runtime/campaigns/<campaign>/` 아래의
`vulnerability_report.html/.md/.json`, `report.json`, `history.json`, `trials.csv`에 남는다.
중간에 정지하면 완료되지 않은 쌍은 취약성 판정에서 제외한다.

```bash
python3 tools/ws2_bench/campaign.py --backend feedback --profile combined --difficulty medium \
  --budget 8 --planner mononav --timeout 180 --goal-distance 8 \
  --output robot/ros_ws/ws2_runtime/campaigns/presentation_01
```

Kim은 `--planner kim --timeout 120`으로 새 output을 사용한다.
`--record-bags`를 추가하면 각 비행의 이륙 전부터 bag을 기록한다.
`--wait-for-recording`은 campaign 시작 전 한 번 Enter를 기다린다.
중단된 실행의 동일 명령·설정·output을 다시 주면 저장된 완료 시험을 재사용한다.
명시적으로 Stop한 시험을 새로 시험하려면 새 campaign을 시작한다.
예전 짧은 미션이나 두 모델 통합 feedback 이력은 새 설정으로 이어 쓰지 않는다.

발표 설명 예시:

> We select one model and run clean and attacked tests.
> The bench measures the flight results and chooses the next condition.
> The report shows failing conditions and whether they repeat.
> We can watch the simulation and model output on the same page.
> We will add an LLM to the test selection step later.

실제 검증 결과와 한계는 [VALIDATION.md](VALIDATION.md),
Git 백업 범위는 [BACKUP.md](BACKUP.md)를 참고한다.
