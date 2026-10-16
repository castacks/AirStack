# WS2 자동 시험 실행·녹화

## 공통 복도: clean·공격 비행과 자동 녹화

설치는 [README](README.md)를 따른다. AirStack 폴더에서 실행한다.
한 터미널에서 viewer를 시작하고 **http://127.0.0.1:8892**를 연다.

```bash
.venv-ws2/bin/python tools/ws2_bench/dashboard.py --port 8892
```

다른 터미널에서:

```bash
.venv-ws2/bin/python tools/ws2_bench/corridor_suite.py \
  --targets mononav kim pulp_dronet --tiers easy \
  --output robot/ros_ws/ws2_runtime/campaigns/corridor_recorded
```

세 모델이 L자/곡선/리을자/U자 각각에서 같은 길이·폭·장애물 배치를 사용한다.
`--targets`로 모델, `--shapes elbow curve zigzag u_obstacles`로 씬을 선택한다.
기본은 환경별 clean2회 확인 후 noise/delay, 필요한 경우 조합을 실행한다.
Kim은 장애물이 있는 곳에서 원본 FCRN patch도 시험한다. MonoNav/PULP patch는 금지.

Medium/hard는 먼저 `--tiers medium hard --phase clean`으로 확인하고, 같은 output에
`--phase attacks`로 이어간다. 기존 완료 비행을 다시 실행한 것으로 세지 않는다.
동일 조건 resume만 가능하며 config/코드/가중치 변경 시 새 output을 사용한다.
작은 clean 표본의 기준은 easy100%, medium75%, hard50% + 각 맵 최소1회 성공이다.
공격 비교는 두 clean을 통과한 맵에서만 수행한다. 일반적인 성공률 입증은 아니다.

녹화는 자동이다. **실제 takeoff 명령부터 착륙/충돌 종료까지**, 왼쪽 Isaac과 오른쪽
추론 화면을 각각960×540으로 저장한다. 바탕화면/다른 작업은 캡처하지 않는다.
`videos.html`에서 비행별 영상을 열고, `flights.csv`와 `qualification.json`에서 결과를 본다.
각 비행 폴더에는 `flight.mp4`, 설정·GT 지표·diagnostics·녹화 시각이 남는다.
새 trial부터 모델 화면이 초기화되며, 모델 로딩 구간은 그 사실을 표시한다.

설정 파일:

- `scenarios/corridor_suite.yaml`: 공통 맵/조명/noise/delay/patch size/관찰 카메라.
- `scenarios/corridor_mononav.yaml`, `corridor_kim.yaml`, `corridor_pulp.yaml`: 각 모델 설정.
- Suite의 공통 맵이 모델 YAML의 scene을 덮어쓴다. 모델별 속도/카메라는 유지한다.

한 번만 실행할 때는 `episode.py <YAML> --no-bag --camera follow --record-video`를 쓴다.
Bag이 필요하면 `--no-bag`를 빼며, 이 경우 이륙 전 지상부터 기록한다.
성공은 ordered gates와 끝 단면 완주이며,500sim s는 상한이다. Kim/PULP에는 goal/route를
주지 않고 MonoNav에만 명시적 task goals를 제공한다. 측정 결과는 해당 integration의 결과다.
웹 Start tests 버튼은 Office용이고, 복도 실행 중에는 웹에서 관찰/pause/resume/stop한다.

## Office 실행

AirStack 폴더에서 다음 명령으로 웹을 연다. 이미 켜져 있으면 브라우저 새로고침만 한다.

```bash
python3 tools/ws2_bench/dashboard.py
```

**http://127.0.0.1:8892** → Target / Difficulty 선택 → Flights / Observation 또는 Timeout 설정
→ 화면 녹화 시작 → **Start tests**.

한 campaign은 **선택한 모델 하나만** 시험한다. 기본 8비행은 4개의 clean/attack
쌍의 상한이다. Expanded에서 clean 실패 후 공격을 생략하면 실제 비행은 더 적다. 첫 쌍이 끝나면 결과를 보고 다음 조건을 골라 자동으로 반복한다.
다른 모델은 새 campaign에서 선택한다. 기본 **Basic feedback**은 LLM 인증 없이
기존 결과 기반 규칙을 사용한다.

Claude를 쓰려면 터미널에서 `claude auth login --claudeai`로 Team 계정에 로그인한 뒤,
Method를 **LLM-guided tests**, LLM을 **Claude · Team login**으로 고른다.
Conditions에서 **Avoidance scenes + sensors (Rui FCRN patch: Kim only)**를 고르면
새 장애물 좌표·회피 challenge·밀도·통로 폭·RGB 노이즈·조명·delay를 선택한다.
Expanded에서 Kim은 원본 Rui FCRN patch의 on/off·크기·시점도 선택한다. MonoNav patch는 차단한다.
Claude는 새 조건 탐색·동일 조건 반복·한 요인 분리·noise/delay 경계 확인 중 목적까지 선택한다.
실험마다 실패 직전 5초의 센서 age·모델 출력·명령·GT 움직임을 받아 다음 판단에 사용한다.
이전 saved 모드를 고를 때만 Clean-checked layouts에 확인한 배치를 입력한다.
Claude가 허용된 환경/센서 조건을 선택하고, 비행 종료 후 아래 **Claude interpretation**에
근거를 포함한 분석과 미검증 방어 후보가 나온다. 기존 metrics와 별도로 저장된다.
후보에는 근거·tradeoff·검증 계획이 붙는다. 실패가 없으면 후보를 꾸며내지 않는다.
LLM 요청·응답과 새 보고서는 모두 영어다. 좌표는 seed와 공간 제약으로 새로 생성하고
검증한 실제 좌표를 저장한다. **Open comparison**에서 방식별 비교 결과를 볼 수 있다.
시험 범위와 해석 기준은 [EVALUATION.md](EVALUATION.md)에 정리되어 있다.

한 쌍만 확인할 때는 **Flights=2 / Extra clean checks=0 / Infrastructure retries=0**.
추가 clean 확인과 인프라 재시도는 기본값이 각각 2/1이므로, 그대로 두면
Flights에 표시한 수보다 실제 실행 횟수가 늘어날 수 있다.
자세한 설정은 [CLAUDE_AGENT.md](CLAUDE_AGENT.md)를 참고한다.

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

아래 표는 **Basic feedback**의 규칙이다. Expanded Claude는 결과와 진단 자료를
읽고 실험 목적과 정확한 설정을 직접 고른다. `Continue` 설정에서도 expanded 모드는
clean 실패 후 공격을 생략한다. 남은 pair에서 다음 선택을 진행하며 비행 예산은 늘리지 않는다.
최종 보고서의 추가 시험 제안은 예산 종료 후의 향후 계획이며 자동으로 추가 실행하지 않는다.

| 결과 | 다음 시험 |
|---|---|
| clean부터 실패 | 공격 효과로 단정하지 않고 다음 배치 |
| clean 성공 / attack 실패 | 같은 쌍을 한 번 더 실행해 재현 확인 |
| 재확인에서도 attack 실패 | 같은 배치에서 공격 강도를 절반으로 감소 |
| 둘 다 성공 / 최소 clearance < 0.35m | 같은 배치에서 강도를 조금 증가 |
| 둘 다 여유 있게 성공 | 다음 배치 + 강도 증가 |
| 실행 환경 오류 | 성능 평가에서 제외, 제한된 재시도 |

기본 sensor profile은 noise/delay를 바꾼다. 원인을 나눠 보려면 Attack에서
**Noise only / Delay only**를 고른다. FCRN Patch only/Combined는 Kim에서만 가능하며
10월6일 센서 검증은 patch off였으며, 10월14일부터 Kim의 agent 검증에는 patch를 허용한다.
MonoNav patch 요청은 실행 전에 거절한다.

Patch는 **원본 이미지 그대로 on/off와 크기만** 바꾼다. 부착 높이는1.2m로
고정이며 contrast·투명도 조절은 없다. 크기가 커진다고 공격 효과가 반드시
커지는 것은 아니므로 결과로 확인한다. 이전 contrast 설정이 들어간 campaign은
보존하되 이어 실행하지 않고, **Start tests로 새 campaign**을 시작한다.
자동으로 모든 단일 요인 재시험까지 수행하는 것은 아니다.

완료된 쌍마다, 그리고 campaign 완료/정지 시 **Vulnerability analysis → Open report**에
결과가 나온다. 실패 조건, clean 대비 metric 차이, 재현 횟수, 충돌 물체·종료 사유와
저장된 실행 경로를 기록한다. clean부터 실패하면 기본 주행의 한계로 구분한다.
Combined 실패만으로 특정 patch/noise가 원인이라고 단정하지 않는다.
예산을 모두 쓰면 추가 제안만 표시하고 더 실행하지 않는다.

LLM을 붙일 때는 이 결과/설정 이력을 읽고 다음 설정을 선택하는 부분을 대체한다.
검증된 bench가 비행·평가·증거 저장을 담당하고, 웹은 관찰과 중단 인터페이스로
남긴다. 현재 Claude가 연결되어 있지만, 해석과 방어 제안은 측정값 및 검증된
인과관계와 구분한다. 복도 qualification 명령은 LLM을 호출하지 않는다.

## 저장 및 CLI

기본 웹 실행은 대용량 ROS bag 없이 설정, GT samples, metric, 로그, 최종 추론
프레임을 저장한다. 결과는 `robot/ros_ws/ws2_runtime/campaigns/<campaign>/` 아래의
`vulnerability_report.html/.md/.json`, `report.json`, `history.json`, `trials.csv`에 남는다.
중간에 정지하면 완료되지 않은 쌍은 취약성 판정에서 제외한다.

```bash
python3 tools/ws2_bench/campaign.py --backend feedback --profile sensors --difficulty medium \
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
> In agent mode, Claude selects the next test and interprets the evidence.

실제 검증 결과와 한계는 [VALIDATION.md](VALIDATION.md),
Git 백업 범위는 [BACKUP.md](BACKUP.md)를 참고한다.

현재 단일 요인 검증/새 회피 환경: [SENSOR_VALIDATION.md](SENSOR_VALIDATION.md).
새 비교 실행 전 검토할 평가/논문 계획: [AGENT_EVALUATION_PLAN.md](AGENT_EVALUATION_PLAN.md).
