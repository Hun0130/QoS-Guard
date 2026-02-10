# Dependency Chain Analysis of ROS 2 DDS QoS Policies: From Lifecycle Tutorial to Static Verification
<p align="center">
  <img alt="ROS2 logo" src="https://img.shields.io/badge/ROS--2-Humble-blue?style=for-the-badge">
  <img alt="Fast DDS logo" src="https://img.shields.io/badge/Fast--DDS-2.6.9-brightgreen?style=for-the-badge">
</p>


## 📝 Paper Summary
ROS 2 is built on the Data Distribution Service (DDS) and leverages more than 20 Quality of Service (QoS) policies to control communication availability, reliability, and resource usage. However, in practice, users often lack clear guidance or pre-verification procedures for combining these policies, which frequently forces them into trial-and-error tuning or results in unexpected runtime failures.
To address this challenge, we decompose DDS publisher–subscriber communication into three phases—Discovery, Data Exchange, and Disassociation—and provide a tutorial-style explanation of how 16 key QoS policies operate at each stage. We also systematically analyze inter-policy dependencies, deriving a QoS Dependency Chain, and classify 40 common constraints into a set of Dependency-Violation Rules.
Building on this analysis, we developed the QoS Guard package, which enables offline verification of DDS XML profiles to detect potential conflicts before deployment. This allows users to safely configure QoS settings without needing to launch a ROS 2 session.
By offering both conceptual insights and a practical tool, this work helps ROS 2 users better understand and manage QoS policies, ultimately improving the reliability of robot communications and the efficiency of resource utilization.

## 💡 How to run it from the terminal

This tool can be run either as a **ROS 2 package** or as a **standalone Python script**. No ROS 2 runtime is required—the code uses only standard Python libraries.

### 모드

1. **XML 페어 모드**: pub.xml, sub.xml 두 파일 직접 지정
2. **패키지 모드**: ROS 2 패키지 경로 지정 → 경로 내 QoS XML 자동 스캔
3. **list 모드**: 패키지 경로 내 모든 XML 파일 목록 출력

### Arguments

- `pub.xml` / `sub.xml`: Writer/Reader QoS profile (XML 페어 모드)
- `package_path`: ROS 2 패키지 경로 (패키지 모드)
- `dds`: DDS 벤더 – `fast` | `cyclone` | `connext`
- `ros_version`: ROS 2 버전 – `humble` | `jazzy` | `kilted`
- `publish_period`: Writer's message interval (PP), e.g. `40ms` (선택, 기본 40ms)
- `rtt`: Estimated round-trip time (RTT), e.g. `50ms` (선택, 기본 50ms)

> ⚠️ XML 형식 판별은 사용자가 지정한 dds에 따라 달라집니다. 현재 규칙은 `fast`+`humble`만 구현되었습니다.

### 외부 XML 프로파일 (환경 변수)

Fast DDS 전용. 패키지 모드에서 `dds=fast`일 때, 다음 환경 변수로 지정된 **외부 XML 파일**도 함께 스캔합니다.

| 환경 변수 | 설명 |
| --- | --- |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | Fast DDS 기본 프로파일 XML 경로 |
| `RMW_FASTRTPS_CONFIG_FILE` | ROS 2 rmw_fastrtps 용 XML 설정 경로 |

외부 XML 경로는 사용자마다 다르므로, **이 환경 변수는 사용자가 직접 설정**해야 합니다. 설정되지 않으면 패키지 내부 XML만 스캔합니다.

```bash
# 실행 전 환경 변수 설정
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/default_profiles.xml
qos_guard /path/to/package fast humble

# 또는 명령어와 함께 한 번만 적용
FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/default_profiles.xml qos_guard /path/to/package fast humble
```

---

## 🔧 Install & Run

### Option A: ROS 2 패키지로 실행

```bash
# 1. Create a ROS 2 workspace (if needed)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone the repository
git clone --branch QosGuard_v3 https://github.com/QosGuard-Anonymous/qos-guard.github.io.git

# 3. Build the package
cd ~/ros2_ws
colcon build --packages-select qos_guard
source install/setup.bash

# 4. Run
ros2 run qos_guard qos_guard pub.xml sub.xml fast humble publish_period=40ms rtt=50ms
ros2 run qos_guard qos_guard /path/to/ros2_package fast humble
ros2 run qos_guard qos_guard list /home/hoon/navigation2   # XML 파일 목록
```

### Option B: Python으로 직접 실행

```bash
# 1. Clone the repository
git clone --branch QosGuard_v3 https://github.com/QosGuard-Anonymous/qos-guard.github.io.git
cd qos-guard.github.io/qos_guard

# 2. Run (Python 3.10+ required)
python3 -m qos_guard.qos_checker test_xml/pub.xml test_xml/sub.xml fast humble publish_period=40ms rtt=50ms
python3 -m qos_guard.qos_checker /path/to/ros2_package fast humble
python3 -m qos_guard.qos_checker . fast humble   # 현재 패키지 스캔
python3 -m qos_guard.qos_checker list /home/hoon/navigation2   # XML 목록
```

ROS 2가 설치되어 있지 않아도 Python만 있으면 실행 가능합니다.

---

## 📂 Project Structure

```
qos_guard/
├── qos_guard/                    # Python package
│   ├── __init__.py
│   ├── cli.py                    # CLI 인자 파싱
│   ├── xml_parser.py             # XML 파싱
│   ├── rules_fastdds_humble.py   # Fast DDS + Humble 전용 규칙 검사
│   ├── package_scanner.py        # ROS 2 패키지 QoS XML 스캐너
│   ├── output.py                 # 출력
│   └── qos_checker.py            # 메인 진입점
├── resource/
│   └── qos_guard
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── test_xml/
│   ├── pub.xml                   # Writer QoS profile
│   └── sub.xml                   # Reader QoS profile
├── package.xml
├── setup.cfg
└── setup.py
```

---

## 🧪 What It Does

This tool parses QoS settings such as:

- `ENTITY_FACTORY`,`PARTITION`,`USER_DATA`,`GROUP_DATA`,`TOPIC_DATA`,`RELIABILITY`,`DURABILITY`, `DEADLINE`, `LIVELINESS`, `HISTORY`, `RESOURCE_LIMITS`, `LIFESPAN`, `OWNERSHIP(+STRENGTH)`, `DESTINATION_ORDER`, `WRITER_DATA_LIFECYCLE` and `READER_DATA_LIFECYCLE`

It checks both Writer and Reader profiles against **40+ rules** and reports:

- 🔴 **Critical** : likely to cause message loss or communication failure
- 🟠 **Conditional** : may cause runtime issues in specific situations
- 🟣 **Incidental** : recommended for better robustness

---

## 📝 QoS-Guard: Fast DDS 프로파일 우선순위 가이드 (Humble ~ Kilted)

이 프로젝트는 **ROS 2 Humble, Jazzy, Kilted** 버전에서 Fast DDS(rmw_fastrtps)를 사용할 때 발생하는 QoS 설정 충돌을 방지하기 위해 다음의 우선순위 규칙을 준수합니다.

### 🔝 1. QoS 적용 골든 룰 (Hierarchy)

여러 곳에 QoS가 정의되어 있을 경우, 아래 순서에 따라 **가장 높은 번호의 설정이 하위 설정을 완전히 덮어씁니다 (Override).**

| 순위 | 설정 위치 | 매칭 방식 | 비고 |
| --- | --- | --- | --- |
| **1 (최우선)** | **ROS 2 소스 코드 (`rclcpp`)** | `rclcpp::QoS` (non-DEFAULT) | XML 설정을 완전히 무시함 |
| **2** | **XML: `<topic profile_name="...">`** | 토픽 이름 자동 매칭 | [강력 권장] 버전 무관 최우선 XML 레이어 |
| **3** | **XML: `<data_writer>` / `<data_reader>`** | 토픽 이름 자동 매칭 | Jazzy/Kilted 스타일 |
| **4** | **XML: `<publisher>` / `<subscriber>`** | 코드 내 명시적 이름 지정 | Humble 스타일 (인라인 `<topic>` 포함) |
| **5 (최하위)** | **XML: `is_default_profile="true"`** | Fallback (기본값) | 명시적 설정이 없을 때만 적용 |

---

### 💡 버전별 매칭 포인트

* **Humble:** 주로 4순위인 `<publisher profile_name="my_pub">` 방식을 사용하며, 이를 적용하려면 코드에서 `PublisherOptions`를 통해 이름을 매칭해야 합니다.
* **Jazzy/Kilted:** 3순위인 `<data_writer profile_name="/topic_name">` 방식이 도입되어 토픽 이름만으로도 자동 매칭이 가능해졌습니다.
* **Best Practice:** 버전에 관계없이 항상 승리하는 설정을 하려면 **2순위 (`<topic>`)**를 사용하십시오.

---

### 🔍 적용 결과 확인하기

설정한 QoS가 실제로 반영되었는지 확인하려면 다음 명령어를 사용하세요.

```bash
# 특정 토픽의 실제 적용된 QoS 상세 확인
ros2 topic echo /your_topic_name --qos-profile all
```

---

## 📎 패키지 모드: QoS 프로파일 매칭 규칙

패키지 모드(`qos_guard /path/to/pkg fast humble`)에서 pub/sub XML 파일을 **어떻게 쌍(pair)으로 묶는지** 정의합니다.

### 1. Base 이름 추출 (정규식)

`profile_name`에서 `_pub`, `_subscriber`, `_writer`, `_reader`, `_profile` 등의 suffix를 제거한 **순수 이름**을 base로 사용합니다.

| profile_name | base |
| --- | --- |
| `cmd_vel_pub` | `cmd_vel` |
| `cmd_vel_subscriber` | `cmd_vel` |
| `latency_publisher_profile` | `latency` |
| `datawriter_profile_example` | `datawriter_profile_example` |

→ `cmd_vel_pub`와 `cmd_vel_subscriber`는 **같은 base**로 인식되어 서로 매칭됩니다.

### 2. 매칭 규칙

| 조건 | 동작 |
| --- | --- |
| **base가 동일** | 해당 pub ↔ sub만 쌍으로 생성 |
| **profile_name 없음** | 모든 pub × sub 조합 생성 |
| **Wildcard 키워드** | `default`, `common`, `generic` 포함 시 → **모든 pub × sub 조합** 생성 |

### 3. Wildcard 키워드

`generic_qos_pub`, `default_profile_sub`, `common_publisher`처럼 base에 다음 키워드가 포함되면 **모든 조합과 매칭**됩니다.

- `default`
- `common`
- `generic`

이를 통해 공용 프로파일을 여러 토픽과 함께 검사할 수 있습니다.

### 4. 요약

```
1. profile_name에서 base 추출 (regex로 _pub, _sub 등 제거)
2. base에 default/common/generic 포함? → 모든 조합 (Wildcard)
3. 그 외: base가 같은 pub-sub만 매칭
4. profile_name 없음? → 모든 조합
```

---

## QoS Guard Rule

Based on the following rules, each profile is automatically validated.

Here is some rules used in validation:

| ID No. | Identifier | QoS Conflict Condition | Entity Scope | Depenency Type | Validation Stage |
| --- | --- | --- | --- | --- | --- |
| 1 | HIST ↔RESLIM | [HIST.kind = KEEP_LAST] ∧ [HIST.depth > RESLIM.max_samples_per_instance] | — | Critical | 1 |
| 2 | RESLIM↔RESLIM | [RESLIM.max_samples < RESLIM.max_samples_per_instance] | — | Critical | 1 |
| 3 | HIST→DESTORD | [DESTORD.kind = BY_SOURCE_TIMESTAMP] ∧ [HIST.kind = KEEP_LAST] ∧ [HIST.depth = 1] | DataReader | Conditional | 1 |
| 4 | RESLIM→DESTORD | [DESTORD.kind = BY_SOURCE_TIMESTAMP] ∧ [HIST.kind = KEEP_ALL] ∧ [RESLIM.max_samples_per_instance = 1] | DataReader | Conditional | 1 |
| 5 | RDLIFE→DURABL | [DURABL.kind ≥ TRANSIENT] ∧ [RDLIFE.autopurge_disposed_samples_delay = 0] | DataReader | Incidental | 1 |
| 6 | ENTFAC→DURABL | [DURABL.kind = VOLATILE] ∧ [ENTFAC.autoenable_created_entities = FALSE] | — | Incidental | 1 |
| 7 | PART→DURABL | [DURABL.kind ≥ TRANSIENT_LOCAL] ∧ [PARTITION ≠ Ø] | — | Incidental | 1 |
| 8 | PART→DEADLN | [DEADLN.period > 0] ∧ [PARTITION ≠ Ø] | — | Incidental | 1 |
| 9 | PART→LIVENS | [LIVENS.kind = MANUAL_BY_TOPIC] ∧ [PARTITION ≠ Ø] | DataReader | Incidental | 1 |
| 10 | OWNST→WDLIFE | [WDLIFE.autodispose_unregistered_instances = TRUE] ∧ [OWNST.kind = EXCLUSIVE] | DataWriter | Incidental | 1 |
| 11 | HIST→DURABL | [DURABL.kind ≥ TRANSIENT_LOCAL] ∧ [HIST.kind = KEEP_LAST] ∧ [HIST.depth < ⌈RTT ⁄ PP⌉ + 2] | DataWriter | Conditional | 1 |
| 12 | RESLIM→DURABL | IF DURABILITY.kind ≥ TRANSIENT_LOCAL:IF HISTORY.kind == KEEP_ALL:RESLIM.max_sampel/instacne < ⌈RTT / PP⌉ + 2 | DataWriter | Conditional | 1 |
| 13 | LFSPAN→DURABL | [HISTORY.kind == KEEP_LAST] ∧ [LFSPAN.duration < RTT] | DataWriter | Conditional | 1 |
| 14 | HIST ↔LFSPAN | DURABL.kind ≥ TRANSIENT_LOCAL] ∧ [LIFESPAN.duration > HISTORY.depth * PP] | DataWriter | Conditional | 1 |
| 15 | RESLIM↔LFSPAN | [HIST.kind = KEEP_ALL] ∧ [LFSPAN.duration > RESLIM.max_samples_per_instance × PP] | DataWriter | Conditional | 1 |
| 16 | DEADLN→OWNST | [OWNST.kind = EXCLUSIVE] ∧ [DEADLN.period = ∞] | DataReader | Conditional | 1 |
| 17 | LIVENS→OWNST | [OWNST.kind = EXCLUSIVE] ∧ [LIVENS.lease_duration = ∞] | DataReader | Conditional | 1 |
| 18 | LIVENS→RDLIFE | [RDLIFE.autopurge_nowriter_samples_delay > 0] ∧ [LIVENS.lease_duration = ∞] | DataReader | Conditional | 1 |
| 19 | PART↔PART | [DataWriter.PARTITION ∩ DataReader.PARTITION = Ø] | — | Critical | 2 |
| 20 | RELIAB↔RELIAB | [DataWriter.RELIAB.kind < DataReader.RELIAB.kind] | — | Critical | 2 |
| 21 | DURABL↔DURABL | [DataWriter.DURABL.kind < DataReader.DURABL.kind] | — | Critical | 2 |
| 22 | DEADLN↔DEADLN | [DataWriter.DEADLN.period > DataReader.DEADLN.period] | — | Critical | 2 |
| 23 | LIVENS↔LIVENS | [DataWriter.LIVENS.kind < DataReader.LIVENS.kind] ∨ [DataWriter.LIVENS.lease_duration > DataReader.LIVENS.lease_duration] | — | Critical | 2 |
| 24 | OWNST ↔OWNST | [DataWriter.OWNST.kind ≠ DataReader.OWNST.kind] | — | Critical | 2 |
| 25 | DESTORD↔DESTORD | [DataWriter.DESTORD.kind < DataReader.DESTORD.kind] | — | Critical | 2 |
| 26 | WDLIFE→RDLIFE | [WDLIFE.autodispose_unregistered_instances = FALSE] ∧ [RDLIFE.autopurge_disposed_samples_delay > 0] | — | Conditional | 2 |
| 27 | RELIAB→DURABL | [DURABL.kind ≥ TRANSIENT_LOCAL] ∧ [RELIAB.kind = BEST_EFFORT] | — | Critical | 3 |
| 28 | HIST→RELIAB | [RELIAB.kind = RELIABLE] ∧ [HIST.kind = KEEP_LAST] ∧ [HIST.depth < ⌈RTT ⁄ PP⌉ + 2] | DataWriter | Conditional | 3 |
| 29 | RESLIM→RELIAB | [RELIAB.kind = RELIABLE] ∧ [HIST.kind = KEEP_ALL] ∧ [RESLIM.max_samples_per_instance < ⌈RTT ⁄ PP⌉ + 2] | DataWriter | Conditional | 3 |
| 30 | LFSPAN→RELIAB | [RELIAB.kind = RELIABLE] ∧ [LFSPAN.duration < RTT] | DataWriter | Conditional | 3 |
| 31 | RELIAB→OWNST | [OWNST.kind = EXCLUSIVE] ∧ [RELIAB.kind = BEST_EFFORT] | — | Conditional | 3 |
| 32 | RELIAB→DEADLN | [DEADLN.period > 0] ∧ [RELIAB.kind = BEST_EFFORT] | — | Conditional | 3 |
| 33 | LIVENS→DEADLN | [DEADLN.period > 0] ∧ [LIVENS.lease_duration < DEADLN.period] | DataReader | Conditional | 3 |
| 34 | RELIAB→LIVENS | [LIVENS.kind = MANUAL_BY_TOPIC] ∧ [RELIAB.kind = BEST_EFFORT] | — | Conditional | 3 |
| 35 | DEADLN→OWNST | [OWNST.kind = EXCLUSIVE] ∧ [DEADLN.period < 2 × PP] | DataReader | Conditional | 3 |
| 36 | LIVENS→OWNST | [OWNST.kind = EXCLUSIVE] ∧ [LIVENS.lease_duration < 2 × PP] | DataReader | Conditional | 3 |
| 37 | RELIAB→WDLIFE | [WDLIFE.autodispose_unregistered_instances = TRUE] ∧ [RELIAB.kind = BEST_EFFORT] | DataWriter | Conditional | 3 |
| 38 | HIST→DURABL | [DURABL.kind ≥ TRANSIENT_LOCAL] ∧ [HIST.kind = KEEP_LAST] ∧ [HIST.depth > ⌈RTT ⁄ PP⌉ + 2] | DataWriter | Incidental | 3 |
| 39 | RESLIM→DURABL | [DURABL.kind ≥ TRANSIENT_LOCAL] ∧ [HIST.kind = KEEP_ALL] ∧ [RESLIM.max_samples_per_instance > ⌈RTT ⁄ PP⌉ + 2] | DataWriter | Incidental | 3 |
| 40 | DURABL→DEADLN | [DEADLN.period > 0] ∧ [DURABL.kind ≥ TRANSIENT_LOCAL] | — | Incidental | 3 |
| 41 | LFSPAN→DEADLN | [LFSPAN.duration < DEADLN.period] | — | Critical | 1 |


---  

## 🖥️ Example Output

Below is an example of how the checker output looks in the terminal:
<img src="qos_guard_example.png" width="600" height="400"/>

> 🔴 Red: Critical  🟠 Orange: Conditional  🟣 Purple: Incidental



## 📢 Notice
This project is currently compatible with ROS 2 Humble using Fast DDS 2.6.9.
Support for other DDS vendors such as Cyclone DDS and OpenDDS is planned in future updates.

### 추후 업데이트 예정
- **latencyBudget** (LATENCY_BUDGET, OMG 표준 QoS): Jazzy/Kilted에서 추가된 파싱 및 규칙 검사 지원

### Contact & Collaboration
If you have any issues or questions about using this tool, please feel free to contact us anytime.

**Email**: [leesh2913@dgist.ac.kr](mailto:leesh2913@dgist.ac.kr)  
**Homepage**: [hun0130.github.io](https://hun0130.github.io/)

Research collaborations and industry-academia partnerships are also welcome!


