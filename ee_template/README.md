# UR16e End-Effector Template (headless, config-driven)

UR16e는 **고정**, 엔드이펙터(그리퍼·F/T센서·스크루드라이버·툴체인저)는 **가변**.
새 툴 = **YAML config 하나**. 코드(`build_ee.py`)는 안 건드린다. 전 과정 GUI 없이 헤드리스.

## 실행 (2-게이트: 구조 → 거동)
```bash
# 게이트 1 — 리깅 + 구조 검증 (L1, pure pxr, Kit 부팅 없음)
PKG=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
PYTHONPATH=$PKG LD_LIBRARY_PATH=$PKG/bin /isaac-sim/python.sh \
    ee_template/build_ee.py ee_template/configs/<name>.yaml
# -> ee_template/out/ur16e_<name>_actuated.usd

# 게이트 2 — 헤드리스 거동 검증 (L2, Kit 부팅, GPU 필요). GUI Play 눈대중 대체.
/isaac-sim/python.sh ee_template/verify_sim.py ee_template/out/ur16e_<name>_actuated.usd
# 각 툴 조인트를 open->close 구동해 수렴/이동/무폭발을 assert. exit 0=PASS.
```

## 구조
```
ee_template/
  build_ee.py            드라이버(고정 코드). 스테이지: convert→attach→rig→구조검증
  verify_sim.py          헤드리스 거동 검증(L2/Kit): 조인트 개폐 구동 + assert
  configs/
    robotiq_2f85.yaml    ✅ 동작 레퍼런스(2지 revolute). kinematic 동일 + 헤드리스 sim PASS
    onrobot_dualtool.yaml 🔷 설계 타깃(Y툴체인저+2FG14+스크루드라이버). 스키마 계약 = 미구현
  modules/               (향후) 카탈로그 부품별 리깅 USD 라이브러리
  out/                   산출물
```

## config 스키마 (모듈 트리)
엔드이펙터를 **모듈 트리**로 기술한다. 모듈 하나 = 링크 하나(움직이거나 고정).
```yaml
name, arm_base, artic_root, mount_link, out
source: { rigged_usd | step , tool_frame }     # 이미 부착된 USD 재사용 or STEP부터
modules:
  - name, parent(=mount_link | 다른 모듈명)      # 트리 위상
    select: { by: centroid_x | assembly_node, ... } # 소스에서 이 모듈 솔리드 고르기
    joint:  { type: revolute | prismatic | fixed, axis_local, axis_sign, pivot_mm/origin_mm, limits/limits_mm }
    body:   { mass_kg, collider }
    drive:  { type, stiffness, damping, max_force, target }
    frame:  { semantic: ft_sensor | tcp }        # (옵션) ROS/제어용 프레임
```
- 어느 모듈도 안 고른 솔리드 → parent 지오메트리로 남음(예: 그리퍼 베이스→wrist_3_link).
- 관절축·피벗은 `tool_frame`(그리퍼 로컬, mm) 기준으로 기술 → 팔 자세와 무관하게 결정론적.

## 새 엔드이펙터 추가 절차
1. 툴 STEP을 규약대로 사전정렬(마운트면=원점, +Z=접근) — Step 2 규약과 동일.
2. `configs/<name>.yaml` 작성(위 스키마). 모듈마다 select·joint·drive 채움.
3. 드라이버 실행 → `out/ur16e_<name>_actuated.usd`.
4. 검증(현재=구조 assert / 향후=헤드리스 시뮬 개폐 assert).

## 현재 구현 상태 (골격)
| 기능 | 상태 |
|---|---|
| rig 스테이지(pure pxr): 링크·강체·콜라이더·조인트·드라이브 | ✅ |
| joint.type = revolute / prismatic / fixed | ✅ |
| select.by = centroid_x | ✅ |
| verify = 구조 검증(articulation root 1개, 중첩 강체 없음) | ✅ |
| **verify_sim = 헤드리스 거동**(개폐 수렴·이동·무폭발 assert; L2/Kit) | ✅ 2F-85 PASS |
| **select.by = assembly_node** (복합 다중툴 분할) | 🔲 미구현 — 듀얼툴 필수 |
| **source.step** (헤드리스 CAD 임포트+부착; L2/Kit) | 🔲 미구현 — 지금은 `rigged_usd` 재사용 |
| verify_sim 렌더 PNG 스냅샷(옵션 시각확인) | 🔲 미구현(numeric assert만) |
| frame.semantic → ROS 프레임/센서 노출 | 🔲 미구현(4b ROS 브리지에서) |

## 듀얼툴까지 남은 일 (우선순위)
1. `select.by=assembly_node` — STEP 어셈블리 서브트리 이름으로 분할(centroid보다 견고).
2. `source.step` — Kit 헤드리스 CAD 컨버터로 STEP→USD+tool0 부착(GUI Step 0·2 대체).
3. `verify` 헤드리스 시뮬 — 눈대중 튜닝을 자동 assert로.
