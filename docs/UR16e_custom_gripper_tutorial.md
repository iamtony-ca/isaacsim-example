# UR16e + 커스텀 그리퍼 Isaac Sim 튜토리얼 (재현 가능)

CAD(STEP)와 로봇 URDF에서 출발해, Isaac Sim에서 **커스텀 그리퍼가 달린 UR16e**를 움직이는
시뮬레이션 로봇으로 빌드업한다. 재현·확장 가능하도록 설계 — 다른 그리퍼/UR 모델로 바꿔도
동일한 절차가 적용된다.

- **Isaac Sim:** 5.1.0
- **시나리오:** 조립된 STEP(UR16e + 커스텀 그리퍼)이 있고, **UR16e 팔만** URDF가 있다(그리퍼는 없음).
  목표: 팔을 관절화하고, 커스텀 그리퍼를 플랜지에 리지드로 장착한 뒤, 마우스로 workspace를 탐색.
  (ROS 2 bridge는 이후 단계)
- **전략(방식 A):** 팔 = URDF(관절화), 그리퍼 = CAD 메시를 플랜지에 리지드 고정. UR16e 팔은 표준이라
  URDF 메시를 써도 손실이 없고, *커스텀*인 그리퍼만 CAD에서 가져온다.

---

## 0. 에셋 목록 (경로는 `test_ws/` 기준)

> 리포는 **린 구조**: CAD·생성 USD 바이너리는 커밋 안 됨(gitignore). `cad/`는 벤더 CAD를
> 받아 두는 곳(`cad/README.md`), `assets/`는 파이프라인이 생성하는 USD(`assets/README.md`).

| 파일 | 역할 |
|---|---|
| `cad/ur16e/UR16e.step` | UR16e 팔 CAD (Universal Robots 공식 STEP) |
| `cad/grippers/GRP-CPL-062.STEP` | Robotiq UR 커플링/어댑터 (UR ISO 9409-1-50-4-M6) |
| `cad/grippers/2f-85-robotiq/Step/2F85_Opened_20190924.STEP` | Robotiq 2F-85 그리퍼 CAD |
| `cad/assemblies/UR16e_2F85_assembly.step` | 전체 조립 CAD (팔+커플링+그리퍼) — *시각 참조용* |
| **`cad/gripper_2f85_tool.step`** | **커플링+그리퍼를 사전 정렬: 마운트면이 원점, +Z=접근축** — 플랜지 부착에 사용 |
| `robot/ur16e.urdf` | UR16e 단독 URDF, 6 revolute 관절, 절대 메시 경로(바로 임포트 가능) |
| `assets/ur16e_2f85.usd` | 조립 CAD의 정지 USD (관절 없음) — 형상/참조용 |
| `assets/ur16e_gripper_base.usd` | URDF 임포트한 관절 팔 (단계 1 산출물) |
| `assets/ur16e_2f85_rigged.usd` | 팔 + 그리퍼(리지드, tool0) — 헤드리스 레퍼런스 (단계 2) |
| `assets/ur16e_2f85_rigged_my.usd` | 〃 GUI로 직접 부착한 버전(CAD 솔리드 보존 → 4a에 사용) |
| `scripts/rig_gripper_fingers.py` | **단계 4a 재현 스크립트** (config 기반 손가락 리깅) |
| `assets/ur16e_2f85_actuated.usd` | 손가락 작동(revolute 2개) (단계 4a 산출물) |
| `ee_template/` | **헤드리스 템플릿 파이프라인** (§8): config 하나로 엔드이펙터 교체 |

> `gripper_2f85_tool.step`이 핵심 재사용 에셋: **마운트면이 (0,0,0), 그리퍼가 +Z로 뻗도록**
> 정렬돼 있어 UR `tool0` 프레임에 **identity(오프셋 0)** 로 얹힌다. 다른 그리퍼면 이 규약대로
> 다시 만들면 단계 2가 동일하게 재현된다.

---

## 1. 로드맵

| 단계 | 내용 | 산출물 |
|---|---|---|
| 0 | 조립 정지 CAD (바닥에 기립) | `ur16e_2f85.usd` |
| **1** | UR16e URDF 임포트 → **관절 팔** | `ur16e_gripper_base.usd` |
| **2** | 커스텀 그리퍼를 `tool0`에 리지드 부착 | `ur16e_2f85_rigged.usd` |
| 3 | 관절 드래그(마우스)로 workspace 검증 | — |
| **4a** | 2F-85 손가락 관절 추가 → 개폐 (**스크립트**) | `rig_gripper_fingers.py` → `ur16e_2f85_actuated.usd` |
| 4b | ROS 2 bridge 연동 | — |

> 위는 GUI를 섞은 학습용 경로. **전 과정 GUI 없이(코드베이스·헤드리스)** 돌리는 템플릿
> 파이프라인은 **§8** 참고 — 새 엔드이펙터 = YAML config 하나(`ee_template/`).

---

## 2. URDF 임포터 옵션 (이 경우 추천값)

목표 = **테이블 고정 팔로 workspace 탐색**(아직 접촉작업 없음).
메뉴: `File > Import…` → `/isaac-sim/test_ws/ur16e.urdf` 선택.

| 옵션 | 설정 | 이유 |
|---|---|---|
| **Model** | `Create In Stage` | 로봇 프림을 스테이지에 바로 생성 → 편집·그리퍼 부착이 쉬움. (`Reference`는 별파일 참조, 여러 씬에서 재사용할 때) |
| **└ Set as Default Prim** | **켬** | 임포트된 루트(`/ur16e`)를 스테이지의 `defaultPrim`으로 지정. 이 base USD는 이후 그리퍼 부착·재리깅에서 **참조 대상**이 되므로, defaultPrim이 있으면 `references=@ur16e_gripper_base.usd@`만으로(프림 경로 생략) 깔끔하게 붙는다. 끄면 참조 시 `</ur16e>` 경로를 매번 명시해야 하고 "No default prim" 경고가 뜬다. |
| **└ Clear Stage on Import** | **켬** (단, 빈 새 스테이지면 무관) | 임포트 직전 현재 스테이지를 통째로 비우고 로봇만 남긴다 → 단계 1 산출물이 항상 로봇 단독이라 재현성↑. ⚠️ **이미 바닥·조명·PhysicsScene 등을 세팅한 스테이지에 팔을 얹는 경우엔 반드시 끌 것**(안 그러면 세팅이 다 날아감). |
| **USD Output** | 기본 경로 | `Reference` 모드에서만 의미. |
| **Links → Base** | **`Static Base`** | UR은 테이블 고정 → 베이스를 월드에 고정(안 쓰러짐). (`Moveable Base`=자유부유) |
| **Default Density** | 기본(~1000) | fallback일 뿐. UR URDF에 관성값 있음. workspace엔 무관. |
| **Joints & Drives → Ignore Mimic** | 기본(끔) | 팔 단독엔 mimic 관절 없음. (그리퍼 임포트 때만 의미) |
| **Joint Config (Stiffness / Natural Frequency)** | 기본값 | 위치드라이브 게인. 기본이면 목표각 추종이 부드러움. 더 단단히 원하면 stiffness↑ |
| **Drive Type** | **`Acceleration`** | 질량 정규화 → 튜닝 쉽고 안정적. (`Force`=실토크, 고급) |
| **Collision From Visuals** | **끔** | UR URDF에 collision 메시(`.stl`)가 있음 → 그걸 사용. |
| **Collider Type** | **`Convex Hull`** | 가볍고 충분(접촉작업 아직 없음). 오목 정밀충돌은 나중에 `Convex Decomposition`. |
| **All Self-Collision** | **끔** | 켜면 초기 침투로 폭주·느려짐. workspace엔 불필요. 나중에 필요시 켬. |
| **Replace Cylinders with Capsules** | 기본 | UR은 메시 기반이라 영향 미미. |
| **Merge Fixed Joints** | **끔** ⚠️ | `flange`/`tool0`을 별도 프림으로 유지 → 그리퍼 부착 프레임(`tool0`)이 깔끔하게 남음. 켜면 `wrist_3_link`로 합쳐져 플랜지 오프셋을 수동 계산해야 함. |

**한 줄 요약:** `Create In Stage`(+ `Set as Default Prim` 켬 · `Clear Stage on Import` 켬) ·
`Static Base` · `Acceleration` · `Convex Hull` · Self-Collision **끔** ·
Collision-from-visuals **끔** · Merge-Fixed-Joints **끔**, 나머지 기본.

---

## 3. 단계 1 — 팔 관절화

1. `File > Import…` → `/isaac-sim/test_ws/ur16e.urdf`, 위 옵션으로 **Import**.
   - articulation 루트가 `/World/ur16e`(URDF `robot name="ur16e"`)로 생성되고,
     링크 `base_link … wrist_3_link`, `flange`, `tool0` 포함.
2. **Play (▶)**. Static Base + 위치드라이브라 자세 유지.
3. `Tools > Physics > Physics Inspector` (Isaac 5.x — `Window`에서 여기로 이동) → 로봇(`/World/ur16e`)
   선택 → Play 중 관절 `Target Position` 슬라이더 드래그:
   `shoulder_pan`(베이스 회전), `shoulder_lift`, `elbow`, `wrist_1/2/3`.
   `tool0` 프레임이 **도달 가능 workspace**를 그림.
   - `Tools`에 없으면 `Window > Extensions`에서 `omni.physx.supportui` 활성화, 또는
     `Window > Physics > Physics Authoring Toolbar`도 진입점.
4. **Stop (■)** 후 `File > Save As…` → `/isaac-sim/test_ws/ur16e_gripper_base.usd`.

✅ 체크포인트: 맨 팔이 마우스로 움직이고 자세를 유지.

---

## 4. 단계 2 — 커스텀 그리퍼를 `tool0`에 리지드 부착

그리퍼는 URDF가 없으므로 CAD 형상을 플랜지 링크에 고정한다.

### 4a. 사전 정렬된 그리퍼 툴을 USD로 변환
1. `File > Import…` → `/isaac-sim/test_ws/gripper_2f85_tool.step`
   (Isaac Sim CAD 변환기 `omni.kit.converter.cad` 사용; STEP은 내부적으로 HOOPS 컨버터를 탄다).
2. 일반 메시(비관절 Xform 트리)로 임포트 → `gripper_2f85_tool.usd`로 저장.
   - CAD는 **mm**, 변환기가 `unitsResolve` 스케일(0.001)을 붙여 미터 스테이지에서 올바른 크기로 보임. **이 스케일 유지.**

#### 임포트 옵션 (이 용도 권장값)

목표 = `tool0`에 붙일 **리지드 그리퍼 메시**로 가져오기. 각 항목의 내부 의미는 공식 HOOPS 컨버터 문서 기준.

| 옵션 (화면) | 내부 의미 | 권장 | 이유 |
|---|---|---|---|
| **Convert Visible Only** | Filter Style — 숨겨진 엔티티 제외(`eOmit`) | ✅ 켬 (기본) | 보이는 형상만 변환 → 숨은 보조 솔리드 제외, 메시 가벼움 |
| **Generate Projection UVs** | 소스에 UV 없으면 생성 | 기본(켬) 그대로 | 그리퍼는 텍스처 안 씀 → 무해, 켜든 끄든 결과 동일 |
| **Enable Instancing** | Instancing Style `eReference`(1) — 동일 솔리드 재사용 | ⚠️ **끔** | 켜면 일부 프림이 instanceable → **개별 편집이 막힘**. Step 2 Collider 추가, Step 4a base/좌/우 분할을 위해 편집 가능한 일반 메시로 둠 |
| **Convert Metadata** | PMI/메타데이터를 USD 속성으로 | ✅ 끔 (기본) | 치수·주석 메타데이터 불필요 |
| **Override Up-Axis** | `0`=File Default / Y-up / Z-up | File Default 그대로 | 강제 회전 금지. 에셋이 이미 "+Z=접근축"으로 정렬됨 → 유지해야 identity 부착 성립 |
| **Unit (Use Model Units)** | Meters Per Unit `0.0` = 소스 단위 유지 | ✅ **Use Model Units 유지** ⭐ | **가장 중요.** mm→`unitsResolve`(0.001)로 미터 스테이지에서 올바른 크기. 위의 "이 스케일 유지"가 곧 이 설정 |
| **Material Type** | Material Select `1`=USD Preview Surface | Preview Surface 그대로 | 범용 호환·형상 위주라 충분 (RTX 외관 원하면 OmniPBR+Preview) |
| **Scene Optimizer Config** | 최적화 JSON 경로/문자열 | 비움 | 고급 워크플로 전용 |

> **한 줄 요약:** **Enable Instancing만 끄고**(편집·분할 대비), **Unit=Use Model Units는 절대 유지**(mm→m 스케일 핵심),
> 나머지(Convert Visible Only 켬·Convert Metadata 끔·Override Up-Axis=File Default·Material Type=Preview Surface)는 기본 그대로.
> Destination Path=`/isaac-sim/test_ws/`, 파일명 `gripper_2f85_tool.usd`. (Tessellation LOD는 화면 밖, 기본 2로 충분; 곡면 더 매끈하려면 LOD↑ 재변환)
>
> 참고: 레퍼런스 `ur16e_2f85_rigged.usd`는 GUI가 아니라 OpenCASCADE 헤드리스로 생성됨 → GUI 임포트 옵션과 1:1로 묶이지 않음.
> 위 표는 "레퍼런스 재현"이 아니라 **GUI에서 편집·후속단계까지 매끄러운 설정** 기준(그래서 Instancing만 끔).
>
> 근거: [omni.kit.converter.cad 개요](https://docs.omniverse.nvidia.com/kit/docs/omni.kit.converter.cad/latest/Overview.html),
> [HOOPS 컨버터 옵션](https://docs.omniverse.nvidia.com/kit/docs/omni.kit.converter.hoops_core/latest/Overview.html)

### 4a′. (대안 시작점) 조립 STEP에서 그리퍼만 뽑아 재정렬

4a는 **사전 정렬된 그리퍼 STEP**(마운트면=원점, +Z=접근축)을 전제한다. 하지만 실무에선
**UR + 그리퍼가 한 덩어리로 조립된 STEP** 하나만 있고, 사전 정렬 툴이 없을 때가 많다. 이때
"그리퍼만 골라 저장 → tool0에 부착"을 GUI 드래그로 시도하면 반드시 막힌다:

- **원점 문제:** 조립 STEP에서 그리퍼만 `Save As`(선택 저장)하면 지오메트리 정점이 **조립 월드
  좌표**(= UR base가 원점)를 그대로 유지한다. 그리퍼 프림의 로컬 원점은 UR base에 남고 형상은
  플랜지 위치(수십 cm 밖)에 떠 있어, tool0에 identity로 붙이면 엉뚱한 곳에 놓인다.
- **단위 문제:** CAD 변환기가 미터 크기 지오메트리를 `metersPerUnit=0.01`(cm) 등으로 **잘못
  태깅**하기도 한다 → 미터 스테이지에 reference하면 100× 스케일 어긋남.
- **GUI duplicate+drag는 답이 아니다:** STEP 프림은 `xformOp:transform`(4×4 **행렬**)이라
  Property의 Scale 칸이 안 눌리고("Toggle offset mode"의 Scale 0은 편집 불가한 표시일 뿐,
  실제 0 아님), reparent는 부모의 단위/스케일 체인을 보존하지 않는다.

**해결 = 정점 리베이스(bake).** `ee_template/recenter_ee.py`(L1 순수 pxr)가 모든 메시의 정점을
새 프레임으로 구워 xformOp를 전부 identity로 만든다 → 선택한 앵커(마운트면)가 원점에 오고,
접근축을 +Z로 회전할 수 있어 **tool0에 identity로 붙는 사전 정렬 툴**이 만들어진다.

**절차:**

1. 조립 STEP을 임포트(단위는 model units 또는 meter). Stage에서 **그리퍼 서브트리만 선택** →
   `File > Save As…`(선택 저장) → 예: `robot/gripper.usd`.
2. **구조 파악**(접근축·마운트면 확인). 환경변수는 CLAUDE.md의 L1 extscache-pxr:
   ```bash
   PKG=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
   PYTHONPATH=$PKG LD_LIBRARY_PATH=$PKG/bin /isaac-sim/python.sh \
     ee_template/recenter_ee.py --in robot/gripper.usd --report
   ```
   출력의 `world AABB size (metres)`로 실제 크기(단위 오태깅 여부)를, per-mesh 센트로이드로
   **접근축**(손가락이 뻗는 축)과 **마운트면**(커플링 메시가 있는 반대쪽 최소면)을 확인.
3. **리베이스(bake).** 마운트면과 단위를 지정해 정렬된 USD 생성:
   ```bash
   PYTHONPATH=$PKG LD_LIBRARY_PATH=$PKG/bin /isaac-sim/python.sh \
     ee_template/recenter_ee.py --in robot/gripper.usd --out robot/gripper_aligned.usd \
     --to bbox-face --axis=-z --set-meters-per-unit 1.0
   ```

   | 옵션 | 의미 |
   |---|---|
   | `--to bbox-face --axis=±x/y/z` | 그 면(커플링면) 중심을 원점으로. ⚠️ 음수 축은 argparse 특성상 **`--axis=-z`**(등호) 필수 |
   | `--to point --point x,y,z` | 마운트 중심을 정확히 알 때(커플링 메시 센트로이드 등) 명시 지정 |
   | `--to ref --ref-usd U --ref-prim P` | **URDF의 tool0 프레임에 완전 정렬**(가장 정밀). 예: `--ref-usd assets/ur16e_gripper_base.usd --ref-prim /ur16e/wrist_3_link/flange/tool0` |
   | `--approach +y` | 그 소스축을 +Z(접근축)로 회전. 접근축이 이미 +Z면 생략 |
   | `--set-meters-per-unit 1.0` | 미터 크기 지오메트리를 cm/mm로 잘못 태깅한 경우 **좌표 스케일 없이** 재태깅 |

   내부 동작: 각 PointBased 프림의 정점(+법선)을 `world × F⁻¹`로 굽고 모든 xformOp를 identity화.
   AABB는 authored extent가 아닌 **실제 정점**에서 계산(CAD가 extent를 padding해 bbox-face 앵커가
   수 cm 밀리는 문제 회피).
4. **검증**: 출력에 다시 `--report`. 마운트면 `AABB min` = **0**, 접근축이 +Z로 뻗고, 크기가
   실제 그리퍼 값(m)이면 OK.
5. `robot/gripper_aligned.usd`를 **tool0 아래에 identity로 reference** → 이후 **4b**로 진행.

> **실제 예 (3-finger 그리퍼, 2026-07-05):** `robot/tn__OpenCASCADESTEPtranslator7912_
> YUCHSV0tFY0.usd` — `metersPerUnit`이 0.01로 오태깅, 지오메트리 ~0.25 m, 원점이 조립 UR
> base(Y≈1.15)에 있었음. report로 **접근축=+Z**(손가락이 Z로 상승), **커플링 메시(1856 pts)=
> 최소 Z면** 확인. `--to point --point 0.001,1.1435,0.2853 --set-meters-per-unit 1.0`으로 bake →
> `robot/gripper_3f_aligned.usd`: 마운트면 Z=0, 손가락 +Z로 0.217 m, X/Y 중심, mpu=1.0,
> 크기 0.233×0.132×0.217 m. tool0에 identity 부착 준비 완료.

### 4b. 플랜지 링크 아래로 페어런팅
1. **Stage** 패널에서 임포트된 `gripper_2f85_tool` Xform을 `/World/ur16e/wrist_3_link/flange/tool0`
   **자식으로** 드래그.
2. 그리퍼 Xform의 **로컬 트랜스폼을 identity**로: `translate=(0,0,0)`, `orient=identity`,
   `scale=1` — 단 **`unitsResolve`(0.001) 스케일은 그대로 둠**.
   - 에셋의 마운트면이 CAD 원점·+Z=접근축이고, `tool0`의 +Z가 플랜지 바깥을 향하므로,
     identity면 커플링이 플랜지에 밀착되고 그리퍼가 바깥을 향함. 수동 오프셋 불필요.

✅ **Transform 검증** (Property 패널이 아래와 같으면 정확):

| 항목 | 화면 값 | 요구값 | 판정 |
|---|---|---|---|
| Prim Path | `/ur16e/wrist_3_link/flange/tool0/gripper_2f85_tool` | tool0의 자식 | ✅ 페어런팅 정확 |
| Translate | (0, 0, 0) | identity | ✅ |
| Orient | (0, 0, 0) | identity | ✅ |
| Scale | (1, 1, 1) | identity | ✅ |
| Scale:unitsResolve | (0.001, 0.001, 0.001) | 그대로 유지 | ✅ 핵심 |

> 추가 트랜스폼(Translate/Orient/Scale)은 전부 identity, `unitsResolve`(0.001)만 보존하는 게 포인트.

### 4c. 링크의 일부로 만들기 (떨어지는 별도 강체가 아니라 리지드)
- 그리퍼 메시는 이제 `tool0` **링크**(articulation의 rigid body)의 자식 → 플랜지와 함께 강체로 움직임.
- 그리퍼에 **별도 `RigidBodyAPI`를 주지 말 것**(자유 강체가 되어 떨어짐).

**Collider는 지금 추가하지 말 것 (단계 4a 이후로 미룸):**
- 현재 목표(단계 3 = workspace 탐색)엔 충돌 불필요 — 시각 메시만으로 그리퍼 끝의 도달범위 확인 가능.
- **열린 2F-85 전체의 단일 Convex Hull은 나쁜 충돌 형상** — 손가락 사이 빈 공간까지 덩어리로 채워 실제로 비어있는 영역이 충돌체가 됨. 파지에 못 씀.
- 올바른 시점: **단계 4a에서 base/좌/우로 분할한 뒤** 각 손가락에 개별 Convex Hull → 형상에 맞는 정확한 충돌체.

✅ 체크포인트: Stage 트리에서 그리퍼가 `tool0` 아래; 뷰포트에서 플랜지에 바깥 방향으로 장착.

`File > Save As…` → `/isaac-sim/test_ws/ur16e_2f85_rigged.usd`.

### 4d. 헤드리스 대안 (레퍼런스 결과 생성에 사용)
GUI 대신, `gripper_2f85_tool.step`을 OpenCASCADE로 테셀레이션(미터 단위 삼각망)해
`/ur16e/wrist_3_link/flange/tool0/Gripper2F85`에 **identity**로 스크립트 부착할 수 있다.
동일한 `ur16e_2f85_rigged.usd`가 나온다. 검증된 배치: `tool0` 월드 `(0.838, 0.291, 0.061)`,
그리퍼 마운트면이 `tool0`에 일치, `tool0`의 +Z로 ~0.16 m 뻗음.

> **무해한 로드 경고:** `Unresolved reference … /visuals/base_link`. UR URDF는 베이스를
> `base_link`(질량 프레임)과 `base_link_inertia`(메시 보유)로 나눔. 베이스 비주얼은
> `/visuals/base_link_inertia`에 있어 로봇은 정상 렌더링됨. 무시해도 됨.

---

## 5. 단계 3 — 그리퍼 포함 workspace 검증

1. `ur16e_2f85_rigged.usd` 열기 → **Play (▶)** → `Tools > Physics > Physics Inspector`에서 팔 관절 드래그.
2. 그리퍼가 플랜지를 타고 움직이며 **그리퍼 끝**이 workspace를 스윕.
3. 점검: 모든 관절 동작에서 그리퍼가 플랜지에 고정 유지; 떨어지거나 폭주 없음(Self-Collision 끔).

---

## 6. 단계 4a — 2F-85 손가락 관절 (개폐) — *스크립트 방식 (재현 가능)*

리지드 부착된 그리퍼는 단일 메시 덩어리라 그대로는 못 움직인다. 개폐하려면 손가락을
**분리된 강체 링크 + revolute 관절**로 리깅해야 한다. **팔은 고정이고 그리퍼·카메라는
제품마다 바뀌므로**, 이 단계는 GUI 수작업이 아니라 **config 기반 스크립트**로 한다 —
공통 파이프라인은 고정, 그리퍼 고유값만 상단 `CONFIG`로 분리해 교체 시 그 블록만 갈아낀다.

> **왜 GUI가 아니라 스크립트인가:** 임포트된 그리퍼는 외부 `gripper_2f85_tool.usd`를
> **payload**로 물고 있어, 그 안 메시는 메인 스테이지에서 **reparent가 금지**된다
> ("Cannot move/rename **ancestral** prim"). 게다가 PhysX는 **RigidBody를 다른 RigidBody의
> 자식으로 두는 걸 금지**하므로, 손가락을 강체로 만들려면 `wrist_3_link` 바깥(`/ur16e` 직속)으로
> 빼내야 한다. 둘 다 스크립트(`stage.Flatten()` + namespace 편집)로는 결정론적으로 처리된다.

**스크립트:** `scripts/rig_gripper_fingers.py` · **산출물:** `assets/ur16e_2f85_actuated.usd`
(입력 = 단계 2 결과 `assets/ur16e_2f85_rigged_my.usd`)

### 6a. 그리퍼 솔리드 규명 (`_my` USD 실측, 그리퍼 로컬·mm)
- 개폐축 = **로컬 X**(손가락 ±X로 벌어짐), 대칭면 ≈ X=0, 접근축 = +Z.
- 솔리드 **11개 = base 1 + 좌 5 + 우 5**:
  - **base**(중앙, cX≈2.4, Z 7~101) → 별도 강체로 안 만들고 `wrist_3_link` 지오메트리로 남김.
  - **좌손가락**(cX≈ −29·−43·−50·−56·−60) 5솔리드 = 강체 1개로 lump.
  - **우손가락**(cX≈ +34·+48·+55·+61·+64) 5솔리드 = 강체 1개(좌의 거울상).
- knuckle 피벗 ≈ 손가락 어셈블리 하단 Z≈55~58mm, X≈±13mm. 회전축 = **그리퍼-로컬 Y**
  (핀이 X-Z 스윙면에 수직; 현재 팔 자세에선 월드 Z로 매핑).
- 실제 2F-85는 4절 링크(mimic 필요)지만 연습용 단순화: **손가락 1개 = 강체 1개 + revolute 1개**.

### 6b. 스크립트가 하는 일 (공통 파이프라인 — 고정)
1. `stage.Flatten()` — payload를 베이크해 메시를 로컬·편집가능으로.
2. 11솔리드를 그리퍼-로컬 centroid X로 분류(base는 그대로, 좌/우 손가락 솔리드만 추림).
3. 손가락 솔리드를 새 링크 `/ur16e/finger_left`·`/ur16e/finger_right`(팔 링크와 형제)로
   reparent하며 **월드 트랜스폼을 베이크**(위치 보존).
4. 각 손가락 링크에 `RigidBodyAPI` + `MassAPI`(0.1kg) + 솔리드별 **Convex Hull** 콜라이더.
5. revolute 2개 작성: `body0 = wrist_3_link`, `body1 = finger_*`, 축을 그리퍼-로컬 Y에
   정렬(localRot0/1 계산), 피벗을 **양 바디 프레임에서 같은 월드점**으로, limit·`DriveAPI` 부여.

### 6c. CONFIG (그리퍼 고유값 — 교체 시 여기만 수정)
| 키 | 의미 | 2F-85 값 |
|---|---|---|
| `fingers[].cx_min/max` | 솔리드→손가락 분류(로컬 centroid X) | 좌 `<-10`, 우 `>10` |
| `fingers[].pivot_mm` | knuckle 피벗(그리퍼 로컬, mm) | 좌 (−13,0,58) · 우 (13,0,58) |
| `fingers[].rot_axis_local` / `axis_sign` | 회전축·부호 | (0,1,0) · 좌 +1 / 우 −1 |
| `fingers[].limits_deg` | 개폐각 범위 | (0, 45) |
| `drive` | 위치드라이브 게인 | accel · stiff 2e4 · damp 2e3 |

### 6d. 실행 & 검증
```bash
PKG=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
PYTHONPATH=$PKG LD_LIBRARY_PATH=$PKG/bin /isaac-sim/python.sh scripts/rig_gripper_fingers.py
```
- 구조 검증(headless): 강체 10개(팔 8 + 손가락 2)·**중첩 강체 없음**·articulation root 보존·
  revolute 2개+드라이브 정상.
- ⚠️ **거동 검증(GUI 필수):** `assets/ur16e_2f85_actuated.usd` 열기 → **Play(▶)** →
  `Tools > Physics > Physics Inspector`에서 `finger_left_joint`/`finger_right_joint` Target
  드래그 → 자연스러운 개폐 확인. 피벗이 어긋나면 **`CONFIG.pivot_mm`만 고쳐 재실행**(결정론적).

> **피벗은 bbox 추정 1차값**이라 GUI Play로 확인·튜닝이 필요하다. 다른 그리퍼로 교체 시도
> 이 한 줄 루프(값 수정→재실행→Play)만 반복하면 된다.
> 입력 1개로 **동기 개폐**가 필요하면 한 손가락을 driver로, 나머지를 **Mimic Joint**로 묶는다(9장 확장).
> 새 그리퍼로 교체하는 전체 절차는 **7장**에 정리.

---

## 7. 새 그리퍼로 교체하기 (재현 절차)

이 파이프라인의 목표는 **"새 그리퍼 = CONFIG 블록만 수정"**이다. 공통 파이프라인(§6b)은
건드리지 않고, 그리퍼마다 다른 값만 갈아끼운다. 아래는 처음부터 끝까지 밟는 절차.

### 7a. 전제 — 그리퍼가 아티큘레이션에 편입되는 구조 이해
손가락 관절은 `body0 = wrist_3_link`로 잡혀 **`/ur16e` 아티큘레이션의 일부**가 된다.
그래서 GUI Play에서 **UR16e 로봇(아티큘레이션)을 선택**해 Physics Inspector로 관절을
드래그해야 움직인다 — 손가락 링크만 따로 선택하면 안 잡힌다. (교체 후에도 동일)

### 7b. 절차 (2단계)

**1단계 — 그리퍼를 "사전 정렬 tool STEP"으로 준비 (Step 2 규약 재사용)**
새 그리퍼 CAD를 `gripper_2f85_tool.step`과 **같은 규약**으로 만든다:
- 마운트면(플랜지 접촉면) = **원점 (0,0,0)**
- **+Z = 접근축** (그리퍼가 뻗는 방향)
- 손가락 벌어짐을 **한 축(권장 X)**에 정렬 → 뒤 분류(centroid)가 단순해짐

→ Step 2를 identity로 재현 → 산출물 `ur16e_<gripper>_rigged_my.usd` (Step 4a 입력).

**2단계 — `rig_gripper_fingers.py`의 CONFIG만 수정 → 재실행 → GUI Play**
파이프라인 코드(`main` 이하)는 **손대지 않는다.** 아래 표의 값만 새 그리퍼에 맞춘다.

| CONFIG 키 | 바꾸는 이유 | 새 그리퍼에서 정하는 법 |
|---|---|---|
| `src` / `out` | 입출력 파일 경로 | 새 그리퍼 파일명 |
| `gripper_root` | tool0 아래 그리퍼 프림 경로 | 임포트된 프림 이름 확인 |
| `solid_parent_tag` | 손가락 솔리드가 매달린 어셈블리 노드 이름 | USD 하이어라키에서 그리퍼 어셈블리 노드 찾기 |
| `fingers[].cx_min/max` | 솔리드→손가락 분류(로컬 centroid X) | §6a처럼 `_my` USD 실측 |
| `fingers[].pivot_mm` | knuckle 피벗(로컬, mm) | CAD 실측 1차값 → Play로 튜닝 |
| `fingers[].rot_axis_local`·`axis_sign` | 회전축·좌우 부호 | 개폐면에 수직인 축, 좌우 반대 부호 |
| `fingers[].limits_deg` | 개폐각 범위 | 그리퍼 스펙(완전 개폐 각) |
| `finger_mass_kg` / `drive` | 링크 질량·드라이브 게인 | 스펙 시트 · 힘/속도에 맞게 |

> **실측 팁:** `solid_parent_tag`와 `cx` 임계값은 §6a 방식으로 정한다 — 새 `_my` USD를 열어
> 각 솔리드의 그리퍼-로컬 centroid X를 뽑고(스크립트 `gcx` 함수와 동일 계산), 좌/우/베이스가
> 갈리는 지점을 임계값으로 잡는다.

### 7c. 그리퍼 유형별 난이도

| 유형 | 필요한 작업 | 코드 수정 |
|---|---|---|
| **2-손가락 각도/평행** (2F-85 류) | CONFIG의 `cx`·`pivot_mm`·`limits_deg`만 조정 | **0줄** — CONFIG만 |
| **3-손가락** (예: Robotiq 3-finger) | `fingers[]`에 3개 항목. X 하나로 못 나눠 분류를 각도(θ)/Y까지 확장 | 분류 함수 `gcx` 소폭 |
| **마디 있는 손가락(관절 ≥2)** | `fingers[]`를 "관절 1개" 단위로 일반화, 부모 링크를 상위 finger 링크로 | 링크·조인트 생성부 |
| **병진 평행 그리퍼** | `RevoluteJoint` → `PrismaticJoint` | 조인트 생성부(§6b-5) 분기 |

- **2-손가락 유형**이면: 1단계(STEP 정렬) → CONFIG 5~6개 값 → 재실행 → Play. 코드 0줄.
- **나머지 유형**은 CONFIG + 스크립트 소폭 확장이 필요하다. 현재 스크립트는 2-손가락 revolute에
  특화돼 있으니, 진짜 임의의 그리퍼까지 재현하려면 **분류 규칙·조인트 타입 일반화**가 다음 개선점.

### 7d. 교체 후 검증 (§6d와 동일 루프)
`ur16e_<gripper>_actuated.usd` 열기 → **Play(▶)** → UR16e 아티큘레이션 선택 →
Physics Inspector에서 새 손가락 관절 Target 드래그 → ① 개폐 방향(안쪽으로 닫히는지, 아니면
`axis_sign` 뒤집기) ② 피벗(힌지 위치) ③ 리밋(완전 개폐)을 확인 → 어긋나면 **CONFIG만
수정해 재실행**. 이 한 줄 루프(값 수정→재실행→Play)가 교체 절차의 전부다.

---

## 8. 헤드리스 템플릿 파이프라인 (GUI 없이)

§7은 "새 그리퍼 = CONFIG 수정"이지만 아직 GUI에 손이 간다(Step 0 기립·Step 2 부착·Play 확인).
**UR16e는 고정, 엔드이펙터는 계속 바뀐다**는 목표를 극대화하려면 전 과정을 **코드베이스·헤드리스**로
돌린다. 산출물: `test_ws/ee_template/` — 새 툴 = **YAML config 하나**, 드라이버 코드는 불변.

### 8a. 두 층위의 headless
| 층위 | 실행 | Kit 부팅 | 담당 |
|---|---|---|---|
| **L1 순수 pxr** | `python.sh` + extscache pxr | 없음(가벼움) | USD 구조 파악·편집, 물리 저작(강체/조인트/드라이브) |
| **L2 헤드리스 Kit** | `SimulationApp(headless=True)` | 있음(~6s, GPU) | CAD 임포트, URDF 임포트, **물리 시뮬**, ROS 2 |

구조 파악·편집은 전부 L1(GUI 0%). 시뮬·CAD 임포트만 L2(창 없는 런타임).

### 8b. config = 모듈 트리
엔드이펙터를 **모듈 트리**로 기술한다(모듈 1개 = 링크 1개, 움직이거나 고정). 단순 2지 그리퍼의
`fingers[]`를 일반화한 형태 — 다중 툴·prismatic/revolute 혼재·센서 프레임까지 담는다.
```yaml
name, arm_base, artic_root, mount_link, out
source: { rigged_usd | step , tool_frame }        # 부착된 USD 재사용 or STEP부터
modules:
  - name, parent(mount_link | 다른 모듈명)          # 트리 위상
    select: { by: centroid_x | assembly_node, ... } # 소스에서 이 모듈 솔리드 선택
    joint:  { type: revolute | prismatic | fixed, axis_local, axis_sign, pivot_mm/origin_mm, limits/limits_mm }
    body:   { mass_kg, collider }
    drive:  { type, stiffness, damping, max_force, target }
    frame:  { semantic: ft_sensor | tcp }           # (옵션) ROS/제어 프레임
```
- 어느 모듈도 안 고른 솔리드 → parent 지오메트리로 남음(예: 그리퍼 베이스→wrist_3_link).
- 관절축·피벗은 `tool_frame`(그리퍼 로컬·mm) 기준 → 팔 자세와 무관하게 결정론적.

### 8c. 2-게이트 실행
```bash
# 게이트1 — 리깅 + 구조검증 (L1, Kit 없음)
PKG=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
PYTHONPATH=$PKG LD_LIBRARY_PATH=$PKG/bin /isaac-sim/python.sh \
    ee_template/build_ee.py ee_template/configs/robotiq_2f85.yaml
#   → out/ur16e_2f85_actuated.usd  (구조 assert: articulation root 1개, 중첩 강체 없음)

# 게이트2 — 헤드리스 거동검증 (L2, Kit 부팅·GPU). GUI Play 눈대중 대체.
/isaac-sim/python.sh ee_template/verify_sim.py ee_template/out/ur16e_2f85_actuated.usd
#   → 각 툴 조인트 open→close 구동, 수렴(±3°)·이동(>1°)·베이스drift(<5cm) assert. exit 0=PASS.
```
2F-85 검증 통과 예: `finger_left/right 0→25° travel≈25°, base drift 0.0000m → PASS`.
즉 개폐가 실제로 되는지를 **수치로** 판정 → CI 가능, 재현 가능.

### 8d. 검증된 현황 · 남은 일
- ✅ rig(L1): 링크·강체·콜라이더·조인트·드라이브 / `joint.type` revolute·prismatic·fixed /
  `select.by=centroid_x`·`assembly_node` / 트리 위상 / `frame.semantic` / 구조검증.
  **2F-85 산출물이 §6 스크립트와 kinematic 동일**.
- ✅ `verify_sim.py`(L2): 조인트타입 인식(°/mm) 헤드리스 거동 assert — **2F-85 + 합성 듈툴 PASS**.
- ✅ **모듈형 다중 툴 일반화 검증**(합성): `ee_template/tests/make_dummy_dualtool.py`가 Y-툴체인저
  (QuickChanger→{Damper, HEX_A→2FG14 L/R, HEX_B→Screwdriver})를 만들고
  `configs/dummy_dualtool.yaml`로 fixed×4+prismatic×2+revolute×1+센서프레임을 build→verify.
- 🔲 `source.step`(헤드리스 CAD 임포트로 Step 0·2 대체) — 실제 OnRobot 듈툴 STEP이 준비되면
  `configs/onrobot_dualtool.yaml`에 연결(2FG14=prismatic / 스크루드라이버=revolute 스핀들).

> 설계 전환 요점: 복합 엔드이펙터는 "덩어리 STEP을 centroid로 분할"이 아니라 **카탈로그 부품을
> 각각 1회 리깅한 모듈 USD로 만들어 트리 config로 조합**하는 쪽이 재사용·확장에 유리. `ee_template/`이
> 그 골격이다. 상세는 `ee_template/README.md`.

---

## 9. 확장

- **다른 그리퍼/커플링:** §7의 절차대로. 같은 규약(마운트면=원점, +Z=접근축)으로
  `gripper_2f85_tool.step`을 다시 만들면 단계 2가 동일. 전 과정 무인화는 §8 템플릿.
- **다른 UR 모델:** `ur16e.urdf`를 `ur5e`/`ur10e` 등으로 교체. 플랜지 규격
  (ISO 9409-1-50-4-M6) 공통이라 동일 그리퍼 장착.
- **ROS 2 bridge:** 리깅된 로봇에 articulation/joint-state 퍼블리셔·서브스크라이버(OmniGraph) 추가.

## 10. 재현성 노트

- Isaac Sim **5.1.0**; UR16e URDF는
  `isaacsim.robot_motion.motion_generation/.../universal_robots/ur16e/ur16e.urdf`에서
  `package://ur_description/` → `file:///opt/ros/jazzy/share/ur_description/` 치환(메시 14개 전부 해결).
- 그리퍼 마운트 변환은 `gripper_2f85_tool.step`에 베이크됨(커플링 바닥→원점, +Z=접근축) →
  플랜지 부착이 `tool0` 아래 **identity** 결정론적 배치.
- 모든 STEP은 mm(AP214); Isaac CAD 변환기가 임포트 시 mm→m `unitsResolve` 스케일 적용.
