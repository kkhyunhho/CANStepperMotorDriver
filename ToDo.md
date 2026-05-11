# ToDo

## Task: Apply MIT Code Convention fixes to `mks_motor.py`

**Date**: 2026-04-09
**Target**: `mks_motor.py`
**Purpose**: Bring the file into compliance with MIT Code
Convention (per `CLAUDE.md` §1) by removing unexplained
abbreviations, eliminating magic numbers, and improving
Python version compatibility.

### 1. Abbreviation fixes (rename or document)

- [x] L209 `dlc` — keep name (CAN standard term) but add a
      comment block defining "Data Length Code" at first use.
- [x] L225 `pkt` → `packet`.
- [x] L391 `spd` → `speed_bytes` (in `home`).
- [x] L439 `acc` → `accel`; also `spd` → `speed` in `move_to`.
- [x] L125 `v` → `value_24bit` in `_int24_bytes`.
- [x] L362 `d` → `data` in `setup` loop.
- [x] L189 `data = list(data)` → `data_bytes = list(data)`
      to avoid parameter shadowing; update downstream uses.
- [x] L135 `_pct_to_speed` docstring: replace literal
      "0-3000 RPM" with reference to `max_speed_rpm` so the
      docstring stays in sync if the constant changes.

### 2. Magic number → named constants

Add new class-level constants and replace usages.
All new constants use `_` prefix per CLAUDE.md §1
(`_settle_mid_s` style = internal constant).

- [x] L67-69 FTDI setup: introduce
      `_ftdi_bitmode_mask = 0xFF`,
      `_ftdi_bitmode_async = 0x40`,
      `_ftdi_read_timeout_ms = 100`,
      `_ftdi_write_timeout_ms = 100`,
      `_ftdi_purge_rx_tx = 1 | 2`.
- [x] L255 `range(5)` → `_response_retry_count = 5`.
- [x] L256 `time.sleep(0.05)` → `_response_retry_delay_s = 0.05`.
- [x] L319, L334 `time.sleep(0.5)` →
      `_limit_recover_delay_s = 0.5`.
- [x] L322 `0.01` mm dummy offset →
      `_dummy_move_offset_mm = 0.01`.
- [x] L327, L412 `300` RPM dummy speed →
      `_dummy_move_speed_rpm = 300`.

### 2b. Existing constants → add `_` prefix

Verified via grep: no external code accesses these
constants (only `MKSMotor.open()` / `MKSMotor.main()` are
used externally). All are internal → rename with `_` prefix
to match CLAUDE.md §1 constant convention.

- [x] `mm_per_turn` → `_mm_per_turn`
- [x] `encoder_per_turn` → `_encoder_per_turn`
- [x] `max_speed_rpm` → `_max_speed_rpm`
- [x] `max_accel` → `_max_accel`
- [x] `max_coord` → `_max_coord` (note: currently unused;
      verify and either keep or delete)
- [x] `max_travel_mm` → `_max_travel_mm`
- [x] `max_wait_sec` → `_max_wait_sec`
- [x] `motion_cmds` → `_motion_cmds`
- [x] `motion_status` → `_motion_status`
- [x] `setting_status` → `_setting_status`
- [x] Update all `self.<name>` and `cls.<name>` /
      `MKSMotor.<name>` references in this file.

### 3. Python compatibility (PEP 701 multiline f-string)

- [x] L242-244 (`_send` TX log): extract the inner expression
      to a local variable `data_hex` so the f-string no longer
      relies on Python 3.12+ multiline f-string syntax.
- [x] L277-279 (`_send` RX log): same pattern — extract
      `status_label` before the print.

### 4. Verification

- [x] Re-read the file and visually confirm all changes apply
      cleanly and stay within the 80-column limit.
- [x] (Skipped: `ruff` not installed in this environment.
      User to run `ruff check` / `ruff format --check` locally
      before commit.)

---

## Task: Correct mm/turn value to 5 in code and docs

**Date**: 2026-05-11
**Target**: `mks_motor.py`, `README.md`
**Purpose**: Align the mechanical pitch constant with the
actual hardware spec (5 mm/turn ball screw).

- [x] `mks_motor.py`: `_mm_per_turn = 4` → `_mm_per_turn = 5`
- [x] `README.md`: Hardware table `3.75 mm/turn` → `5 mm/turn`

---

## Task: Label TX/RX log messages with motor CAN ID

**Date**: 2026-05-11
**Target**: `mks_motor.py`
**Purpose**: When two motors run concurrently, distinguish which
motor each log line belongs to by prefixing with the CAN ID.

- [x] `_send`: `[TX]` / `[RX]` → `[M{can_id:02X}][TX]` / `[M{can_id:02X}][RX]`
- [x] `_wait`: `[RX]` / `[LIMIT]` / `[ERROR]` 동일하게 수정

---

## Task: Create running_test_xz.py (Z sync + X independent)

**Date**: 2026-05-11
**Target**: `running_test_xz.py` (신규 파일)
**Purpose**: Z축 2개 모터를 threading으로 동기화하고,
X축 1개 모터를 순차적으로 제어하는 시퀀스 구현.

### 동작 순서
1. Z 모터 2개 setup + home (병렬)
2. X 모터 setup + home (순차)
3. Z 모터 2개 → 200mm 이동 (동기)
4. X 모터 → 목표 지점 이동
5. Z 모터 2개 → 목표 지점으로 하강 (동기)

### 포트 할당 (가정)
- Z motor A : port 0
- Z motor B : port 1
- X motor   : port 2

### 구현 방식
- Z 동기 이동은 `threading.Barrier(2)` + 헬퍼 함수로 반복 제거
- 이동 파라미터(mm, speed_pct, accel_pct)는 파일 상단 상수로 정의

- [ ] `running_test_xz.py` 작성
- [ ] GitHub 이슈 등록
- [ ] 커밋 & 푸시

---

### 5. Out of scope

- No behavioral changes; only renames, constant extraction,
  and f-string restructuring.
- The "Use complete sentences" comment nit on the USB2CAN
  field labels (L226-235) is intentionally **not** addressed
  because the existing manual reference at L223-224 already
  provides sufficient context.
