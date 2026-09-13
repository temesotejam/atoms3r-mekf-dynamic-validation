# First V46 dynamic validation

This is the required first hardware check for the V46 MEKF build. Do **not** begin with Autonomous Energy Control. The first run is a passive/manual-motion capture with motor output at 0 mA.

## Goal

Verify on the real AtomS3R/IMU installation that:

- the adopted MEKF angle has the intended static and dynamic sign,
- the online dynamic-beta Madgwick comparison is recorded at the same time,
- adaptive accelerometer rejection behaves plausibly,
- RWLOG v46 timing and LED video anchors are present,
- the passive path keeps all motor/current/pulse commands at zero.

Only after this run passes should the autonomous V7 motion experiment be attempted.

## Firmware

Use a successful GitHub Actions artifact from the `feature/v46-mekf-dynamic-validation` branch. The artifact contains:

- `bootloader.bin` at `0x0000`
- `partitions.bin` at `0x8000`
- `boot_app0.bin` at `0xe000`
- `firmware.bin` at `0x10000`
- `merged-firmware.bin`, which contains the same four images at those offsets and is written starting at `0x0000`
- `SHA256SUMS.txt`
- `FLASH_LAYOUT.txt`

The expected V46 attitude revision is:

`v46_mekf_adopted_dynamic_beta_compare_20260912`

## Before starting

1. Place the mechanism in a safe bench setup with the reaction wheel clear of hands, cables, and fixtures.
2. Power the system normally, but **do not press `Start autonomous energy control`** during this first validation.
3. Wait for the firmware/UI to reach a state in which a capture can be started.
4. Connect to the firmware access point and open `http://192.168.4.1/`.
5. Start the fixed-horizon video before starting the capture so the complete START/MID/END LED sequence is visible.

Current firmware AP settings are defined in `src/config.h`.

## Run 1: passive/manual-motion validation

1. Keep the mechanism still initially.
2. In the web UI press **Start passive capture**.
3. Observe the START LED synchronization pattern.
4. After the START signature, move/release the mechanism manually once so it performs a representative free rocking motion.
5. Do not press the autonomous-control button during the run.
6. Keep recording through the END LED signature.
7. Download the RWLOG from the web UI.

The passive capture is the motor-off validation path. Every sample in this first run must retain zero motor/current/pulse command fields.

## Convert the RWLOG

```powershell
python tools\convert_rwlog_to_csv.py <run>.rwlog --out converted_run
```

Use the generated `timeseries.csv` for the first checks.

## Mandatory checks

### 1. Passive path remained motor OFF

Across the full passive run verify that the command fields remain zero, including:

- `motor_cmd_mA == 0`
- `current_mA_setting == 0`
- `pulse_width_ms == 0`
- `pulse_active == 0`

A non-zero value is a stop condition for further V46 validation.

### 2. V46 estimator signals exist

Confirm the converted log contains valid samples for:

- `pitch_mekf_control_deg`
- `pitch_mekf_abs_deg`
- `pitch_madgwick_dynamic_abs_deg`
- `mekf_q_w/x/y/z`
- `mekf_bias_x/y/z_dps`
- `mekf_accel_confidence`
- `mekf_accel_residual_deg`
- `mekf_accel_mag_error_g`
- `imu_update_dt_us`
- `imu_sample_age_us`
- `mekf_accel_used`
- `attitude_filter_adopted`

`attitude_filter_adopted` must identify MEKF (`1`).

### 3. Sign and continuity

Use the known physical direction of the manual motion to confirm that MEKF pitch follows the historical detector convention. Check for unexpected sign reversal, jumps, resets, or discontinuities.

For video comparison use:

- `pitch_mekf_abs_deg`
- `pitch_madgwick_dynamic_abs_deg`

Do **not** use `pitch_mekf_control_deg` as the absolute video angle because it is run-relative/zero-subtracted for control.

### 4. Adaptive accelerometer handling

During clean gravity-dominated motion, `mekf_accel_confidence` should remain high and `mekf_accel_used` should normally be `1`. During clearly contaminated acceleration, confidence may fall and `mekf_accel_used` may become `0`.

The first run is not intended to tune rejection thresholds. It is intended to verify that the real hardware produces sane diagnostics and that rejection is not permanently stuck ON or OFF.

### 5. Timing

Check `imu_update_dt_us` and `imu_sample_age_us` for large gaps or pathological outliers. The configured IMU period is 5 ms; the log period is 20 ms.

### 6. Video synchronization

Confirm the RWLOG includes `led_state` and `sync_event_id` events corresponding to:

- START signature,
- first MID anchor at 2.5 s,
- later MID anchors every 5.0 s,
- END signature.

Construct the video/RWLOG time mapping from these anchors before measuring estimator error or lag from video.

## Pass condition for proceeding to Autonomous V7

Proceed only when all of the following are true:

- passive motor/current/pulse commands are all zero,
- MEKF has the correct physical sign and no unexplained discontinuity,
- dynamic-beta Madgwick comparison data is present,
- MEKF accel confidence/rejection signals are plausible,
- V46 timing fields are healthy enough for comparison,
- LED video anchors are recoverable,
- RWLOG conversion/CRC succeeds.

After this pass, the next experiment is the controlled Autonomous Energy Control V7 capture using the same V46 firmware, with MEKF as the adopted angle and dynamic-beta Madgwick as comparison-only.
