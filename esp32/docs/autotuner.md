# Autotuner (ESP32 dual motor)

Step-response autotuning is built in for both loops:
- Master (M1) position loop
- Slave (M2) sync-to-master loop

Tuning runs a guarded step, measures rise/settle/overshoot, suggests new PID, and can apply them live.

## Prereqs & safety
- Clear travel for the step: defaults are `TUNE_STEP_TARGET=800` pulses for M1 and `TUNE_STEP_TARGET_M2=600` (configurable in `config.h`).
- Motors start from zeroed/stopped; brakes disengage at the end of a run.
- Timeout: `TUNE_TIMEOUT` (default 7s). Settling requires `TUNE_SETTLE_DWELL` ms inside the band.
- While tuning runs, all commands are blocked except `TUNE_STOP`.

## Commands (Serial @115200)
- `TUNE_BOTH [m1_target] [m2_target] [APPLY]` — tunes M1 then M2 sequentially. Optional targets override defaults. `APPLY` applies both suggested gains.
- `TUNE_STEP [target] [APPLY]` — tune M1 only (position loop).
- `TUNE_STEP_M2 [target] [APPLY]` — tune M2 only (sync loop).
- `TUNE_STOP` — abort the current run immediately.

Notes:
- Targets are encoder pulses; omit to use defaults. Provide `0` explicitly if you want to force default.
- `APPLY` writes suggested gains into the running controller; otherwise they are only printed.
- Normal CSV logging is automatically paused during tuning to keep the console readable; it resumes after the run. The tune itself is buffered and dumped as a CSV block between `TUNE_LOG_BEGIN` / `TUNE_LOG_END` for saving to text/CSV.

## Typical flow
1) Open Serial Monitor/terminal at 115200.
2) Ensure the mechanism is at zero and can move the step distance.
3) Run `TUNE_BOTH APPLY` (recommended). For a dry run: `TUNE_BOTH`.
4) Watch serial output:
   - `TUNE_METRICS: step M1/M2` shows rise/settle/overshoot and steady error.
   - `TUNE_SUGGEST: M1/M2 kp=... ki=... kd=...` shows proposed gains (applied if `APPLY` given).
5) After completion, the system halts at the final position. Use `z` to re-zero if you plan another run.

## Defaults (config.h)
- `TUNE_STEP_TARGET`, `TUNE_STEP_TARGET_M2` — step sizes for M1/M2 tuning.
- `TUNE_SETTLE_DWELL` — ms inside band before declaring settled.
- `TUNE_TIMEOUT` — ms before abort.

## What the autotuner does
- Starts from zero, waits briefly for quiet (low RPM) then issues a step setpoint.
- Measures 10%/90% times, overshoot, settling, and steady-state error.
- Adjusts Kp/Ki (and modest Kd) heuristically; clamps to safe ranges.
- Applies gains if requested, halts, and (if `TUNE_BOTH`) immediately launches the M2 pass after M1 finishes.
- Logs samples during the tune and emits them as a CSV block for saving.

## Tips
- If M1 overshoots heavily, re-run M1 tuning before trusting M2 results.
- If travel is limited, reduce targets: `TUNE_BOTH 400 300 APPLY`.
- For a single-direction setup, keep targets positive to avoid direction reversals during tune.
