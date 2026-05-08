# Codebase Walkthrough

This document is the quickest way for a new teammate to learn how the firmware
fits together without reverse-engineering it from scratch.

## System Overview

The ESP32 firmware has three distinct concerns:

- **Real-time acquisition on Core 0**: sample force and IMU data every 10 ms.
- **Low-priority control on Core 0**: receive host commands, coordinate modes,
  and stay out of the way of acquisition.
- **Processing on Core 1**: filter signals, compute warnings/scores, and emit
  JSON telemetry.

The host GUI talks to the board over UART0. The firmware sends two kinds of
structured output:

- normal runtime JSON at 20 Hz
- proof-mode JSON snapshots for the multitasking dashboard

## Read This First

If you are new to the repo, read these files in this order:

1. [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c)
   - startup sequence
   - queue creation
   - task creation and core pinning
2. [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c)
   - 100 Hz timer wake path
   - ADC + IMU reads
   - handoff to Core 1
3. [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c)
   - UART0 command ingress
   - low-priority control-plane work
4. [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c)
   - filter / state / warning / score / telemetry pipeline
   - multitasking proof snapshot assembly
5. [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c)
   - blocking calibration stages and their outputs

## Runtime Data Flow

The main sensor path is:

`esp_timer -> s_timer_sem -> acquisition_task -> raw_q -> processing_task -> UART JSON`

More concretely:

- `acquisition_init()` configures the ADC, IMU UART, and periodic timer.
- `timer_isr_cb()` runs through the ESP-IDF `esp_timer` task every 10 ms.
- `acquisition_task()` wakes on the semaphore, samples FSR + IMU data, fills
  one `raw_sample_t`, and posts it to `raw_q`.
- `processing_task()` blocks on `raw_q`, then runs the full signal-processing
  and scoring pipeline for each sample.
- Every 5th sample, Core 1 emits JSON to the host at 20 Hz.

The queue between cores is intentionally the only inter-core data path for live
sensor samples.

## Control Flow

The host-control path is separate from the sensor-data path:

`UART0 event queue -> control_task -> processing control queue -> processing/calibration actions`

Important pieces:

- `uart_driver_install(..., &uart_event_q, ...)` in `main.c` creates a UART0
  event queue.
- `control_task()` blocks on that queue in normal mode.
- `poll_uart_once()` reads `UART_DATA` events and turns short text commands
  into firmware actions.
- `processing_submit_control()` moves control messages from Core 0 to Core 1
  so processing-owned state stays single-threaded.

Examples of control commands:

- `I` identify the board
- `S` stop / reset session state
- `E`, `M`, `H` difficulty modes
- `C1` through `C4` calibration steps
- `MT_ON`, `MT_OFF` multitasking proof mode

## Where Multitasking Happens

The class-definition multitasking story is on **Core 0**.

The real low-priority project task is `control_task`. It is not a fake proof
task. It exists because host commands and control coordination should not live
inside the hard-real-time acquisition loop.

The chain is:

`control_task -> esp_timer callback -> semaphore give -> acquisition_task -> control_task resumes`

That is why the project now qualifies as true same-core preemptive
multitasking:

- low-priority real project work is running
- higher-priority timer/acquisition work becomes ready
- the scheduler preempts the low-priority task
- the low-priority task resumes afterward

The proof dashboard in
[tests/multitask_proof.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/multitask_proof.py)
shows that chain as `CTRL -> T -> S -> W -> D -> CTRL`.

## Where Parallelism Happens

Parallelism is the **Core 0 / Core 1 split**, not the same thing as the
multitasking proof.

- **Core 0** owns acquisition and control.
- **Core 1** owns processing, calibration actions, normal telemetry, and proof
  snapshot assembly.

This split is used for timing isolation: variable-latency processing work on
Core 1 cannot directly delay the 10 ms acquisition deadline on Core 0.

## Interrupts and Async Sources

The three important asynchronous sources in this project are:

1. **`esp_timer` periodic trigger**
   - drives the 100 Hz acquisition cadence
2. **UART0 host command receive events**
   - the UART driver posts `UART_DATA` events for `control_task`
3. **UART2 IMU serial data arrival**
   - the BNO085 sends UART-RVC bytes asynchronously; `acquisition_task`
     reads and decodes the driver buffer each cycle

Important nuance:

- a `STOP` command is not a separate interrupt source
- it is one example of a UART0 host command

## DSP and Telemetry

### Butterworth filters

The filter helpers in
[main/filters.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c)
implement fixed-coefficient Butterworth biquad sections for the 100 Hz runtime:

- low-pass 10 Hz for FSR smoothing
- low-pass 12 Hz for IMU stability logic
- band-pass 6–12 Hz for tremor energy isolation
- a 5-sample backward difference for `dF/dt`

These are strong code highlights because the processing pipeline is not just
passing noisy raw data downstream. The scorer depends on cleaner signals and
frequency-shaped features.

### JSON telemetry

The normal JSON stream is intentionally reduced to 20 Hz, even though the
acquisition loop runs at 100 Hz. That is a systems decision, not just a
formatting choice:

- the host still gets live feedback
- UART bandwidth stays manageable
- Core 0 is protected from serial-output backpressure

Proof-mode snapshots use the same idea: Core 0 emits compact raw events, but
Core 1 assembles and serializes the human-readable dashboard packets.

## Calibration Guide

Calibration lives in
[main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c).

The important stages are:

- **C1**: still-hand baseline
  - gyro bias
  - neutral pose
  - no-motion tremor reference
- **C2**: no-pressure FSR baseline
  - per-finger mean / sigma
  - contact on/off thresholds
- **C3**: reference grip force
  - stable intentional grip
  - stored as `f_ref_open`
- **C4**: normal hand motion reference
  - broadband motion RMS
  - tremor-band ratio under normal motion
  - orientation excursion references

Calibration is invoked from the processing side, but it consumes the same
`raw_q` stream produced by acquisition.

## File-by-File Guide

### Firmware core

- [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c)
  - boot path, driver setup, queue creation, and task creation
- [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c)
  - Core 0 hardware init
  - timer callback
  - `acquisition_task()`
  - multitasking-proof raw event emission
- [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c)
  - UART0 event handling
  - command parsing and dispatch
  - proof-mode behavior for the real low-priority control task
- [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c)
  - `processing_task()`
  - control-message handling
  - filter/feature/warning/score pipeline
  - proof-cycle reassembly and snapshot emission
- [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c)
  - all user-guided calibration steps and progress telemetry

### Support modules

- [main/filters.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c)
  - Butterworth biquad and derivative primitives
- [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h)
  - shared structs, thresholds, GPIO mapping, and protocol constants
- [main/nvs_storage.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/nvs_storage.c)
  - calibration persistence and defaults
- [main/motor_control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/motor_control.c)
  - motor pulse abstraction
- [main/led_status.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/led_status.c)
  - simple status LED state machine

### Test / demo tools

- [tests/multitask_proof.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/multitask_proof.py)
  - enables `MT_ON`, reads proof snapshots, and renders the Core 0 proof
- [tests/dual_core_monitor.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/dual_core_monitor.py)
  - visualises dual-core runtime behavior from normal telemetry
- [tests/imu_test.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/imu_test.py)
  - validates IMU-related JSON fields
- [tests/fsr_test.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/fsr_test.py)
  - validates force-related JSON fields

## What To Modify For Common Tasks

- **Change acquisition timing or sensor reads**
  - start in `acquisition.c`
- **Add a new host command**
  - add command ingress in `control.c`
  - add command effect handling in `processing.c`
- **Change scoring or warnings**
  - work in `processing.c`
- **Change calibration behavior**
  - work in `calibration.c`
- **Change telemetry fields**
  - runtime JSON: `processing.c`
  - proof JSON: `processing.c` + `tests/multitask_proof.py`

## Final Mental Model

If you only remember one picture, remember this:

- Core 0 samples and controls
- Core 1 thinks and reports
- the queue is the clean boundary
- `control_task` is the real low-priority Core 0 task
- `acquisition_task` is the real-time Core 0 task that preempts it

That model is enough to orient yourself before diving into individual
functions.
