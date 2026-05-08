# Thesis Defense Prep

This document is a professor-facing technical defense guide for the current
project implementation. It is not a general tutorial. The goal is to help you
explain what was designed, why it was designed that way, where it exists in the
code, and how to defend those decisions under questioning.

Two evidence labels are used throughout:

- **Implementation fact**: directly supported by the current code or current repo docs
- **Engineering rationale**: a design argument inferred from the requirements,
  runtime constraints, and implementation tradeoffs

## Critical Correction Up Front

The firmware samples at **100 Hz**, which means a **10 millisecond period** per
sample. It does **not** sample at 10 Hz.

- **Implementation fact**: the periodic timer is started with
  `esp_timer_start_periodic(..., 10000)`, where `10000` is microseconds, or
  10 ms, in [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:288).
- **Why this matters**: if you say "10 Hz" in a defense, you are describing a
  100 ms period system, which is not what the code does.

## 1. Purpose and Defense Posture

This project is best defended as a real-time, multitasking, dual-core,
signal-processing system for surgical-skill coaching.

The firmware is not just reading sensors. It is deliberately partitioned into:

- a periodic Core 0 acquisition path
- a low-priority Core 0 control path
- a Core 1 processing and telemetry path

The host side is also part of the architecture, not just decoration. The Python
GUI and Python proof tools are how the design is validated live.

- **Implementation fact**: the host GUI consumes live JSON telemetry and drives
  calibration and control in
  [gui/esp32_controller.py](/Users/shanthanu/uart_echo_VitalSignsLab4/gui/esp32_controller.py:4).
- **Implementation fact**: the repo also includes targeted Python diagnostic
  tools for FSR, IMU, dual-core timing, and multitasking proof in
  [tests/fsr_test.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/fsr_test.py:3),
  [tests/imu_test.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/imu_test.py:3),
  [tests/dual_core_monitor.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/dual_core_monitor.py:254),
  and [tests/multitask_proof.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/multitask_proof.py:3).

**What to say**

This is a real embedded system with a time-critical sampling path, a lower-priority
control path, a separate processing core, and a host-side tooling layer for
telemetry, calibration, and proof.

**What proves it**

The task split, queue boundary, timer source, processing pipeline, and host
tools all exist in the repo as first-class components.

**What they may challenge**

- **Challenge: "Is this just a sensor demo?"**  
  **Answer:** No. The firmware has a timer-driven acquisition path, a separate
  low-priority control path, a queue-based inter-core boundary, and a Core 1
  processing pipeline. That is systems design, not just sensor reading.
- **Challenge: "Is the GUI superficial?"**  
  **Answer:** No. The GUI is a real protocol consumer that drives calibration and
  control, and the repo also includes Python proof and diagnostic tools built on
  the same telemetry/control boundary.
- **Challenge: "Where is the actual systems design?"**  
  **Answer:** In the timer/semaphore wake path, the Core 0 to Core 1 queue
  handoff, the task pinning and priorities, and the host-side protocol tooling.
  Those are first-class architectural decisions in the codebase.

## 2. System Overview

The cleanest end-to-end story is:

`esp_timer -> s_timer_sem -> acquisition_task -> raw_q -> processing_task -> UART JSON -> Python GUI`

- **Implementation fact**: that overall architecture is reflected in
  [README.md](/Users/shanthanu/uart_echo_VitalSignsLab4/README.md:131) and
  [CODEBASE_WALKTHROUGH.md](/Users/shanthanu/uart_echo_VitalSignsLab4/CODEBASE_WALKTHROUGH.md:43).
- **Implementation fact**: the inter-core queue is created in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:59).
- **Implementation fact**: task pinning is established in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:113).

### Core responsibilities

- **Core 0**
  - timer-driven acquisition
  - UART0 control ingress
  - multitasking proof source
- **Core 1**
  - filtering
  - state logic
  - scoring
  - JSON telemetry
  - proof snapshot assembly

- **Implementation fact**: Core 0 and Core 1 roles are explained directly in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:121) and
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:828).

**What to say**

The project deliberately separates "sample on time" from "think about the
sample." Core 0 owns timing-critical acquisition. Core 1 owns variable-latency
processing and output.

**What proves it**

Queue creation, core pinning, and the producer-consumer comments are explicit in
`main.c`, `acquisition.c`, and `processing.c`.

**What they may challenge**

- **Challenge: "Why split across cores at all?"**  
  **Answer:** Because acquisition has a fixed 10 ms cadence, while processing and
  telemetry have more variable latency. The split gives timing isolation so
  heavier Core 1 work does not directly compete with the acquisition deadline.
- **Challenge: "Why use a queue instead of globals?"**  
  **Answer:** Because the queue creates a clean producer-consumer boundary.
  Core 0 produces complete `raw_sample_t` packets, and Core 1 consumes copies of
  them without ad hoc shared mutable state.

## 3. Code Callout Map

| Concept | Code anchor | Why it matters |
|---|---|---|
| Real-time timing source | [main/acquisition.c:288](</Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:288>) | Starts the periodic 10 ms timer that defines the acquisition cadence |
| Acquisition wake path | [main/acquisition.c:329](</Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:329>) | Shows the task blocking on the semaphore until the timer releases it |
| Core 0 sample handoff | [main/acquisition.c:383](</Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:383>) | Shows the queue transfer from Core 0 to Core 1 |
| Low-priority control task | [main/control.c:163](</Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:163>) | Real project-owned low-priority task used in the multitasking proof |
| Inter-core queue | [main/main.c:74](</Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:74>) | Clean producer-consumer boundary between cores |
| Task pinning | [main/main.c:144](</Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:144>) | Shows acquisition on Core 0, control on Core 0, processing on Core 1 |
| Filter primitives | [main/filters.c:15](</Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c:15>) | Real IIR biquad implementation, not a black-box pipeline |
| Filter application | [main/processing.c:870](</Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:870>) | Shows where filtering is actually applied in runtime |
| Proof snapshot path | [main/processing.c:302](</Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:302>) | Shows Core 1 serializing the multitasking proof |
| Runtime JSON serialization | [main/processing.c:1098](</Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1098>) | Shows 20 Hz telemetry throttling and packet assembly |

**What to say**

If someone asks for "where in the code," start with this map. It gets you to
the right file and line quickly.

**What proves it**

Each entry points at the current implementation, not just comments in prose.

**What they may challenge**

- **Challenge: "Are these comments, or is there real logic there?"**  
  **Answer:** There is real logic there. Each anchor points to an active runtime
  path such as timer setup, semaphore blocking, queue transfer, filter execution,
  proof packet assembly, or JSON serialization.

## 3.1 Important Module-by-Module Callouts

If a professor asks you to move beyond the architecture diagram and point at the
actual implementation boundaries, these are the modules to call out first.

### `main/main.c` — system assembly

- queue creation:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:59)
- UART0 driver install:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:82)
- processing/control module init:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:109)
- task pinning and priority decisions:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:113)

**Why this module matters**  
This is the system-composition file. It shows that the architecture is not just
conceptual; the queue, UART driver, module init, core pinning, and priority
choices are all wired together here.

### `main/acquisition.c` — real-time acquisition

- timer callback:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:137)
- IMU packet reader:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:165)
- timer creation and 100 Hz start:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:288)
- acquisition loop:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:307)
- queue handoff to Core 1:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:383)

**Why this module matters**  
This is the real-time heart of the system. If someone questions your sampling
rate, timer model, or what exactly happens every 10 ms, this is the file.

### `main/control.c` — low-priority Core 0 control plane

- role statement:
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:15)
- command dispatch:
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:50)
- UART event consumption:
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:133)
- task body and proof-mode behavior:
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:163)

**Why this module matters**  
This file proves that the low-priority task in the multitasking proof is real
project work, not an artificial spinner inserted just for the demo.

### `main/processing.c` — filtering, state, scoring, telemetry

- processing task loop:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:828)
- FSR filtering:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:870)
- IMU filtering and tremor band-pass:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:878)
- feature extraction:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:918)
- state detection and engagement gate:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:954)
- warning/error logic:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1011)
- running score:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1089)
- runtime JSON serialization:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1098)

**Why this module matters**  
This file is where raw data becomes interpretable state. If someone asks where
the actual “skill trainer” logic lives, this is the strongest answer.

### `main/filters.c` and `main/filters.h` — DSP primitives

- Direct Form II Transposed biquad:
  [main/filters.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c:15)
- 12 Hz low-pass:
  [main/filters.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c:26)
- 6-12 Hz band-pass:
  [main/filters.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c:47)
- 10 Hz low-pass:
  [main/filters.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c:80)
- filter interfaces and intended use:
  [main/filters.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.h:27)

**Why this module matters**  
This proves the DSP is real implementation work, not library magic or raw-data
thresholding.

### `main/calibration.c` — personalization logic

- C1 still-hand baseline:
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1039)
- C2 no-pressure/contact threshold learning:
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1136)
- C3 grip reference:
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1217)
- C4 normal motion reference:
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1253)
- independent step dispatcher:
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1302)

**Why this module matters**  
This file shows that calibration is not cosmetic. It computes the user-specific
references that later shape runtime thresholds and interpretation.

### `main/data_types.h` — shared runtime contract

- inter-core sample struct:
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:30)
- persisted calibration struct:
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:67)
- force, tremor, hold, and score constants:
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:109)
- warning/error bitmasks:
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:199)

**Why this module matters**  
This is the best place to show how the project encodes its assumptions:
sample shape, calibration state, thresholds, difficulty constants, and bitmasks.

### `gui/esp32_controller.py` — host runtime consumer

- file-level purpose:
  [gui/esp32_controller.py](/Users/shanthanu/uart_echo_VitalSignsLab4/gui/esp32_controller.py:1)
- JSON routing:
  [gui/esp32_controller.py](/Users/shanthanu/uart_echo_VitalSignsLab4/gui/esp32_controller.py:2319)
- high-rate telemetry handling:
  [gui/esp32_controller.py](/Users/shanthanu/uart_echo_VitalSignsLab4/gui/esp32_controller.py:2334)

**Why this module matters**  
This shows that the GUI is a real protocol consumer and validation surface, not
just a pretty shell around the firmware.

### `tests/multitask_proof.py` and `tests/dual_core_monitor.py` — live proof tools

- proof-mode activation and packet handling:
  [tests/multitask_proof.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/multitask_proof.py:237)
- proof dashboard rendering:
  [tests/multitask_proof.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/multitask_proof.py:340)
- dual-core timing monitor:
  [tests/dual_core_monitor.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/dual_core_monitor.py:254)

**Why this module matters**  
These tools are the bridge between the firmware design claims and a live demo.
They are part of the technical validation story.

## 4. Real-Time Design

### What real-time means here

Real-time does not mean "fast." It means a repeated job has to complete within a
defined time interval. In this project, that repeated job is sensor acquisition
at 100 Hz.

- **Implementation fact**: the repeated job is the acquisition path, and it is
  timer-driven in [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:303).

### How 100 Hz is implemented

The timer is configured with:

- callback = `timer_isr_cb`
- dispatch method = `ESP_TIMER_TASK`
- period = `10000` microseconds

- **Implementation fact**: see
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:288).

The callback gives a semaphore:

- **Implementation fact**: `xSemaphoreGive(s_timer_sem)` in
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:147).

The acquisition task waits for that semaphore:

- **Implementation fact**: `xSemaphoreTake(s_timer_sem, portMAX_DELAY)` in
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:329).

### What the deadline is

The acquisition path must keep up with one full sample cycle every 10 ms:

- read 3 FSR channels
- read or attempt to read an IMU packet
- package `raw_sample_t`
- post it to the queue

- **Implementation fact**: the sampling pipeline is visible in
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:336).

### How the repo proves the timing claim

1. **Timer period in code**
   - [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:297)
2. **Measured rate log in firmware**
   - [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:389)
3. **External proof pin toggle**
   - [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:145)
4. **Host timing monitor**
   - [tests/dual_core_monitor.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/dual_core_monitor.py:254)
5. **Runtime JSON includes `actual_hz`**
   - [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1117)

### Important distinction: 100 Hz processing vs 20 Hz telemetry

The firmware processes every sample at 100 Hz, but it only emits JSON every 5th
sample.

- **Implementation fact**: JSON output every 5th sample appears in
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1098).
- **Implementation fact**: this is also documented in
  [README.md](/Users/shanthanu/uart_echo_VitalSignsLab4/README.md:875).

- **Engineering rationale**: serial telemetry is for human/GUI consumption, not
  the real-time control deadline. Decimating telemetry protects bandwidth and
  keeps serial output from dominating runtime behavior.

**What to say**

The real-time job is periodic acquisition at 100 Hz. The timer creates the 10 ms
cadence, the semaphore wakes the acquisition task, and the task completes one
sample cycle per tick.

**What proves it**

The timer period, semaphore path, measured frequency log, proof pin, and timing
monitor all exist in the codebase.

**What they may challenge**

- **Challenge: "How do you know it is 100 Hz and not just approximately 100 Hz?"**  
  **Answer:** Because the timer is explicitly started at 10,000 microseconds per
  period, which is 10 ms, and the repo also includes measured-rate logging, a
  proof pin, a host timing monitor, and an `actual_hz` telemetry field.
- **Challenge: "Is the 20 Hz JSON stream your real sample rate?"**  
  **Answer:** No. The internal acquisition and processing cadence is 100 Hz. The
  20 Hz stream is decimated host telemetry produced every 5th sample.
- **Challenge: "What exactly is the deadline?"**  
  **Answer:** The deadline is one full acquisition cycle per 10 ms timer tick:
  ADC reads, IMU read attempt, sample packaging, and queue handoff all have to
  stay within that schedule.

## 5. Interrupts and Asynchronous Sources

The three important asynchronous sources in the current implementation are:

1. `esp_timer`
2. UART0 RX events from the host
3. UART2 IMU serial packet arrival/read path

These are best defended as **asynchronous event sources** or
**interrupt-driven paths**. Do **not** overstate this as "three explicit
application ISRs we wrote ourselves," because the current implementation also
relies on ESP-IDF timer and UART driver infrastructure.

### `esp_timer`

- **Implementation fact**: `timer_isr_cb()` is the periodic timing source in
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:137).

### UART0 host command events

- **Implementation fact**: the UART driver installs an event queue in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:98).
- **Implementation fact**: `control_task` consumes those events in
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:133).

### UART2 IMU path

- **Implementation fact**: the IMU UART is configured in
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:271).
- **Implementation fact**: `rvc_read_packet(...)` drains and decodes bytes from
  the driver buffer in
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:165).

### Why "stop" is not a separate interrupt

- **Implementation fact**: stop is one command meaning handled by the UART0
  command path, not a separate hardware event source.

### Why the IMU path still counts as asynchronous

- **Engineering rationale**: the IMU generates serial bytes independently of the
  main application flow. The firmware then consumes that arrival stream from the
  UART driver buffer during each acquisition cycle. It is asynchronous at the
  peripheral/driver level even though the application-side read is scheduled.

### Plain-English explanation

If you need to explain this to someone who has never used an IMU, say it this
way:

- the IMU is the motion sensor
- UART2 is the separate serial connection used to listen to that sensor
- the IMU sends motion information on that line whenever it is ready
- the firmware then picks up that data and turns it into usable movement values

That is also why the IMU path is different from the FSR path: the IMU pushes
serial data toward the ESP32, while the FSR sensors are simply analog inputs
that the firmware samples on schedule.

**What to say**

The project has one periodic timing source and two serial asynchronous sources:
the timer, UART0 host commands, and UART2 IMU traffic.

**What proves it**

Timer setup, UART0 event queue installation, and IMU UART reads are all visible
in the code.

**What they may challenge**

- **Challenge: "Is a stop command an interrupt?"**  
  **Answer:** No. `STOP` is one meaning of UART0 data arrival. The asynchronous
  source is the UART receive event itself; the command meaning is interpreted
  afterward.
- **Challenge: "If UART2 is read in the acquisition task, why do you call it asynchronous?"**  
  **Answer:** Because the IMU still produces serial bytes independently of the
  application flow. The application-side read is scheduled, but the
  peripheral-side data arrival is asynchronous.

## 6. Multitasking Design

### What multitasking means here

In class terms, multitasking is one CPU apparently doing multiple tasks by
switching among them. The real class-definition proof in this project happens on
**Core 0**, not across both cores.

### Why `control_task` exists

`control_task` is real low-priority project work:

- UART command ingress
- mode changes
- calibration control
- proof-mode coordination

- **Implementation fact**: the control role is documented in
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:22).
- **Implementation fact**: the task body starts in
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:163).

### The same-core preemption chain

The proof chain is:

`control_task -> timer callback -> semaphore give -> acquisition_task -> control_task resumes`

- **Implementation fact**: timer event capture starts in
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:140).
- **Implementation fact**: `control_task` stays runnable in proof mode in
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:168).
- **Implementation fact**: acquisition emits wake and done markers in
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:330)
  and [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:400).

### Why this is genuine

- **Implementation fact**: `control_task` is a real project task, not a fake
  benchmark loop.
- **Implementation fact**: the proof dashboard consumes firmware-assembled proof
  snapshots in [tests/multitask_proof.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/multitask_proof.py:262).
- **Engineering rationale**: because the low-priority task already exists for
  real control-plane reasons, showing it preempted by higher-priority
  timer/acquisition work is a real multitasking proof, not a staged demo.

### How the proof is kept honest

Raw proof events are captured on Core 0, but the expensive proof packet assembly
is done on Core 1.

- **Implementation fact**: Core 1 drains raw proof events in
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:344).
- **Implementation fact**: proof snapshots are serialized from Core 1 in
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:302).

**What to say**

Multitasking is demonstrated on Core 0 by showing a real low-priority task being
interrupted by higher-priority timer and acquisition work, then resuming.

**What proves it**

The firmware records the event chain, and the host proof dashboard displays
`CTRL -> T -> S -> W -> D -> CTRL` from live instrumentation.

**What they may challenge**

- **Challenge: "Is `control_task` a fake task made only for the demo?"**  
  **Answer:** No. It is the real low-priority Core 0 control-plane task that
  handles UART ingress, mode changes, calibration coordination, and proof-mode
  control.
- **Challenge: "How do you know the proof tool is not inventing the timeline?"**  
  **Answer:** Because the host dashboard consumes proof snapshots assembled from
  raw firmware event timestamps on Core 1. It is displaying instrumented firmware
  events, not guessing from host-side packet timing.

## 7. Dual-Core / Parallel Architecture

### Multitasking is not the same as parallelism

Multitasking proof is the same-core preemption story on Core 0.
Parallelism is the fact that the ESP32 has two physical cores, and this project
uses both.

### Where the split is defined

- **Implementation fact**: task pinning is done with
  `xTaskCreatePinnedToCore(...)` in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:144).
- **Implementation fact**: the queue boundary is created in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:74).
- **Implementation fact**: the shared work unit is `raw_sample_t` in
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:30).

### Why this split was chosen

- **Engineering rationale**: acquisition has a strict periodic deadline
- **Engineering rationale**: processing, telemetry, and calibration have more
  variable runtime
- **Engineering rationale**: putting processing on a different core gives timing
  isolation and reduces the chance that serial formatting or heavier feature
  extraction disturbs the 10 ms sampling cadence

### Why one queue is good design

- **Implementation fact**: the queue is the only live inter-core sample channel,
  explicitly documented in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:61).
- **Engineering rationale**: a single producer-consumer boundary makes ownership
  and timing easier to reason about than cross-core shared mutable state.

**What to say**

Core 0 owns sensing and control. Core 1 owns interpretation and output. The queue
is the only inter-core sample boundary, and that is intentional for timing
isolation and clean ownership.

**What proves it**

Task pinning, queue creation, and blocking receive on Core 1 are explicit in the
code.

**What they may challenge**

- **Challenge: "Why not do everything on one core?"**  
  **Answer:** Because one-core designs mix time-critical sampling with
  variable-latency processing, calibration, and telemetry. The dual-core split
  gives the acquisition path timing isolation.
- **Challenge: "Why not share globals instead of copying samples through a queue?"**  
  **Answer:** Because the queue makes ownership and synchronization explicit. It
  is cleaner and easier to reason about than shared cross-core mutable state.

## 8. FreeRTOS and Why It Was Used

### What FreeRTOS is in this project

FreeRTOS provides:

- tasks
- priorities
- queues
- semaphores
- blocking waits

- **Implementation fact**: all major modules include FreeRTOS headers, such as
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:1),
  [main/acquisition.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.h:4),
  and [main/processing.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.h:3).

### How it helps here

1. **Periodic wakeup model**
   - timer gives semaphore
   - acquisition task blocks until release
2. **Inter-core communication**
   - queue copies `raw_sample_t` by value
3. **Priority separation**
   - low-priority control
   - higher-priority acquisition
4. **Event-driven processing**
   - Core 1 blocks on queue rather than busy-waiting

- **Implementation fact**: queue copy semantics are explained in
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:43).
- **Implementation fact**: processing blocks on queue receive in
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:862).

### Why not one monolithic loop

- **Engineering rationale**: a monolithic polling loop would mix time-critical
  acquisition with variable-latency processing, command handling, calibration,
  and serial output
- **Engineering rationale**: FreeRTOS gives explicit structure to timing,
  ownership, and blocking behavior

**What to say**

FreeRTOS is what makes the design clean: periodic wakeups, safe queue handoff,
priority-based scheduling, and blocking tasks instead of ad hoc polling.

**What proves it**

The timer/semaphore/queue/task architecture is everywhere in the current code.

**What they may challenge**

- **Challenge: "Could you have done this without FreeRTOS?"**  
  **Answer:** In principle yes, but the structure would be much weaker. FreeRTOS
  is what makes the timer/semaphore wake model, safe queue handoff, priority
  separation, and blocking behavior explicit instead of ad hoc.
- **Challenge: "What did the RTOS actually buy you?"**  
  **Answer:** Deterministic wakeups, explicit priorities, safe inter-core
  communication, and tasks that block when idle instead of polling constantly.

## 8.1 Task Priorities and Why They Were Chosen

Task priority is a design choice about urgency, not just a number assignment. In
FreeRTOS, if two tasks on the same core are ready to run, the higher-priority
task runs first. So priorities are how this project decides which work must win
when timing matters.

- **Implementation fact**: the task creation block and priority rationale are in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:113).

### The actual priorities in this project

- ESP-IDF `esp_timer` dispatch task (framework-owned): priority `22`
- `acquisition_task`: priority `10`
- `processing_task`: priority `9`
- `control_task`: priority `1`

- **Implementation fact**: the code comments in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:121)
  explicitly describe those choices.

Important clarification:

- the application does **not** assign priority 22 itself
- what the application chooses is `.dispatch_method = ESP_TIMER_TASK` in
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:291)
- ESP-IDF then runs the callback in its own higher-priority timer service task

### What these numbers mean

The numbers themselves are less important than the ordering:

1. the timer-dispatch context must be above the application tasks
2. acquisition must be above low-priority control work on Core 0
3. processing must be high enough to keep up, but it does not share Core 0
4. control should be low because it is not deadline-critical

So the meaning is not "10 is magically correct." The meaning is:

- `22 > 10 > 1` on Core 0, so timing and acquisition beat control
- `9` on Core 1 keeps processing responsive without interfering with Core 0 timing

### Why `esp_timer` is above everything

The 100 Hz timing source has to be reliable. If the timer-dispatch context were
not above the normal project tasks, then even the release event that starts each
sampling cycle could be delayed.

- **Implementation fact**: `main.c` explicitly notes that `acquisition_task` is set
  to priority 10 so the `esp_timer` task at priority 22 can still preempt it in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:125).

- **Engineering rationale**: the timing source should outrank the work it triggers.

### Why `acquisition_task` is priority 10

`acquisition_task` is the highest-priority project-owned task on Core 0 because it
owns the real-time sampling deadline. When the timer releases the semaphore, this
task needs to run before ordinary project work like UART command handling.

- **Implementation fact**: `acquisition_task` is created at priority 10 in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:144).

### Why 10 instead of 5?

The defensible answer is: because the exact number is not the main point; the
ordering is.

What mattered in the design was:

- it had to be **well above** the low-priority control task
- it had to remain **below** the `esp_timer` dispatch context
- it needed to leave room for clear scheduling hierarchy instead of crowding the bottom of the priority range

So `10` was chosen as a clear high project priority, not because 10 has some unique
physical meaning. A value like `5` could also work **if** the ordering and behavior
stayed correct. But `10` makes the hierarchy obvious and leaves less ambiguity:

- timer source at 22
- acquisition clearly high at 10
- processing also high at 9
- control clearly low at 1

- **Engineering rationale**: choosing 10 instead of 5 is about making the intended
  scheduling relationship explicit and giving comfortable separation from the
  lowest-priority control-plane work.

### Why `processing_task` is priority 9

`processing_task` is computationally important, but it lives on Core 1. It needs to
be high enough to keep up with the incoming queue stream, but it does not need to
outrank the timing source on Core 0.

- **Implementation fact**: `processing_task` is created at priority 9 in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:160).
- **Implementation fact**: `main.c` explicitly notes that relative priority between
  acquisition and processing does not create preemption between them because they
  are on different cores in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:138).

### Why 9 instead of 10 or 1?

It was placed just below acquisition to show that:

- acquisition is the highest-priority project task
- processing is also important, but secondary to the Core 0 sampling path
- control is clearly much lower

A priority of `1` for processing would be too weak if Core 1 ever had competing work,
while making it equal to or above acquisition would blur the intended design story.
So `9` makes the hierarchy easy to explain: high, but not the highest project task.

### Why `control_task` is priority 1

`control_task` is intentionally low because it is real work, but it is not
time-critical in the same way as acquisition. It handles UART ingress, mode control,
calibration commands, and proof coordination.

- **Implementation fact**: `control_task` is created at priority 1 in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:152).
- **Implementation fact**: `main.c` says this keeps it below acquisition so
  timer/acquisition work can preempt it in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:129).

### Why 1 instead of 2 or 5?

Again, the exact number is less important than the meaning. `1` makes the intent
unmistakable: this is background control-plane work, and it should lose to the
real-time sampling path whenever there is contention on Core 0.

That matters for two reasons:

- it protects the acquisition deadline
- it makes the multitasking proof genuine, because a real low-priority project task
  is actually being interrupted by higher-priority work

### Why these priorities matter to the defense

This priority structure supports three technical claims at once:

1. **Real-time claim**  
   the timing source and acquisition path are protected from lower-priority work

2. **Multitasking claim**  
   `control_task` is a real low-priority task that gets preempted by
   higher-priority timer/acquisition work on Core 0

3. **Dual-core claim**  
   processing is important, but it is physically separated onto Core 1 so it does
   not directly interfere with Core 0 timing

**What to say**

We chose priorities by urgency. The timer-dispatch context is above all project
tasks, acquisition is the highest-priority project task because it owns the 10 ms
deadline, processing is also high priority but on Core 1, and control is kept low
because it is necessary control-plane work but not deadline-critical. The exact
numbers are less important than the ordering, but we chose 10, 9, and 1 because
that ordering is explicit, easy to defend, and leaves clear separation between
critical and noncritical work.

**What proves it**

The priorities and the rationale comments are documented in
[main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:121).

**What they may challenge**

- **Challenge: “Why 10 instead of 5?”**  
  **Answer:** Because the exact number is less important than the ordering, but
  10 makes the hierarchy clear: well above low-priority control and still below
  the framework-owned timer-dispatch context.
- **Challenge: “Why is control only 1?”**  
  **Answer:** Because it is real control-plane work, but it is not
  deadline-critical and must lose to the real-time acquisition path whenever
  there is contention on Core 0.
- **Challenge: “Why is processing 9?”**  
  **Answer:** Because it still needs to be high priority on Core 1 so it keeps
  up with the queue stream, but it is intentionally one step below acquisition in
  the design story.
- **Challenge: “What would change if these priorities were swapped?”**  
  **Answer:** The design would be weaker: low-priority control could interfere
  more with sampling, and the scheduling argument for the Core 0 multitasking
  proof would be much less clean.

## 9. Signal Processing and Butterworth Filters

### What a Butterworth filter is

A Butterworth filter is a standard signal-processing filter designed for a
maximally flat passband. In practical terms here, that means it preserves the
frequencies we want to keep without adding intentional ripple in the passband.

- **Engineering rationale**: for coaching and threshold logic, you want cleaner
  signals without artificial passband shaping that could complicate
  interpretation.

### What the repo implements

- 10 Hz 2nd-order low-pass for FSR smoothing
- 12 Hz 2nd-order low-pass for IMU stability branch
- 6-12 Hz 4th-order band-pass for tremor isolation
- 5-sample backward difference for force-change rate

- **Implementation fact**: the filter definitions live in
  [main/filters.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.h:27).
- **Implementation fact**: the actual coefficients and biquad steps live in
  [main/filters.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c:26),
  [main/filters.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c:47),
  and [main/filters.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c:80).
- **Implementation fact**: the core IIR primitive is `biquad_step(...)` in
  [main/filters.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c:15).

### Where they are applied

- FSR low-pass:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:870)
- IMU low-pass:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:878)
- Tremor band-pass:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:888)
- `dF/dt`:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:875)

### Why Butterworth was chosen

- **Engineering rationale**: it is a good fit for smoothing and band isolation in
  a small embedded pipeline
- **Engineering rationale**: the fixed coefficients are stable, lightweight, and
  cheap to run at 100 Hz
- **Engineering rationale**: the project needs interpretable runtime features,
  not raw noisy sensor streams

**What to say**

The filtering is real DSP work, not cosmetic smoothing. Butterworth was chosen
because it gives a smooth passband, runs efficiently as fixed-coefficient IIR on
the ESP32, and supports the force, stability, and tremor features used by the
scorer.

**What proves it**

The coefficients, biquad implementation, and runtime filter calls are all in the
repo.

**What they may challenge**

- **Challenge: "Why Butterworth instead of a moving average?"**  
  **Answer:** Because the project needs more than generic smoothing. It needs
  both smoothing and controlled frequency shaping, including tremor-band
  isolation.
- **Challenge: "Why band-pass 6-12 Hz?"**  
  **Answer:** Because the current tremor feature definition is built around that
  band, and both runtime and calibration logic are shaped consistently around it.
- **Challenge: "Are you actually implementing the filter yourselves?"**  
  **Answer:** Yes. The repo contains the biquad primitive and the fixed
  coefficients for the low-pass and band-pass stages directly in `filters.c`.

## 10. Sampling Rate and Frequency Choice

### Why 100 Hz?

This is the strongest repo-grounded answer:

1. The entire DSP and windowing pipeline is designed around 100 Hz
   - **Implementation fact**: filter headers explicitly say `fs=100 Hz` in
     [main/filters.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.h:27)
   - **Implementation fact**: 250 ms and 1 s windows are encoded as 25 and 100
     samples in [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:58)
2. The tremor band of interest is 6-12 Hz
   - **Implementation fact**: the tremor band-pass is explicitly 6-12 Hz in
     [main/filters.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.h:32)
3. The compute budget still fits inside 10 ms
   - **Implementation fact**: the manual DFT comment explicitly notes the
     runtime cost is well within the 10 ms budget in
     [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:514)

- **Engineering rationale**: 100 Hz is high enough to capture the motion band of
  interest with comfortable headroom above Nyquist, while remaining lightweight
  enough for acquisition, processing, and telemetry on this hardware.

### Why not 10 Hz, 50 Hz, or 200 Hz?

- **Engineering rationale**: 10 Hz would be too close to or below parts of the
  tremor band of interest
- **Engineering rationale**: 50 Hz would be more workable, but 100 Hz gives
  better timing granularity, more comfortable filter design space, and cleaner
  motion features
- **Engineering rationale**: 200 Hz would increase compute and serial pressure
  without a clearly justified payoff in the current coaching design

### Why 20 Hz telemetry instead of 100 Hz telemetry?

- **Implementation fact**: telemetry is emitted every 5th sample in
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1098).
- **Implementation fact**: UART0 runs at 115200 baud in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:86).
- **Implementation fact**: the repo explicitly explains the TX-buffer logic in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:83).

**What to say**

We chose 100 Hz because the entire runtime DSP and feature pipeline is designed
around that rate, the tremor band of interest is 6-12 Hz, and 100 Hz gives
strong headroom while staying within the timing budget.

**What proves it**

Filter definitions, window sizes, and the DFT budget comment are all anchored to
100 Hz in the code.

**What they may challenge**

- **Challenge: "Who decided 100 Hz?"**  
  **Answer:** The current codebase did, in the sense that the DSP, window sizes,
  and feature pipeline are all explicitly designed around 100 Hz.
- **Challenge: "Why not sample slower or faster?"**  
  **Answer:** Slower rates give less headroom for the motion band of interest;
  faster rates increase compute and serial pressure without a clearly justified
  benefit in the current design.

## 11. Telemetry Design

### Why JSON was used

- **Implementation fact**: the runtime packet is structured JSON in
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1104).
- **Implementation fact**: the GUI directly parses incoming JSON in
  [gui/esp32_controller.py](/Users/shanthanu/uart_echo_VitalSignsLab4/gui/esp32_controller.py:23)
  and [gui/esp32_controller.py](/Users/shanthanu/uart_echo_VitalSignsLab4/gui/esp32_controller.py:2323).

- **Engineering rationale**: JSON is readable, structured, easy to inspect in a
  serial terminal, easy to parse in Python, and easy to extend as fields evolve.

### Why it is throttled

- **Implementation fact**: normal telemetry is emitted every 5th sample in
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1098).
- **Implementation fact**: TX buffering concerns are documented in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:83).

- **Engineering rationale**: sending every sample at full richness would make
  serial bandwidth more intrusive. The project separates internal processing rate
  from host update rate to protect timing.

**What to say**

JSON was chosen as a structured host boundary, not because it is the smallest
format, but because it gives inspectable packets, simple Python parsing, and easy
extension while still being manageable at a throttled 20 Hz output rate.

**What proves it**

The packet format and the 20 Hz throttling are explicit in the runtime code.

**What they may challenge**

- **Challenge: "Why not binary?"**  
  **Answer:** Because the host boundary in this project values inspectability and
  ease of parsing. JSON is readable, easy to debug, and easy to extend.
- **Challenge: "Is JSON too expensive for an embedded system?"**  
  **Answer:** It would be a problem if sent at full internal rate, but the design
  avoids that by decimating runtime telemetry to 20 Hz and using the UART TX
  buffer to absorb bursts.

## 12. Python vs LabVIEW

This section must be defended as **engineering rationale**, not as a repo fact.

### The strong technical argument

1. **One host stack does multiple jobs**
   - GUI
   - calibration orchestration
   - serial parsing
   - proof dashboard
   - subsystem diagnostics
2. **The data contract is naturally JSON + Python**
   - direct parsing
   - direct plotting
   - direct scripting
3. **The tooling is source-reviewable**
   - easier to diff, review, and defend than opaque visual wiring
4. **The project already benefits from code reuse**
   - the same serial/JSON model supports the GUI and the test/proof tools

- **Implementation fact**: the main GUI is Python in
  [gui/esp32_controller.py](/Users/shanthanu/uart_echo_VitalSignsLab4/gui/esp32_controller.py:4).
- **Implementation fact**: multiple focused host diagnostics are also Python in
  [tests/fsr_test.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/fsr_test.py:3),
  [tests/imu_test.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/imu_test.py:3),
  and [tests/multitask_proof.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/multitask_proof.py:3).

### What not to say

Do **not** say "Python was easier than LabVIEW."

That is weak and subjective.

### Better phrasing

Use this framing:

> Python was chosen because the host side needed to be more than a GUI. It also
> needed structured serial parsing, reusable diagnostics, a multitasking proof
> dashboard, and source-reviewable tooling. Python let us use one version-controlled
> codebase for all of those roles against the same JSON protocol.

**What to say**

Python unified the host-side architecture: one language, one protocol, one
tooling model, multiple applications.

**What proves it**

The repo contains a GUI and multiple Python diagnostics built around the same
telemetry/control model.

**What they may challenge**

- **Challenge: "Why not LabVIEW if this is instrumentation-heavy?"**  
  **Answer:** Because the host side needed more than visualization. It needed a
  GUI, serial parsing, calibration orchestration, proof tooling, and reusable
  diagnostics in one version-controlled codebase.
- **Challenge: "What do you gain technically from Python?"**  
  **Answer:** One host stack for multiple roles, direct JSON handling, scriptable
  diagnostics, source-reviewable logic, and easy reuse across GUI and proof
  tools.

## 13. Calibration Design

### Overall philosophy

Calibration personalizes the runtime logic so scoring is not based only on fixed
generic constants.

- **Implementation fact**: persisted calibration parameters are stored in
  `cal_params_t` in [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:67).
- **Implementation fact**: defaults exist even before calibration in
  [main/nvs_storage.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/nvs_storage.c:92).

### C1: still-hand baseline

Learns:

- gyro bias
- neutral pose
- no-motion tremor baseline

- **Implementation fact**: C1 starts in
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1039).
- **Implementation fact**: C1 stores `gyro_bias`, `tremor_rms_ref`, and neutral
  orientation fields in
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1108).

### C2: no-pressure FSR baseline

Learns:

- baseline mean per finger
- baseline sigma per finger
- contact on/off thresholds

- **Implementation fact**: C2 starts in
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1136).
- **Implementation fact**: thresholds are calculated as `mu + 5*sigma` and
  `mu + 3*sigma` with floors in
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1180).

### C3: reference grip force

Learns:

- `f_ref_open`

- **Implementation fact**: C3 starts in
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1217).
- **Implementation fact**: `params->f_ref_open = grip_force` in
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1236).

### C4: normal hand motion reference

Learns:

- broadband motion RMS
- tremor-band fraction during normal motion
- `f95`
- roll/pitch excursion references

- **Implementation fact**: C4 starts in
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1253).
- **Implementation fact**: motion references are stored in
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1278).

### Why this sequence makes sense

- **Engineering rationale**: C1 must happen before motion analysis because it
  establishes IMU bias and quiet tremor baseline
- **Engineering rationale**: C2 must happen before force/contact logic because it
  establishes what "no pressure" looks like for this user
- **Engineering rationale**: C3 then learns intended grip force
- **Engineering rationale**: C4 finally learns deliberate non-tremor motion, so
  runtime can distinguish normal motion from abnormal shake

**What to say**

Calibration progresses from quiet baseline, to no-pressure contact learning, to
intentional grip, to normal motion. Each stage feeds a different runtime branch.

**What proves it**

Each stage has a dedicated calibration function and writes specific fields into
`cal_params_t`.

**What they may challenge**

- **Challenge: "What exactly is calibrated?"**  
  **Answer:** IMU bias and neutral pose, tremor baseline, FSR mean/sigma
  baselines, contact thresholds, reference grip force, and normal-motion
  references.
- **Challenge: "Why not use fixed thresholds for everyone?"**  
  **Answer:** Because sensor fit, hand size, baseline pressure, and natural
  movement vary too much from user to user. Calibration makes the runtime logic
  user-aware.

## 14. Scoring and Threshold Design

### Where thresholds live

There are three layers:

1. **Personalized calibration values**
   - `on_thresh`
   - `off_thresh`
   - `gyro_bias`
   - `f_ref_open`
   - `motion_tremor_ratio_ref`
   - `tremor_rms_ref`
2. **Fixed difficulty constants**
   - easy / medium / hard multipliers
3. **Runtime logic**
   - warning/error bitmasks and score penalties

- **Implementation fact**: calibrated values are defined in
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:68).
- **Implementation fact**: difficulty constants live in
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:121).
- **Implementation fact**: warning/error flags live in
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:199).

### How difficulty modes are applied

- **Implementation fact**: `apply_difficulty(...)` centralizes difficulty scaling
  in [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:587).
- **Implementation fact**: mode commands feed that function in
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:643).

### How force thresholds work

- **Implementation fact**: force modes scale from `f_ref_open` in
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:606).
- **Implementation fact**: sustained force windows use `FORCE_SUSTAIN_SAMPLES`
  in [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:119)
  and [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1059).

### How tremor thresholds work

- **Implementation fact**: tremor warnings require both amplitude and ratio
  criteria in [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:141)
  and [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1035).

### How hold instability works

- **Implementation fact**: hold instability combines angular-speed RMS and
  orientation spread in
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1015).

### How score penalties work

- **Implementation fact**: difficulty-specific warning and error penalties are
  defined in [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:185).

### Why this design is reasonable

- **Engineering rationale**: calibration makes the system user-aware
- **Engineering rationale**: difficulty constants give consistent mode behavior
- **Engineering rationale**: runtime bitmask logic keeps interpretation explicit
  and inspectable

**What to say**

Scoring is not purely hard-coded and not purely learned. It is a hybrid:
calibration personalizes the baseline, difficulty selects tolerance, and runtime
logic turns the current signal state into warnings, errors, and score changes.

**What proves it**

The calibrated fields, mode constants, and runtime threshold branches all exist
separately and are wired together in the current code.

**What they may challenge**

- **Challenge: "How are thresholds actually computed?"**  
  **Answer:** Some come directly from calibration, such as contact thresholds and
  reference force; others come from fixed mode constants; runtime logic combines
  both when it sets warnings, errors, and score penalties.
- **Challenge: "What is learned versus fixed?"**  
  **Answer:** Learned values are the personalized baselines from calibration.
  Fixed values are things like mode multipliers, floors, and the structure of the
  warning/error policy.
- **Challenge: "Why are difficulty modes meaningful?"**  
  **Answer:** Because they do not just rename one threshold. They retune
  multiple tolerance dimensions such as force, tremor, hold instability, and
  scoring penalties.

## 15. Likely Challenge Questions with Suggested Answers

This section is intentionally written as oral-defense material, not just prompts.
For each likely question, the answer is phrased in a way you can actually say,
and the code anchors tell you what to point at if they ask for proof.

### 15.1 Real-time and timing

**Q: Why is this real-time?**

**Suggested answer**  
This project is real-time because it has a repeated time-critical job with a fixed
deadline: sensor acquisition every 10 ms. The timer creates the schedule, the
semaphore wakes the acquisition task, and the acquisition task has to complete one
sample cycle per tick. It is not just “running fast”; it is deadline-driven.

**Code to point at**
- timer setup:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:288)
- semaphore give:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:147)
- acquisition wait:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:329)

**Q: How do you prove the system is 100 Hz?**

**Suggested answer**  
The strongest proof is the timer configuration itself: `esp_timer_start_periodic(..., 10000)`
means a 10,000 microsecond period, which is 10 ms, or 100 Hz. Then the repo adds
secondary evidence: a measured-rate log in firmware, a proof GPIO toggle for
external observation, a host-side timing monitor, and an `actual_hz` field in the
runtime JSON.

**Code to point at**
- timer period:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:297)
- measured-rate log:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:389)
- proof pin toggle:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:145)
- host monitor:
  [tests/dual_core_monitor.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/dual_core_monitor.py:254)
- JSON field:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1117)

**Q: Is the 20 Hz JSON stream your sample rate?**

**Suggested answer**  
No. The internal acquisition and processing cadence is 100 Hz. The JSON stream is
intentionally decimated to 20 Hz by transmitting every 5th sample so that host
telemetry does not dominate serial bandwidth or interfere with runtime behavior.

**Code to point at**
- 20 Hz emission logic:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1098)
- TX buffer rationale:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:83)

**Q: What exactly is the deadline?**

**Suggested answer**  
The deadline is that the acquisition side has to keep up with one full sample cycle
per 10 ms timer tick. That means ADC reads, IMU read attempt, sample packaging,
and queue handoff all have to stay within that periodic schedule.

**Code to point at**
- acquisition loop stages:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:325)

### 15.2 Interrupts, async sources, and event design

**Q: What are your three interrupts or asynchronous event sources?**

**Suggested answer**  
The three important asynchronous sources are the periodic `esp_timer` trigger,
UART0 host-command receive events, and the UART2 IMU serial data path. The timer
drives the 100 Hz cadence, UART0 brings in commands from the host, and the IMU
produces serial bytes independently of the application flow.

**Code to point at**
- timer callback:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:137)
- UART0 event queue install:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:98)
- UART0 event consumption:
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:133)
- IMU UART config:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:271)
- IMU packet read:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:165)

**Q: Are those three things explicit application ISRs you wrote yourselves?**

**Suggested answer**  
No. The more accurate statement is that they are three asynchronous or
interrupt-driven paths in the system. The timer path uses ESP-IDF timer
infrastructure, UART0 uses the driver event queue, and UART2 is an asynchronous
serial receive path whose data is consumed from the driver buffer by
`acquisition_task`.

**Code to point at**
- timer dispatch mode:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:291)
- UART0 event queue:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:98)
- UART2 byte-drain path:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:165)

**Q: Is a stop command itself an interrupt?**

**Suggested answer**  
No. A stop command is one meaning of UART0 data arrival. The asynchronous source is
the UART receive event; “stop” is just one command interpreted from that event.

**Code to point at**
- UART event handling:
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:137)
- stop command dispatch:
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:101)

**Q: If the IMU is read from the acquisition task, why do you still call it asynchronous?**

**Suggested answer**  
Because the IMU produces serial bytes independently of the main application flow.
The firmware then consumes those bytes from the UART driver buffer during the next
acquisition cycle. So the peripheral data arrival is asynchronous even though the
application-side read is scheduled.

**Code to point at**
- IMU byte drain:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:168)

**Q: Why is the IMU UART path asynchronous but the FSR path is not?**

**Suggested answer**  
Because the BNO085 pushes serial data toward the ESP32 on its own, so UART2 sees
bytes arriving independently of the main application flow. The FSR sensors do not
push packets or events. They are analog voltages, so the firmware samples them
when the 10 ms acquisition cycle runs.

**Code to point at**
- IMU UART config:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:271)
- FSR ADC read path:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:336)

### 15.3 Multitasking and scheduling

**Q: Where exactly is the multitasking in your project?**

**Suggested answer**  
The strongest class-definition multitasking example is on Core 0. `control_task`
is the real low-priority project task, and every 10 ms the higher-priority
timer/acquisition chain interrupts it. After acquisition finishes, that same
control task resumes.

**Code to point at**
- low-priority control role:
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:15)
- proof-mode runnable control task:
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:168)
- timer/semaphore/acq events:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:140)

**Q: How do you know the multitasking proof is not fake?**

**Suggested answer**  
Because the low-priority task is a real project task that already exists for UART
and control-plane work. The proof path does not invent a fake task; it instruments
the real one. Also, the host dashboard is not guessing from packet timing. Core 1
assembles proof snapshots from raw firmware event timestamps.

**Code to point at**
- control task is real project work:
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:18)
- proof snapshots built from raw events:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:344)
- host does not re-derive timing:
  [tests/multitask_proof.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/multitask_proof.py:262)

**Q: Why does `control_task` exist at all?**

**Suggested answer**  
Because UART command ingress, mode changes, calibration control, and proof-mode
coordination are real project responsibilities, but they should not live inside the
hard-real-time acquisition loop. Separating them into a low-priority control-plane
task is cleaner architecturally and also gives an honest multitasking story.

**Code to point at**
- control responsibilities:
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:15)
- control-to-processing intent messages:
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:32)

**Q: What would break if the priorities were wrong?**

**Suggested answer**  
If control were too high, it could delay the sampling path on Core 0. If the timer
dispatch context were not above acquisition, the release event itself could slip.
If acquisition were not above control, the multitasking proof would also be weaker
because the real-time job would no longer clearly outrank low-priority work.

**Code to point at**
- priority rationale:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:121)

**Q: Do you set timer priority 22 yourself?**

**Suggested answer**  
No. The application does not assign that value directly. What it chooses is
`ESP_TIMER_TASK` dispatch mode. ESP-IDF then runs the callback in its framework
timer service task, which sits above the project tasks and can release
`acquisition_task` on time.

**Code to point at**
- dispatch mode selection:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:291)
- rationale comment in app code:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:125)

### 15.4 Dual-core architecture

**Q: What is the difference between multitasking and dual-core parallelism here?**

**Suggested answer**  
Multitasking is the same-core preemption story on Core 0: a low-priority task is
interrupted by higher-priority work. Dual-core parallelism is the hardware split:
Core 0 handles acquisition and control, while Core 1 handles processing and output.

**Code to point at**
- task pinning:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:144)
- Core 1 consumer loop:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:828)

**Q: Why not do everything on one core?**

**Suggested answer**  
Because acquisition has a fixed 10 ms cadence, while processing, calibration, and
JSON formatting have more variable latency. Splitting them across cores gives timing
isolation so heavier processing does not directly compete with the acquisition
deadline.

**Code to point at**
- queue split and rationale:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:59)
- processing runs separately on Core 1:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:837)

**Q: Why use a queue instead of shared globals?**

**Suggested answer**  
The queue makes the ownership boundary explicit: Core 0 produces complete
`raw_sample_t` packets, Core 1 consumes copies of them. That avoids ad hoc shared
mutable state and lets FreeRTOS provide the synchronization.

**Code to point at**
- queue copy semantics:
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:43)
- queue producer/consumer description:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:61)

**Q: What happens if Core 1 falls behind?**

**Suggested answer**  
The queue depth gives Core 1 some slack, but not unlimited slack. At 100 Hz with a
10-deep queue, Core 1 has about 100 ms of backlog tolerance before Core 0 starts
dropping the oldest sample and posting the newest one instead.

**Code to point at**
- queue depth rationale:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:69)
- oldest-sample drop/replacement logic:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:383)

### 15.5 FreeRTOS and tasking model

**Q: Why use FreeRTOS instead of one polling loop?**

**Suggested answer**  
FreeRTOS gave the project a cleaner separation of concerns: a timer-driven
acquisition task, a low-priority control task, a queue-based Core 1 processing
task, and well-defined blocking behavior. A single polling loop would mix
time-critical and variable-latency work much more tightly.

**Code to point at**
- tasking model assembly:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:113)
- processing blocks on queue:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:855)

**Q: What did FreeRTOS actually buy you here?**

**Suggested answer**  
It bought deterministic wakeups, explicit priorities, safe queue handoff, and a
clean distinction between blocking and active work. In this design, that matters
more than raw speed because the architecture depends on scheduling semantics.

**Code to point at**
- semaphore wake path:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:329)
- queue receive:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:862)

### 15.6 Signal processing and feature extraction

**Q: Why Butterworth instead of a moving average?**

**Suggested answer**  
A moving average can smooth noise, but it does not give the same frequency-shaping
control. This project needed both smoothing and tremor-band isolation. Butterworth
IIR sections give a smooth passband and practical embedded implementation, and the
repo uses both low-pass and band-pass stages.

**Code to point at**
- low-pass and band-pass definitions:
  [main/filters.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c:26)
  and [main/filters.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c:47)

**Q: Are you actually implementing the filters yourselves?**

**Suggested answer**  
Yes. The filter primitive is a Direct Form II Transposed biquad implemented in
`biquad_step(...)`, and the project defines fixed coefficients for the 10 Hz,
12 Hz, and 6-12 Hz filters directly in the code.

**Code to point at**
- biquad implementation:
  [main/filters.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c:15)

**Q: Why 6-12 Hz for tremor?**

**Suggested answer**  
Because the current project defines that as the tremor band of interest, and the
runtime and calibration logic are both built around that same band so the signal
processing and threshold logic stay consistent.

**Code to point at**
- band-pass definition:
  [main/filters.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.h:32)
- tremor application:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:888)

**Q: What actual features are you computing beyond raw force and gyro?**

**Suggested answer**  
The project computes filtered force sum, force derivative, angular-speed RMS,
tremor RMS, tremor ratio, `f95`, force coefficient of variation, roll/pitch
excursion, swing rate, contact state, engagement gate, warn/error bitmasks, and a
running score.

**Code to point at**
- feature extraction block:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:918)
- warning/error logic:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1011)

### 15.7 Sampling-rate and telemetry design

**Q: Why 100 Hz instead of 50 Hz?**

**Suggested answer**  
Because the DSP and window sizes are designed around 100 Hz, the tremor band of
interest is 6-12 Hz, and 100 Hz gives comfortable headroom above Nyquist while
still leaving enough compute budget for the runtime pipeline.

**Code to point at**
- filter sample-rate assumptions:
  [main/filters.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.h:27)
- 100 Hz windows:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:58)
- DFT budget:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:514)

**Q: Why not 200 Hz?**

**Suggested answer**  
At the current project scope, 200 Hz would increase compute and serial pressure
without a clearly justified benefit. The present feature set already fits the 100 Hz
budget well, and the codebase is tuned around that rate.

**Code to point at**
- current rate assumptions throughout processing:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:834)

**Q: Why JSON if bandwidth is limited?**

**Suggested answer**  
Because JSON is the host boundary, not the control loop. It is human-readable,
easy to inspect and parse in Python, and easy to extend. Bandwidth is managed by
only transmitting every fifth sample and by using the UART TX buffer to absorb
bursts.

**Code to point at**
- TX buffer rationale:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:83)
- JSON packet assembly:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1104)

### 15.8 Python vs LabVIEW

**Q: Why Python instead of LabVIEW?**

**Suggested answer**  
Because the host side had to be more than a display. It had to support structured
JSON parsing, a full GUI, calibration orchestration, subsystem diagnostics, and a
multitasking proof dashboard. Python let the project use one version-controlled,
scriptable host stack for all of those roles against the same UART/JSON protocol.

**Code to point at**
- GUI role:
  [gui/esp32_controller.py](/Users/shanthanu/uart_echo_VitalSignsLab4/gui/esp32_controller.py:4)
- FSR diagnostic:
  [tests/fsr_test.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/fsr_test.py:3)
- IMU diagnostic:
  [tests/imu_test.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/imu_test.py:3)
- proof dashboard:
  [tests/multitask_proof.py](/Users/shanthanu/uart_echo_VitalSignsLab4/tests/multitask_proof.py:3)

**Q: What do you gain technically from Python?**

**Suggested answer**  
Unified tooling, reusable parsers, scriptable diagnostics, source-reviewable host
logic, and easy extension of the protocol consumer without introducing a second
host-toolchain model.

**Code to point at**
- GUI packet routing:
  [gui/esp32_controller.py](/Users/shanthanu/uart_echo_VitalSignsLab4/gui/esp32_controller.py:2319)

### 15.9 Calibration and thresholds

**Q: What exactly is calibrated?**

**Suggested answer**  
The project calibrates IMU bias and neutral pose, a no-motion tremor baseline,
FSR baseline mean and sigma, contact on/off thresholds, a reference grip force,
and normal-motion references like broadband RMS, tremor-band ratio, and motion
excursion values.

**Code to point at**
- calibration state fields:
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:67)
- C1:
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1039)
- C2:
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1136)
- C3:
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1217)
- C4:
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1253)

**Q: What is learned versus fixed?**

**Suggested answer**  
The personalized baseline values are learned in calibration. The mode constants,
some threshold floors, and the structure of the warn/error logic are fixed in the
code. Runtime scoring combines both: learned baselines plus fixed policy.

**Code to point at**
- calibrated fields:
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:68)
- fixed difficulty constants:
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:121)

**Q: How are contact thresholds computed?**

**Suggested answer**  
They are learned in C2 from the no-pressure baseline. The code computes candidate
thresholds as `mu + 5*sigma` for contact-on and `mu + 3*sigma` for contact-off,
then applies minimum floors so the thresholds do not collapse unrealistically low.

**Code to point at**
- C2 threshold calculation:
  [main/calibration.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/calibration.c:1180)

**Q: How are easy, medium, and hard actually defined?**

**Suggested answer**  
They are defined by fixed difficulty constants that scale allowable force, tremor,
hold instability, force variability, force spikes, and score penalties. The modes
do not just rename one threshold; they retune several tolerance dimensions at once.

**Code to point at**
- difficulty constants:
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:121)
- difficulty application:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:587)

**Q: How is the score updated?**

**Suggested answer**  
The runtime computes warning and error bitmasks first. Then warnings subtract the
warning penalty and errors subtract the error penalty, with penalty size depending
on the selected difficulty.

**Code to point at**
- warn/error bitmasks:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1011)
- score update:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1089)

### 15.10 Limits, failure modes, and next steps

**Q: What are the main current limitations?**

**Suggested answer**  
The thresholds and difficulty scaling are code-grounded and calibrated, but they
still need broader user validation. The host telemetry is intentionally throttled,
which is good for runtime but means it is not a full-rate raw data logger. Motor
feedback is wired in but not yet the main finished user-facing loop.

**Code to point at**
- difficulty constants:
  [main/data_types.h](/Users/shanthanu/uart_echo_VitalSignsLab4/main/data_types.h:121)
- telemetry throttling:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1098)
- motor pulses:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1083)

**Q: What would you improve next?**

**Suggested answer**  
I would validate thresholds across more users, formalize empirical tuning, add
clearer benchmark datasets for calibration and scoring evaluation, consider more
compact telemetry if the host link becomes a bottleneck, and continue integrating
the motor-feedback path into a stronger closed-loop coaching experience.

## 16. Fast Reference: Best Screenshot Anchors

If you need quick code screenshots for slides or a live walkthrough, use:

- Real-time timer source:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:288)
- Acquisition wake:
  [main/acquisition.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/acquisition.c:329)
- Low-priority control task:
  [main/control.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/control.c:163)
- Dual-core pinning:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:144)
- Inter-core queue:
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:74)
- Filter primitive:
  [main/filters.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/filters.c:15)
- Runtime filter application:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:870)
- JSON telemetry packet:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:1104)
- Proof snapshot serialization:
  [main/processing.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/processing.c:302)
