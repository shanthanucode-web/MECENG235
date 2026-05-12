# Haptic Surgical Skill Trainer — Lab 4

ESP32 firmware and Python tools for a teaching-focused surgical skill trainer glove.
The glove measures finger force with three FSR 402 sensors, measures hand motion with
a BNO085 IMU, scores force and tremor behavior in real time, and streams live JSON
telemetry to a desktop GUI.

This README is intentionally written as both:

- a normal project README, so you can build, flash, and run the system
- a teaching guide, so you can understand how the code works well enough to modify it

The project is implemented natively with ESP-IDF on the ESP32. It does not use Arduino.

For a deeper developer-facing map of the repo, see
[CODEBASE_WALKTHROUGH.md](CODEBASE_WALKTHROUGH.md).

---

## Table of Contents

1. [Project at a Glance](#project-at-a-glance)
2. [Why This Project Exists](#why-this-project-exists)
3. [System Overview](#system-overview)
4. [Glossary](#glossary)
5. [Hardware Guide](#hardware-guide)
6. [Firmware Architecture](#firmware-architecture)
7. [Real-Time Behavior](#real-time-behavior)
8. [Asynchronous Event Sources](#asynchronous-event-sources)
9. [Multitasking Behavior](#multitasking-behavior)
10. [Code Tour by Subsystem](#code-tour-by-subsystem)
11. [Calibration Guide](#calibration-guide)
12. [Scoring and Warning Logic](#scoring-and-warning-logic)
13. [UART Protocol and Telemetry](#uart-protocol-and-telemetry)
14. [GUI Guide](#gui-guide)
15. [Build, Flash, and Run](#build-flash-and-run)
16. [Diagnostic Tools](#diagnostic-tools)
17. [How to Modify the Project](#how-to-modify-the-project)
18. [Troubleshooting](#troubleshooting)

---

## Project at a Glance

### What the glove does

The glove is trying to answer a simple question:

> Is the user applying force and moving in a controlled way, or are they showing
> excessive force and tremor-like shaking?

To do that, the system combines:

- **force sensing** from three FSR 402 sensors
- **motion sensing** from a BNO085 IMU
- **real-time signal processing** on the ESP32
- **live telemetry and calibration guidance** in a Python GUI

### What is implemented today

Implemented and working in the current codebase:

- dual-core ESP32 firmware
- 100 Hz data acquisition
- live force and motion processing
- four-step calibration
- persistent calibration storage in NVS
- live JSON telemetry at 20 Hz
- desktop GUI with calibration wizard, plots, session score, and hand visualizer

### What is not fully finished

The codebase includes motor-control scaffolding for haptic feedback, but the motors are
not yet fully integrated and tested in final hardware. In practical terms:

- the firmware contains motor pulse logic
- GPIO assignments for the motors exist
- the hardware/software integration with transistor driver stages was not fully completed

So the glove is ready as a sensing and scoring platform, but haptic motor feedback should
be treated as an incomplete subsystem.

### The big idea in one sentence

Core 0 samples the glove every 10 ms, Core 1 processes those samples, the GUI displays
the results, and calibration personalizes the thresholds so the score reflects tremor
and force problems rather than generic movement.

---

## Why This Project Exists

A surgical trainer should not punish someone just for moving their hand. Real surgical
movement includes normal grip changes, normal repositioning, and intentional motion.
What matters more is whether the user shows:

- **too much force**
- **unstable holding**
- **tremor-like shaking**

This project tries to separate those categories instead of treating all movement as bad.

### What the score is intended to mean

The score is meant to represent clinical control, not just stillness. In the current
design, the score is mainly driven by:

- **force behavior**
- **tremor behavior**

Normal hand movement by itself should not reduce the score.

### Why calibration matters

Every person wears the glove differently. Sensor baselines drift. Hands differ in size,
grip style, and natural motion. A hardcoded threshold would be fragile. Calibration lets
the firmware learn:

- what "still" looks like for this user and glove fit
- what "no finger pressure" looks like
- what a normal light grip looks like
- what normal intended hand motion looks like

That makes the runtime scorer far more useful than a one-size-fits-all threshold table.

---

## System Overview

Think of the project as a pipeline:

```text
FSR sensors + BNO085 IMU
        |
        v
Core 0 acquisition task (100 Hz)
        |
        v
FreeRTOS queue
        |
        v
Core 1 processing task
  - filtering
  - calibration logic
  - warning/error logic
  - score updates
  - JSON telemetry
        |
        v
UART0 @ 115200 baud
        |
        v
Python GUI
  - calibration wizard
  - plots
  - status panels
  - hand visualizer
```

### End-to-end story

1. The ESP32 wakes the acquisition task every 10 ms.
2. The acquisition task reads the three force sensors and tries to read a fresh IMU packet.
3. It packs the sample into `raw_sample_t` and posts it to a FreeRTOS queue.
4. The processing task receives the sample on the other core.
5. The processing task filters the signals, updates contact and motion state, checks
   warnings/errors, updates the running score, and emits JSON telemetry.
6. The GUI reads the JSON stream and turns it into human-readable feedback.

### Why the design is split this way

The project deliberately separates **sampling** from **thinking**:

- sampling must happen on time
- processing can be more computationally expensive

That is why the firmware uses two cores and a queue between them.

---

## Glossary

This section is here so a first-time reader does not have to infer basic terms.

| Term | Meaning in this project |
|---|---|
| Task | A FreeRTOS thread of execution |
| Core | One CPU inside the ESP32 dual-core chip |
| Queue | A thread-safe FIFO used to pass data from one task/core to another |
| Real-time | The code must complete repeated work inside a fixed timing deadline |
| Calibration | A guided process that stores personal reference values |
| Telemetry | The JSON data stream sent from the ESP32 to the host |
| Tremor band | The 6-12 Hz motion band used as a tremor indicator |
| NVS | Non-volatile storage on the ESP32 used to save calibration |
| Hold state | The glove is engaged and motion is low enough to count as holding still |
| Active state | The glove is engaged and moving |

---

## Hardware Guide

### Main components

| Component | Role |
|---|---|
| ESP32 HUZZAH32 | Main microcontroller; runs the firmware and all real-time logic |
| 3x FSR 402 | Measures finger force at the thumb, index, and middle finger |
| BNO085 | Measures orientation and acceleration for motion analysis |
| Host computer | Runs the GUI and displays feedback |
| Motor driver stage | Intended for haptic feedback; not fully integrated/tested yet |

### Pin map

| Component | GPIO | Notes |
|---|---|---|
| FSR thumb | GPIO 34 | ADC1_CH6, input-only |
| FSR index | GPIO 39 | ADC1_CH3, input-only |
| FSR middle | GPIO 36 | ADC1_CH0, input-only |
| BNO085 data to ESP32 | GPIO 32 | UART2 RX |
| BNO085 input from ESP32 | GPIO 33 | Unused in UART-RVC mode |
| Motor 0 | GPIO 25 | force warning motor |
| Motor 1 | GPIO 26 | force error motor |
| Motor 2 | GPIO 27 | instability motor |
| Motor 3 | GPIO 14 | tremor motor |
| Status LED | GPIO 13 | built-in board LED |
| Frequency proof pin | GPIO 4 | toggles every 100 Hz tick |
| Core 1 debug pin | GPIO 12 | toggles every processing cycle |

### BNO085 UART-RVC notes

This project uses the BNO085 in **UART-RVC mode**.

Important implications:

- the IMU sends orientation and acceleration over UART
- the firmware derives angular velocity in software from Euler angle differences
- the firmware does **not** currently use a UART2 event queue for IMU input
- the acquisition task reads the IMU directly each cycle

Plain-English picture:

- the **IMU** is the motion sensor; it tells the system how the hand is moving
- **UART2** is the separate serial connection the ESP32 uses to listen to that sensor
- this is separate from **UART0**, which is used for host GUI commands and telemetry
- this is also different from the FSR sensors, which do not "send packets" on their own;
  the firmware actively samples them through the ADC every 10 ms

Required wiring:

- ESP32 3.3 V -> BNO085 VIN
- ESP32 GND -> BNO085 GND
- ESP32 GPIO32 -> BNO085 SDA
- BNO085 `P0/PS0` tied high
- BNO085 `P1/PS1` low or unconnected

After changing `P0/P1`, power-cycle the IMU itself. Resetting only the ESP32 is not
enough to guarantee the BNO085 re-enters UART-RVC mode.

### What "force" means here

The glove does not measure force directly in SI units at the sensor. Each FSR is a
nonlinear sensor whose voltage changes with pressure. The firmware:

1. samples ADC voltage
2. converts millivolts into an estimated Newton value through a lookup/interpolation model
3. uses calibration and thresholds on those Newton estimates

So when you see force in the GUI, that is an interpreted engineering quantity, not raw ADC.

### Motor status

The code has a motor subsystem (`motor_control.c/.h`) and warning events still trigger
motor pulse calls, but `MOTORS_ENABLED` is set to `0` in `main/data_types.h`. That means:

- the interface exists
- the warning-to-motor mapping exists
- the current project should be presented as **sensor-first**, with haptic feedback
  hardware still incomplete

---

## Firmware Architecture

### The short version

The firmware is divided into two responsibilities:

- **Core 0** owns the real-time acquisition path and the low-priority control plane
- **Core 1** processes sensor data

### The actual task split

```text
CORE 0
  esp_timer callback -> semaphore
  acquisition_task wakes
  reads ADC + IMU
  posts raw_sample_t to queue
  control_task handles UART0 command ingress
  control_task coordinates host commands and proof-mode state

CORE 1
  processing_task waits on queue
  filters and interprets sample
  updates warnings, errors, score, and state
  sends JSON and proof snapshots to the host
```

### Why this matters

If one task did everything on one core, JSON formatting, GUI traffic, and signal
processing could delay sampling. This design isolates timing-sensitive work from
higher-level work.

### Queue handoff

The two cores communicate through one FreeRTOS queue:

- producer: `acquisition_task`
- consumer: `processing_task`
- item type: `raw_sample_t`

This is the clean boundary in the firmware. If you want to understand the system, learn
that struct and the producer-consumer relationship first.

### Plain-English workflow

If you need to explain the runtime without sounding like you are reciting file names,
say it this way:

1. every 10 ms, a timer event releases the acquisition path on Core 0
2. Core 0 reads the FSR sensors and checks for fresh IMU data
3. Core 0 packages one `raw_sample_t` and pushes it into the queue
4. Core 1 receives that sample, filters it, and updates state, warnings, and score
5. every 5th sample, Core 1 emits one JSON packet to the host GUI

That is the simplest accurate mental model of the system.

---

## Real-Time Behavior

This project should be explained in the class definition of real-time:

> Real-time programming is not "run as fast as possible." It means a repeated task must
> complete its work within a specified time interval.

### What is the repeated real-time task here?

The repeated time-critical task is **sensor acquisition at 100 Hz**.

- period: 10 ms
- source: `esp_timer_start_periodic(..., 10000)`
- deadline: the acquisition path must keep up with one full sample cycle every 10 ms

### How the timing works

The firmware uses an `esp_timer` periodic callback every 10,000 microseconds.
That callback:

- is dispatched via `ESP_TIMER_TASK`
- gives `s_timer_sem`
- wakes `acquisition_task`

Important nuance:

- our application chooses the **dispatch mode** (`ESP_TIMER_TASK`)
- the ESP-IDF framework provides the high-priority timer service context
- we do **not** assign that framework task priority ourselves in application code

Then `acquisition_task`:

- blocks until the semaphore is released
- reads all three FSR channels
- reads the BNO085 packet if available
- fills `raw_sample_t`
- sends that sample to the queue

The important point is that the acquisition task is driven by a fixed schedule.
The timer defines the 10 ms cadence; acquisition performs the time-critical work
inside that cadence.

Another important nuance:

- the timer period is 10 ms
- `acquisition_task` is released every 10 ms
- `acquisition_task` does **not** run for the full 10 ms; it runs only long enough
  to complete one sample cycle and then blocks again

### Why this is real-time

It is real-time because:

- the job is periodic
- the period is fixed
- the work has a deadline
- missing the deadline would degrade the correctness of the system

### What this is not

It is **not** merely:

- using a fast microcontroller
- writing optimized C
- updating the GUI quickly

Those help, but they are not the definition of real-time.

---

## Asynchronous Event Sources

The three important asynchronous or interrupt-driven paths in the current system are:

1. **the periodic `esp_timer` trigger**
   - this is the 100 Hz timing source
2. **UART0 host command receive events**
   - commands such as `STOP`, `EXIT`, `E`, `M`, `H`, and `C1`-`C4` arrive this way
3. **UART2 IMU serial data arrival**
   - the BNO085 sends UART-RVC bytes independently of the main application flow

The easiest plain-English explanation is:

- the timer interrupts the normal flow every 10 ms to release acquisition
- the host GUI can interrupt the control path by sending UART0 commands
- the IMU can produce fresh serial data independently of both of those

Important distinctions:

- a `STOP` command is **not** a separate interrupt type; it is one meaning of UART0 data arrival
- `acquisition_task` is **not** the IMU interrupt; it is the task that consumes the
  asynchronously arriving UART2 data
- the FSR sensors are **not** asynchronous event sources in the same sense; the firmware
  polls their analog values on schedule rather than receiving pushed packets from them

If someone asks whether these are "three ISRs we wrote ourselves," the honest answer is no.
They are better described as **three asynchronous event sources / interrupt-driven paths**
used by the implementation.

---

## Multitasking Behavior

This project should also be explained in class terms:

> Computer multitasking is the apparent simultaneous performance of one or more tasks by a CPU.

### How multitasking appears in this project

FreeRTOS provides multiple real tasks in the project itself:

- `acquisition_task`
- `control_task`
- `processing_task`
- background framework tasks such as the timer task and UART driver support

The important class-definition example is on **Core 0**:

- `control_task` is a real low-priority project task
- the ESP-IDF `esp_timer` dispatch task runs at higher priority
- `acquisition_task` wakes at priority 10 every 10 ms

That means a lower-priority **project-owned** task is genuinely interrupted by
higher-priority work on the same CPU core.

### The misconception to avoid

One easy mistake is to say that the project proves multitasking simply because the ESP32
has two cores. That is not the strongest explanation.

- **dual-core use** proves parallelism
- **same-core preemption** proves the class-definition multitasking story

The stronger defense is therefore:

- Core 0 proves multitasking, because `control_task` is interrupted by the timer/acquisition path
- Core 1 proves parallelism, because processing runs on a physically separate CPU core

### Why that distinction matters

FreeRTOS gives the project a multitasking model. The dual-core ESP32 also turns the two
main application paths into genuine parallel execution:

- acquisition + control on Core 0
- processing on Core 1

So the system is both:

- a multitasking system in the software sense, because Core 0 has
  priority-based interruption between real tasks
- a parallel dual-core design in the hardware sense

### Why the professor will care

If you present this project, the correct phrasing is:

- multitasking is demonstrated on Core 0, where the low-priority `control_task`
  is interrupted by higher-priority timer/acquisition work
- the dual-core ESP32 separately lets the acquisition/control side and the
  processing side run in parallel
- the queue is the synchronization boundary between them

Do not reduce the explanation to "it uses two cores, so it is multitasking." That is
too loose and will sound less rigorous than the actual design.

---

## Code Tour by Subsystem

This section is the guided tour for someone opening the codebase for the first time.

### 1. `main/main.c` — startup and system wiring

Start here if you want the top-level picture.

`main.c` does the following:

- loads saved calibration from NVS, or defaults if none exist
- creates the inter-core queue
- initializes acquisition support
- initializes LED and motor subsystems
- installs the UART0 driver and gets its event queue
- initializes processing and control modules
- creates the three pinned tasks
- prints a `READY` line to the host

If you want to understand how the application is assembled, `main.c` is the entry point.

Jump into code:

- [main/main.c](main/main.c:30) — `app_main()`
- [main/main.c](main/main.c:59) — inter-core queue creation
- [main/main.c](main/main.c:99) — UART0 driver install
- [main/main.c](main/main.c:144) — acquisition task pinning/priority
- [main/main.c](main/main.c:152) — control task pinning/priority
- [main/main.c](main/main.c:160) — processing task pinning/priority

### 2. `main/acquisition.c` and `main/acquisition.h` — Core 0 data acquisition

This subsystem is responsible for getting raw measurements into the system on time.

Key responsibilities:

- configure ADC1 for the three FSR inputs
- configure UART2 for the BNO085
- create the periodic `esp_timer`
- wake on every 10 ms tick
- read sensor values
- estimate IMU angular velocity from Euler differences
- push `raw_sample_t` into the queue

Important things to notice in the code:

- `timer_isr_cb()` gives the semaphore
- `acquisition_task()` blocks on the semaphore
- if the queue is full, the oldest sample is dropped so the system stays live

That last point is important: the code prefers **fresh data** over backlog.

Jump into code:

- [main/acquisition.c](main/acquisition.c:137) — `timer_isr_cb()`
- [main/acquisition.c](main/acquisition.c:247) — `acquisition_init()`
- [main/acquisition.c](main/acquisition.c:307) — `acquisition_task()`
- [main/acquisition.c](main/acquisition.c:336) — per-cycle acquisition steps
- [main/acquisition.c](main/acquisition.c:165) — UART-RVC packet drain/decode helper

### 3. `main/control.c` and `main/control.h` — Core 0 control plane

This subsystem is the low-priority project task that makes the multitasking story
defensible in class terms.

Key responsibilities:

- read UART0 command events from the driver queue
- parse host commands like `I`, `E`, `C1`, `MT_ON`, and `Z`
- coordinate those commands with the Core 1 processing task
- mark the real low-priority control-plane activity that proof mode instruments

This task is necessary even outside proof mode because the host control plane needs a
home that is separate from both the hard real-time acquisition path and the heavier
Core 1 signal-processing path.

Jump into code:

- [main/control.c](main/control.c:50) — `dispatch_command(...)`
- [main/control.c](main/control.c:133) — UART0 event consumption
- [main/control.c](main/control.c:163) — `control_task()`

### 4. `main/processing.c` and `main/processing.h` — Core 1 interpretation

This is the brain of the runtime system.

Key responsibilities:

- receive each `raw_sample_t`
- smooth and filter force and motion signals
- determine contact, engagement, and board state
- compute tremor indicators
- apply warning and error logic
- update the score
- emit JSON telemetry

If you are trying to change scoring, thresholds, warning behavior, or telemetry, this is
the file you will spend the most time in.

Jump into code:

- [main/processing.c](main/processing.c:740) — `processing_init(...)`
- [main/processing.c](main/processing.c:841) — `processing_task()`
- [main/processing.c](main/processing.c:888) — tremor band-pass filtering
- [main/processing.c](main/processing.c:954) — engagement and board-state logic
- [main/processing.c](main/processing.c:1011) — warning/error logic
- [main/processing.c](main/processing.c:1098) — JSON telemetry output

### 5. `main/calibration.c` and `main/calibration.h` — guided personalization

This subsystem captures the reference values that make the runtime logic user-specific.

Current active flow:

- `C1`: still-hand tremor baseline
- `C2`: relaxed-finger pressure baseline
- `C3`: normal light grip
- `C4`: normal hand motion

Important detail: the file still contains some older C3/C4-era legacy structures and
deprecated command responses from earlier experiments. The active user-facing flow is the
four-step version above.

Jump into code:

- [main/calibration.c](main/calibration.c:1039) — `cal_c1(...)`
- [main/calibration.c](main/calibration.c:1136) — `cal_c2(...)`
- [main/calibration.c](main/calibration.c:1217) — `cal_c3_grip(...)`
- [main/calibration.c](main/calibration.c:1253) — `cal_c4_motion(...)`
- [main/calibration.c](main/calibration.c:1302) — `calibration_run(...)`

### 5. `main/filters.c` and `main/filters.h` — signal-processing primitives

This subsystem contains the reusable digital filters. These are not just cosmetic
smoothing filters. They are how the firmware turns noisy physical measurements into
stable interpretable signals.

Examples:

- low-pass force smoothing
- low-pass IMU smoothing
- band-pass 6-12 Hz tremor isolation

Jump into code:

- [`main/filters.c`](main/filters.c)
- [`main/filters.h`](main/filters.h)
- these files define the reusable IIR filter building blocks used by both runtime scoring
  and calibration

### 6. `main/nvs_storage.c` and `main/nvs_storage.h` — persistence

This subsystem reads and writes `cal_params_t` to ESP32 NVS.

Why it matters:

- without it, calibration would be lost on reset
- with it, the glove remembers its references between sessions

Jump into code:

- [`main/nvs_storage.c`](main/nvs_storage.c)
- [`main/nvs_storage.h`](main/nvs_storage.h)
- defaults are initialized in `nvs_get_defaults(...)`
- load/save behavior is implemented in `nvs_load_calibration(...)` and `nvs_save_calibration(...)`

### 7. `main/motor_control.c/.h` and `main/led_status.c/.h` — feedback peripherals

- `motor_control` manages timed motor pulses
- `led_status` controls the status LED behavior

Even though motors are not fully deployed in hardware, this code is still useful to
understand because warning events are already wired to these interfaces.

Jump into code:

- [`main/motor_control.c`](main/motor_control.c)
- [`main/motor_control.h`](main/motor_control.h)
- [`main/led_status.c`](main/led_status.c)
- [`main/led_status.h`](main/led_status.h)

### 8. `gui/esp32_controller.py` — the main desktop application

This is the operator-facing control surface.

It handles:

- serial connection
- calibration wizard
- live metrics and plots
- session scoring display
- command buttons and modes

If you want to change the wording, labels, calibration flow, or GUI behavior, start here.

Jump into code:

- [gui/esp32_controller.py](gui/esp32_controller.py:123) — `CALIBRATION_STEPS`
- [gui/esp32_controller.py](gui/esp32_controller.py:174) — `CORE_CALIBRATION_FLOW`
- [gui/esp32_controller.py](gui/esp32_controller.py:1092) — C3/C4 stage-event handling

### 9. `gui/hand_visualizer.py` — motion/force visualization

This file turns telemetry into a 3D-style hand visualizer. It is part of the "wow factor"
of the project because it makes the otherwise abstract telemetry visible to the user.

Jump into code:

- [`gui/hand_visualizer.py`](gui/hand_visualizer.py)

### 10. `tests/` scripts — focused diagnostics

These scripts are worth treating as part of the documentation:

- `tests/fsr_test.py` helps validate force sensing
- `tests/imu_test.py` helps validate IMU data
- `tests/dual_core_monitor.py` helps validate timing behavior

Jump into code:

- [`tests/fsr_test.py`](tests/fsr_test.py)
- [`tests/imu_test.py`](tests/imu_test.py)
- [`tests/dual_core_monitor.py`](tests/dual_core_monitor.py)

---

## Calibration Guide

Calibration is where the glove learns what is normal for the current user and fit.

### C1 — Still Hand

**User meaning:** keep the hand still.

**Firmware meaning:** measure the no-motion 6-12 Hz tremor-band baseline so later tremor
scoring can compare live motion against a personal stillness reference.

What it stores:

- `tremor_rms_ref`

### C2 — Relaxed Fingers

**User meaning:** do not press the force sensors.

**Firmware meaning:** measure the baseline force distribution and compute contact-on and
contact-off thresholds for each finger.

What it stores:

- `mu[3]`
- `sigma[3]`
- `on_thresh[3]`
- `off_thresh[3]`

### C3 — Normal Light Grip

**User meaning:** hold the glove with a normal light training grip.

**Firmware meaning:** learn a personal grip-force reference so the runtime force thresholds
scale from a meaningful user-specific value instead of a hardcoded assumption.

What it stores:

- `f_ref_open`

### C4 — Normal Hand Motion

**User meaning:** keep a light grip and move the hand naturally.

**Firmware meaning:** learn what intentional movement looks like so the system can separate
normal hand motion from tremor-like motion.

What it stores:

- `f95_ref`
- `pp_roll_ref`
- `pp_pitch_ref`
- `motion_rms_ref`
- `motion_tremor_ratio_ref`

### Why this matters at runtime

After calibration:

- force thresholds are scaled from the learned grip reference
- tremor logic can compare live tremor-band behavior against both:
  - a still-hand baseline
  - a normal-motion baseline

That is what lets the project avoid treating all movement as bad.

### Current public calibration commands

| Command | Meaning |
|---|---|
| `C1` | Run still-hand calibration |
| `C2` | Run relaxed-fingers calibration |
| `C3` | Run normal light grip calibration |
| `C4` | Run normal hand motion calibration |
| `Z` | Erase saved calibration |

There are a few deprecated calibration-related command paths left in the code for legacy
compatibility, but the active supported workflow is `C1` through `C4`.

---

## Scoring and Warning Logic

### The main idea

The runtime logic is trying to answer:

- Is the glove engaged?
- Is the hand holding still or moving?
- Is the force acceptable?
- Is the motion tremor-like?

### Board state

The processing task reduces behavior to four broad states:

- `IDLE`
- `HOLD`
- `ACTIVE`
- `EXITED`

Broadly:

- `IDLE` means not engaged
- `HOLD` means engaged and relatively still
- `ACTIVE` means engaged and moving
- `EXITED` means the tasks were suspended by command

### What affects warnings and errors

The current logic checks several categories:

- hold instability
- tremor
- force opening / excessive force
- sustained compression
- force variability
- force spikes

### What should affect the score

The important conceptual rule is:

> The score should reflect force problems and tremor problems, not normal intended hand motion.

That is why C4 exists.

### Force logic

The code uses:

- per-finger thresholds
- summed force thresholds
- sustained time windows
- force derivative checks for spikes

Mode commands (`E`, `M`, `H`) scale force thresholds using `f_ref_open`.

### Tremor logic

Tremor is not just "motion is large." The code treats tremor as a combination of:

- enough energy above the still baseline
- enough of the motion being concentrated in the 6-12 Hz tremor band

This is a better distinction than raw motion magnitude alone.

### Telemetry cheat sheet

| Field | Meaning |
|---|---|
| `t` | sample timestamp in ms |
| `f0`, `f1`, `f2` | thumb/index/middle force in Newtons |
| `ax`, `ay`, `az` | acceleration in g |
| `gx`, `gy`, `gz` | derived angular velocity in deg/s |
| `roll`, `pitch`, `yaw` | orientation angles in degrees |
| `f_sum` | total grip force |
| `tremor` | live tremor index used for reporting |
| `f95` | 95 percent power frequency estimate |
| `pp_roll`, `pp_pitch` | peak-to-peak motion over the current analysis window |
| `cv_f` | force coefficient of variation |
| `swing` | swing-rate style motion metric |
| `contact` | per-finger contact flags |
| `engaged` | whether the instrument is considered engaged |
| `gate` | whether engagement came from `FSR`, `IMU`, or `NONE` |
| `state` | `IDLE`, `HOLD`, `ACTIVE`, or `EXITED` |
| `warn`, `err` | warning/error bitmasks |
| `score` | running performance score |
| `actual_hz` | firmware-reported nominal/observed sample rate field |

### Warning bitmasks

Defined in `main/data_types.h`:

- `WARN_HOLD_INSTABILITY`
- `WARN_TREMOR`
- `WARN_SMOOTHNESS`
- `WARN_SWING_RATE`
- `WARN_FORCE_OPEN`
- `WARN_FORCE_VARIABILITY`
- `WARN_FORCE_SPIKE`

### Error bitmasks

- `ERR_HOLD_INSTABILITY`
- `ERR_TREMOR`
- `ERR_FORCE_OPEN`
- `ERR_SUSTAINED_COMPRESS`

If you modify the scoring model, this is one of the first places to inspect.

---

## UART Protocol and Telemetry

All host communication uses UART0 at 115200 baud.

### Commands (host -> ESP32)

| Byte/String | Response | Effect |
|---|---|---|
| `I` | `ESP32_TRAINER` | identify device |
| `E` | `EASY` | easy thresholds |
| `M` | `INTERMEDIATE` | medium thresholds |
| `H` | `HARD` | hard thresholds |
| `S` | `STOPPED` | reset runtime state |
| `C1` | JSON | run Step 1 calibration |
| `C2` | JSON | run Step 2 calibration |
| `C3` | JSON | run Step 3 calibration |
| `C4` | JSON | run Step 4 calibration |
| `X` | `EXITED` | suspend tasks |
| `Z` | JSON | erase saved calibration |

### Telemetry rate

The firmware processes samples at 100 Hz but emits JSON every 5th sample, so host telemetry
is approximately 20 Hz.

That is a deliberate bandwidth decision:

- processing stays at 100 Hz
- serial output is reduced so it does not dominate runtime behavior
- the UART TX ring buffer absorbs bursts

### Example telemetry packet

```json
{"t":12340,"f0":0.196,"f1":0.000,"f2":0.000,
 "ax":0.010,"ay":-0.020,"az":1.000,
 "gx":0.10,"gy":-0.20,"gz":0.00,
 "roll":1.2,"pitch":-0.5,"yaw":45.0,
 "f_sum":0.196,"tremor":0.02,"f95":1.6,
 "pp_roll":0.5,"pp_pitch":0.3,"cv_f":0.04,"swing":0.0,
 "contact":[1,0,0],"engaged":1,"gate":"FSR",
 "state":"HOLD","warn":0,"err":0,
 "score":100.0,"actual_hz":100.00}
```

### Timing and asynchronous event sources

The project has three important asynchronous/event-driven paths:

1. `esp_timer` callback -> gives `s_timer_sem`
2. UART0 driver -> posts `UART_DATA` events to `uart_event_q`, consumed by `control_task`
3. BNO085 UART-RVC -> read directly by `acquisition_task` each cycle

That third point matters because the IMU path is **not** currently modeled as a UART2
event queue consumer.

---

## GUI Guide

The GUI is the human-facing side of the project.

### What it does

- connects to the ESP32 serial port
- runs the four-step calibration wizard
- shows live telemetry
- shows warning/error state
- plots force, motion, and score over time
- renders a hand visualization

### What the user sees during calibration

The calibration wizard presents the current supported flow:

1. Still Hand
2. Finger Pressure
3. Normal Light Grip
4. Normal Hand Motion

This is important because the GUI wording and the firmware commands now match. Older
directional wording like `yaw_right`, `pitch_back`, or `figure8` should not be treated
as the active workflow anymore.

### Why the GUI matters technically

The GUI is not just decoration. It is the primary debugging surface for:

- serial transport health
- calibration sequencing
- score evolution
- warning/error interpretation
- validating whether the firmware is reacting to real glove behavior

---

## Build, Flash, and Run

### Prerequisites

- ESP-IDF v6.x installed locally
- Python 3 for the GUI and test tools
- serial access to the ESP32

### Build firmware

```bash
source ~/.espressif/tools/activate_idf_v6.0.sh
python3 ~/.espressif/v6.0/esp-idf/tools/idf.py build
```

### Flash and monitor

```bash
python3 ~/.espressif/v6.0/esp-idf/tools/idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

Exit the monitor with `Ctrl-]`.

### Run the GUI

```bash
pip install -r gui/requirements.txt
python gui/esp32_controller.py
```

### Expected startup behavior

At boot, the firmware should:

- initialize calibration values from NVS or defaults
- create the queue
- start acquisition on Core 0
- start processing on Core 1
- print a ready message over UART0

If you are verifying the dual-core behavior live, watch for the boot log and the task
placement logs immediately after reset.

---

## Diagnostic Tools

These scripts are useful because they isolate subsystems.

### Force sensor test

```bash
python tests/fsr_test.py
python tests/fsr_test.py --list-ports
python tests/fsr_test.py --log
```

Use this when you want to validate FSR behavior without the full GUI.

### IMU test

```bash
python tests/imu_test.py
python tests/imu_test.py --raw
```

Use this when the IMU seems dead, stuck, or misconfigured.

### Dual-core / timing monitor

```bash
python tests/dual_core_monitor.py
python tests/dual_core_monitor.py --list-ports
python tests/dual_core_monitor.py --port /dev/cu.usbserial-XXXX
```

This helps validate timing stability and host-observed telemetry behavior.

### Multitasking proof dashboard

```bash
python3 tests/multitask_proof.py
python3 tests/multitask_proof.py --port /dev/cu.usbserial-XXXX
```

Use this during the presentation to prove the class-definition multitasking path on
Core 0. The script auto-detects a likely ESP32 port by default and shows a short
chooser if several plausible ports are present.

In proof mode, the firmware instruments the real always-on Core 0 `control_task`.
That task already exists in the project because UART command ingress and host control
should not live inside the hard-real-time acquisition loop. Raw proof events are
captured on Core 0, but proof snapshots are assembled and serialized from Core 1 so
the proof mechanism does not distort the Core 0 control task it is trying to prove.
The dashboard shows that this real project task is running, then gets interrupted by
higher-priority timer-dispatch work and `acquisition_task`, and then resumes.

Demo day sequence:

1. flash the current firmware
2. run `python3 tests/multitask_proof.py`
3. let the script send `MT_ON`
4. point to the `Proof Snapshot` block first
5. point to `CTRL -> T -> S -> W -> D -> CTRL`
6. explain that the first `CTRL` means the real low-priority `control_task` was running before the interrupt, and the second `CTRL` means it resumed after acquisition finished
7. exit with `Ctrl+C`, which sends `MT_OFF`

Default dashboard reading order:

- `Proof Snapshot` gives the four key numbers:
  - timer period
  - timer -> acquisition wake latency
  - acquisition runtime
  - acquisition done -> control-task resume latency
- `Proof Health` shows whether any raw trace events or whole cycles were lost
- `Latest Core 0 Timeline` shows the event order for the most recent cycle
- `Status` keeps only the supporting context needed during the presentation

What the event chain means:

- `CTRL -> T -> S -> W -> D -> CTRL`
- first `CTRL` = the real low-priority Core 0 `control_task` was running
- `T` = timer callback begins in the `esp_timer` dispatch context
- `S` = semaphore is given
- `W` = `acquisition_task` wakes and runs
- `D` = acquisition finishes its cycle
- second `CTRL` = the same real Core 0 `control_task` resumed after acquisition

This dashboard is live firmware instrumentation of direct same-core interruption and
resume behavior. It is not a simulated timeline. The period and jitter values shown in
the dashboard come from assembled firmware cycle timing, not from inter-packet arrival
spacing on the host.

If you need the more detailed diagnostic layout, run:

```bash
python3 tests/multitask_proof.py --verbose
```

While proof mode is active, normal runtime JSON is intentionally suppressed so the
UART stream stays clean and presentation-friendly.

---

## How to Modify the Project

This section is written as a teaching recipe. Use it when you want to change behavior
without getting lost.

### Recipe 1 — Change a force or tremor threshold

Start in:

- `main/data_types.h`
- `main/processing.c`

What to look for:

- force threshold constants such as `FORCE_WARN_SUM_N`
- tremor threshold constants such as `TREMOR_MED_WARN_EXCESS_DPS`
- runtime threshold application in `processing.c`

What concept you are changing:

- when the firmware decides a warning or error should happen
- how sensitive the score becomes

What to test afterward:

- does a normal session still avoid false warnings?
- do obvious excessive-force cases still trigger?
- do mode commands `E`, `M`, `H` still scale as expected?

### Recipe 2 — Add a telemetry field

Start in:

- `main/processing.c`
- optionally `gui/esp32_controller.py`

What to do:

1. identify where the value is computed in `processing.c`
2. add it to the `snprintf` JSON block
3. update the GUI parser if the GUI should display it

What concept you are changing:

- host visibility, not sensor acquisition itself

What to test afterward:

- the JSON stays valid
- the GUI does not crash on missing/new fields
- telemetry bandwidth remains reasonable

### Recipe 3 — Adjust calibration timing or strictness

Start in:

- `main/calibration.c`

What to look for:

- sample-count constants
- timeout constants
- force and motion acceptance thresholds

Examples:

- make grip hold longer or shorter
- require more motion for C4
- loosen the stillness requirement for C1

What concept you are changing:

- what the system considers a clean calibration capture

What to test afterward:

- each step still passes for a real user
- noisy or bad calibrations still fail
- runtime scoring still makes sense afterward

### Recipe 4 — Change GUI labels or calibration copy

Start in:

- `gui/esp32_controller.py`

What to look for:

- `CALIBRATION_STEPS`
- status text
- success/failure wording

What concept you are changing:

- user communication, not firmware behavior

What to test afterward:

- the wording matches the firmware commands
- no stale directional terminology remains
- the calibration wizard flow still makes sense visually

### A good rule when modifying anything

Ask two questions:

1. Does this change affect **calibration**, **runtime scoring**, or **telemetry**?
2. Which file owns that concept?

That mental model will save you time:

- acquisition owns sensor sampling
- processing owns runtime interpretation
- calibration owns personal references
- GUI owns wording and visualization

---

## Troubleshooting

### I only see flash output, not the real boot log

The flash output is not the same as runtime monitor output.

To inspect the runtime boot sequence:

1. open `idf.py monitor`
2. reset the board
3. watch the first lines immediately after reset

### IMU fields stay zero

Check:

- wiring to GPIO32
- `P0/PS0` high before power-up
- `P1/PS1` low/unconnected
- common ground
- actual IMU power cycle after wiring changes

### The IMU has power but still sends no useful motion

The LED only proves power, not mode correctness.

Try:

- explicit `P1/PS1` to ground during bring-up
- a full power cycle
- `python tests/imu_test.py --raw` with the serial monitor closed

### Force readings look wrong

Use `tests/fsr_test.py` first. That tells you whether the problem is:

- sensor wiring
- ADC conversion
- force-model assumptions
- higher-level processing

### The GUI or test script cannot connect

Only one program can own the serial port at a time.

Stop:

- `idf.py monitor`
- any other Python serial tool

before launching the GUI or test scripts.

### Calibration feels wrong or stale

Erase saved calibration with `Z` and rerun:

1. `C1`
2. `C2`
3. `C3`
4. `C4`

Do this especially after:

- changing glove fit
- changing users
- changing force thresholds or calibration logic

### `X` was sent and the board stopped responding

That is expected. `X` suspends the tasks. Reset the board to resume normal operation.

---

## Final Advice for Reading the Code

If you are new to the codebase, read it in this order:

1. `main/main.c`
2. `main/data_types.h`
3. `main/acquisition.c`
4. `main/processing.c`
5. `main/calibration.c`
6. `gui/esp32_controller.py`

That order mirrors how the system actually works:

- startup
- shared data definitions
- sampling
- interpretation
- calibration
- presentation

If you keep that mental model, the project becomes much easier to modify without breaking it.
