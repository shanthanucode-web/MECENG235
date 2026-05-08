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

- "Is this just a sensor demo?"
- "Is the GUI superficial?"
- "Where is the actual systems design?"

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

- "Why split across cores at all?"
- "Why use a queue instead of globals?"

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

- "Are these comments, or is there real logic there?"

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

- "How do you know it is 100 Hz and not just approximately 100 Hz?"
- "Is the 20 Hz JSON stream your real sample rate?"
- "What exactly is the deadline?"

## 5. Interrupts and Asynchronous Sources

The three important asynchronous sources in the current implementation are:

1. `esp_timer`
2. UART0 RX events from the host
3. UART2 IMU serial packet arrival/read path

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

**What to say**

The project has one periodic timing source and two serial asynchronous sources:
the timer, UART0 host commands, and UART2 IMU traffic.

**What proves it**

Timer setup, UART0 event queue installation, and IMU UART reads are all visible
in the code.

**What they may challenge**

- "Is a stop command an interrupt?"
- "If UART2 is read in the acquisition task, why do you call it asynchronous?"

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

- "Is `control_task` a fake task made only for the demo?"
- "How do you know the proof tool is not inventing the timeline?"

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

- "Why not do everything on one core?"
- "Why not share globals instead of copying samples through a queue?"

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

- "Could you have done this without FreeRTOS?"
- "What did the RTOS actually buy you?"

## 8.1 Task Priorities and Why They Were Chosen

Task priority is a design choice about urgency, not just a number assignment. In
FreeRTOS, if two tasks on the same core are ready to run, the higher-priority
task runs first. So priorities are how this project decides which work must win
when timing matters.

- **Implementation fact**: the task creation block and priority rationale are in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:113).

### The actual priorities in this project

- `esp_timer` dispatch task: priority `22`
- `acquisition_task`: priority `10`
- `processing_task`: priority `9`
- `control_task`: priority `1`

- **Implementation fact**: the code comments in
  [main/main.c](/Users/shanthanu/uart_echo_VitalSignsLab4/main/main.c:121)
  explicitly describe those choices.

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

- “Why 10 instead of 5?”
- “Why is control only 1?”
- “Why is processing 9?”
- “What would change if these priorities were swapped?”

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

- "Why Butterworth instead of a moving average?"
- "Why band-pass 6-12 Hz?"
- "Are you actually implementing the filter yourselves?"

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

- "Who decided 100 Hz?"
- "Why not sample slower or faster?"

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

- "Why not binary?"
- "Is JSON too expensive for an embedded system?"

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

- "Why not LabVIEW if this is instrumentation-heavy?"
- "What do you gain technically from Python?"

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

- "What exactly is calibrated?"
- "Why not use fixed thresholds for everyone?"

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

- "How are thresholds actually computed?"
- "What is learned versus fixed?"
- "Why are difficulty modes meaningful?"

## 15. Likely Challenge Questions

### Real-time and timing

**Q: Why is this real-time?**  
Because acquisition is a periodic job with a 10 ms deadline, driven by a timer
and semaphore path, not by "run whenever convenient."

**Q: How do you prove 100 Hz?**  
Show the `esp_timer_start_periodic(..., 10000)` call, the acquisition timing log,
the proof pin toggle, and the timing monitor.

**Q: Is the 20 Hz JSON stream your sample rate?**  
No. JSON is decimated telemetry. The internal acquisition and processing cadence
is 100 Hz.

### Multitasking and dual-core

**Q: Where exactly is the multitasking?**  
On Core 0: `control_task` is the low-priority task, and timer/acquisition work
preempts it every 10 ms.

**Q: How do you know the multitasking proof is not fake?**  
Because the low-priority task is a real project task, and the proof dashboard
displays live firmware instrumentation assembled from raw event timestamps.

**Q: What is the difference between multitasking and dual-core here?**  
Multitasking is same-core preemption on Core 0. Dual-core is the Core 0 / Core 1
parallel split with a queue between them.

### DSP and sensing

**Q: Why Butterworth?**  
Smooth passband, practical IIR implementation, efficient on embedded hardware,
and appropriate for smoothing and tremor isolation.

**Q: Why 6-12 Hz tremor band?**  
Because the current design uses that band as the tremor indicator, and the
runtime and calibration code explicitly shape features around it.

**Q: Why 100 Hz instead of 50 Hz?**  
100 Hz gives more comfortable headroom for the 6-12 Hz motion band, better time
resolution, and the codebase is already designed and budgeted around it.

### Host and tooling

**Q: Why Python instead of LabVIEW?**  
Because the host side needed a GUI, proof dashboard, diagnostics, serial parsing,
and source-reviewable tooling around a JSON protocol. Python unified all of that
in one codebase.

**Q: Why JSON if bandwidth is limited?**  
Because readability and inspectability mattered on the host boundary, and the
design controls bandwidth by throttling normal telemetry to 20 Hz.

### Calibration and thresholds

**Q: What exactly is calibrated?**  
Gyro bias, neutral pose, no-pressure force baseline, contact thresholds, grip
reference force, tremor baseline, and normal-motion references.

**Q: What is fixed versus personalized?**  
Personalized values come from calibration. Difficulty constants and some floors
come from fixed configuration. Runtime logic combines both.

### Limitations and next steps

**Q: What would you improve next?**  
A stronger answer is: validate thresholds on more users, formalize empirical
tuning, consider binary or packed telemetry if bandwidth becomes limiting, and
continue integrating motor feedback as a complete closed-loop coaching path.

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
