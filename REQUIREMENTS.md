# Marine Head Vacuum Pump Monitoring Device

## Functional Requirements

### 1. Pump running status reporting

For each pump, report to Signal K whether the pump is running.

Required behavior:

- send `true` when the pump is running
- send `false` when the pump is not running
- Each pump is reported indenpendently
- A different signal k path should be used for each pump.
- The running state should also be re-sent periodically even if it has not changed.
- This periodic state report interval should be configurable via the SensESP UI as one shared setting for both pumps.
- The default periodic state report interval should be 2 seconds.
- Each run cycle should be timed. 
- Report a "last cycle run time" and "last cycle idle time" to signal k each time a cycle status changes


### 2. Device heartbeat

The device should send a heartbeat signal to Signal K to indicate that it is alive and functioning. 

Behavior:
- Use the `millis()` uptime as the value of the heartbeat.
- Report the heartbeat value in seconds, using a reusable SensESP-style producer chain rather than an imperative tick loop.
- The heartbeat report interval should be configurable via the SensESP UI.
- The default heartbeat report interval should be 10 seconds.
- The hearbeat is for the device itself, not for any specific pump.


### 3. Pump function enable/disable

The device should support an "enabled" flag that can be used to enable or disable the pump function. 

Behavior:
- Report this control state to Signal K as a boolean switch state:
- `true` - pump function is enabled
- `false` - pump function is disabled
- Each pump gets its own enabled flag
- Control the pump relay with the inverse of this flag (since relays are "normally closed")
- The flag state should be reported to Signal K at a switch-style path using `*.enabled`.
- The flag state should also be re-sent periodically even if it has not changed.
- This periodic state report interval should use the same shared configurable interval used for the pump running-state reports.
- The flag should also be controllable via a Signal K PUT listener.
- The Signal K metadata for this path should include `supportsPut: true`. Do NOT include `units: "bool"`,
  otherwise compatibility with KIP switches will break.

#### Relay failure detection
A timer should be used to track how long the pump has been disabled.  Report this disabled time to Signal K. If a pump reports that it is running via its digital input **despite** the fact that it is disabled, an alert should be sent to Signal K. Don't consider run signals that occur prior to at least 3 full seconds of disabled time.


### 4. Configurable max run time

The device must support a maximum continuous run time for each pump.

Purpose:
- Detect failure in the vacuum system (e.g. a leak in a seal or hose)
- Prevent overheating and damage to the pump

Behavior:
- if a pump has been detected as running longer than the max run time, the device should auto-disable the pump and send an alert to Signal K
- the interrupt logic should apply independently to each pump


### 5. Configurable cycle min stop time

The device must support a minimum stop time used to determine when a pump cycle has truly completed.

Purpose:
- avoid treating short signal drops or brief pauses as a completed stop/start cycle
- debounce the logical transition from running to stopped

Behavior:
- a pump should only be considered stopped after it has been idle for at least the minimum stop time


### 5. Idle time tracking, Configurable min idle time / short cycle detection
The device must track an idle time between pump cycles and track short cycles 

Purpose:
- Track how long a pump has been idle
- Support the max consecutive short cycles feature

Behavor:
- Each pump gets its own idle timer
- When a pump cycle is completed (stopped for at least the minimum stop time), the idle timer starts
- If a new cycle starts before "min idle time", the short cycle count is incremented
- If the idle timer exceeds "min idle time", the short cycle count is reset to zero
- When a new cycle **starts**, report the "last idle time" to Signal K

### 5a. Cycle statistics and counts
The device must track additional cycle statistics for each pump.

Behavior:
- Track a daily completed cycle count for each pump
- Reset the daily cycle count on the midnight boundary using the device wall clock when valid
- Track the current consecutive short cycle count for each pump
- Track the average completed cycle run time since boot for each pump
- Track the average idle time since boot for each pump
- When computing the average idle time, ignore idle periods longer than 24 hours so unusually long unattended gaps do not distort the vacuum-hold baseline
- Track a daily flush count for each pump
- Track a total flush count for each pump
- Do not reset the averages daily
- Do not reset the total flush count automatically
- A cycle only counts as a flush after it has fully stopped
- Flush detection must use both the idle time measured before the cycle started and the completed cycle run duration
- A cycle only counts as a flush if the prior idle time is at least the configured minimum idle time and the single-cycle run duration is at least the configured minimum flush time
- The minimum flush time should be configurable via the SensESP UI
- Report these values to Signal K on boot, whenever they change, and at minimum once every configured aggregate report interval
- The aggregate report interval should be configurable in seconds via the SensESP UI as one shared setting for both pumps
- The default aggregate report interval should be 10 seconds
- These values should each use their own configurable Signal K path
- The daily flush count and total flush count should persist across reboot


### 6. Configurable Max consecutive short cycles
The device should disable a pump if it has too many consecutive short cycles

Purpose:
- prevent rapid cycling that could damage the pump or reduce its lifespan
- Alert the user of possible system issues

Behavior:
- If a pump's short cycle count exceeds this max consecutive short cycles value:
  - Disable the pump
  - Send an alert to Signal K

### 7. Configurable max cycles and flushes in last 60 minutes
The device should protect each pump against excessive activity within a rolling 60-minute window.

Purpose:
- detect abnormal rapid usage or leak-driven recharge behavior that may not be caught by single-cycle limits alone
- auto-disable a pump when recent cycle or flush activity exceeds expected bounds

Behavior:
- Track a cycle window epoch, cycle window count, and cycle window flush count for each pump
- At the end of each completed cycle:
  - Increment the cycle window count
  - If the completed cycle qualified as a flush, increment the cycle window flush count
  - Compare both counts against their configured maximums
  - If the cycle window count exceeds the configured max cycles in last 60 minutes:
    - Send an alert to Signal K on a dedicated cycles notification path
    - Auto-disable the pump
    - Reset the cycle window epoch and both cycle window counters
  - If the cycle window flush count exceeds the configured max flushes in last 60 minutes:
    - Send an alert to Signal K on a dedicated flushes notification path
    - Auto-disable the pump
    - Reset the cycle window epoch and both cycle window counters
- Once per minute, if the current time minus the cycle window epoch exceeds 60 minutes, reset the cycle window epoch and both cycle window counters
- The max cycles in last 60 minutes should be configurable via the SensESP UI
- The max flushes in last 60 minutes should be configurable via the SensESP UI
- Defaults:
  - max cycles in last 60 minutes = 10
  - max flushes in last 60 minutes = 4


## Behavioral Notes

### Pump state determination

Pump state is based on the corresponding digital input:

- master pump state comes from `D1`
- guest pump state comes from `D2`


### Pump relay control

- master pump relay control is GPIO 16
- guest pump relay control is GPIO 17
- Relay is wired "normally closed" (to fail safe)
- When the relay is activated (GPIO high), the relay opens and interrupts the pump circuit


## Configuration via SensESP UI

- Any item listed as "configurable" should be configurable via the SensESP UI.
- Use the configuration system native to SensESP.

## SensESP Design Guidance

The project should prefer standard SensESP producer / transform / consumer patterns over ad-hoc polling and direct reporting logic.

- A sensor or input should normally be represented by a SensESP producer such as `DigitalInputState` or `RepeatSensor`.
- A value that is computed or tracked internally should normally be represented by a `ValueProducer` / `ObservableValue` rather than a plain member variable when other code may want to subscribe to it.
- Repeated reporting to Signal K should be implemented using a reusable repeat-style transform or producer, rather than manual "last reported" bookkeeping.
- Logic that transforms one value into another should prefer a SensESP transform, including custom transforms when the built-in ones are not sufficient.
- Driving an output pin from another state should prefer normal SensESP wiring, for example producer -> transform -> `DigitalOutput`.
- Signal K output classes should generally be consumers connected in `main.cpp` or in a component `begin()` convenience method, rather than being tightly coupled to the internal sensing logic.
- Metadata should be attached to all custom Signal K paths. Numeric values should specify units. Writable paths should declare `supportsPut` when appropriate.

For "complex components" such as the VacuFlush pump monitor:

- It is acceptable for a class to encapsulate multiple internal SensESP objects and the logic that ties them together.
- Such a class should expose producer-style accessors for its important outputs so other code can subscribe in multiple places.
- Internal logic should update producers via `set()` / `emit()` so all subscribed consumers are notified through the normal SensESP chain.
- Convenience wiring to default `SKOutput*` consumers may be done in the component's `begin()` method, but the underlying values should still be available as producers.

Specific implementation notes discovered during planning and refactoring:

- `DigitalInputState` is a good fit for pump run input, but the externally reported "running" state is a logical state, not always the raw GPIO state. The raw input should be kept separate from the debounced / qualified logical running state.
- The stop detection for this project is asymmetric: a pump should only become logically stopped after `min_stop_time`, so this should not be replaced with a plain symmetric debounce transform.
- The enabled switch state and the running state both need periodic re-reporting to Signal K using one shared configurable interval for both pumps.
- Default Signal K paths for pump-specific values should be derived from the pump role (`master` or `guest`) rather than passed around as many unrelated constants.
- Shared or reusable SensESP enhancements are preferred over one-off workarounds. If a missing primitive is needed, such as a repeat transform whose interval can be updated at runtime, add a reusable local implementation.

## Signal K Output Paths
- All Signal K output paths should have defaults consistent with the Signal K spec
- All paths should be configurable via the SensESP UI.
- Suggested default pump statistic paths are:
- `electrical.pumps.sanitation.vacuum.<role>.dailyCycleCount`
- `electrical.pumps.sanitation.vacuum.<role>.shortCycleCount`
- `electrical.pumps.sanitation.vacuum.<role>.averageCycleRunTime`
- `electrical.pumps.sanitation.vacuum.<role>.averageCycleIdleTime`
- `electrical.pumps.sanitation.vacuum.<role>.dailyFlushCount`
- `electrical.pumps.sanitation.vacuum.<role>.totalFlushCount`


## Signal K Notification Paths (Alarms & Faults)
It is idiomatic in Signal K to use distinct paths for different fault types to allow for granular alerting and metadata.

* Short Cycling Alarm:
* Path: notifications.sanitation.vacuum.cycling
   * Severity: warn
   * Logic: Trigger when pump cycles exceed defined frequency (e.g., losing vacuum due to a seal leak).
* Continuous Run Alarm:
* Path: notifications.sanitation.vacuum.runtime
   * Severity: alarm
   * Logic: Trigger when pump runs longer than a safe timeout (e.g., major leak or mechanical failure).

### Delta Update Implementation
When a fault is detected, the device should emit a JSON delta.
Example: Fail-to-Stop (Continuous Run) Alarm

{
  "updates": [{
    "values": [{
      "path": "notifications.sanitation.vacuum.runtime",
      "value": {
        "state": "alarm",
        "message": "Vacuum pump exceeded max runtime (Possible Major Leak)",
        "method": ["visual", "sound"]
      }
    }]
  }]
}

### Auto-Clear Logic
To clear an alarm once the condition is resolved (or the system is reset), the device should resend the notification on the same path with its `state` set to `normal`.

Example clear payload:

{
  "updates": [{
    "values": [{
      "path": "notifications.sanitation.vacuum.runtime",
      "value": {
        "state": "normal",
        "message": ""
      }
    }]
  }]
}


## Local Reference Patterns
The project should use existing code patterns in this repository as the primary reference for code structure, startup flow, and organization.

As a secondary source of SensESP patterns, refer to the author's SensESP fork in `.pio/libdeps/halmet/SensESP`.

## Signal K Source Of Truth
- The historical Signal K specification repository is useful for general background, terminology, and older examples.
- For current 2.x behavior, metadata conventions, and implementation reality, the Signal K Server source code is the authoritative source of truth.
- When using DeepWiki or other code-aware references, prefer `SignalK/signalk-server` over `SignalK/specification`.

## Initial source tree
- `src/utils` contains timer utilities, including ElapsedTimer, ExpiryTimer, and the ONCE_EVERY macro
- `src/system` contains halmet-specific helper functions and classes

## Debug Logging
- When outputting status, error, or debug messages, use the macros found in `src/Debug.h`
- Deployment of device uses RemoteDebug to manage, so avoid using ESP32 logging directly.

## Notes
This is a boat-installed embedded device, so priorities are:
- reliability
- predictable startup
- simple behavior
- clear fault handling
- easy bench testing before installation
