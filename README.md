# Marine Head Vacuum Pump Monitoring Device

Firmware for a Hatlabs Halmet board that monitors and protects two Dometic VacuFlush vacuum pump circuits on a boat.

## Overview

This device monitors whether each vacuum pump is running and reports that status to Signal K.

It also provides a protective interrupt feature that can stop a pump if it has been running too long without establishing vacuum.

The two monitored pumps are:

- Master head vacuum pump
- Guest head vacuum pump

## Hardware

### Inputs

The Halmet uses two digital inputs to detect pump activity:

- `D1` = master head pump running sense
- `D2` = guest head pump running sense

Each input should report whether the corresponding pump is currently running.

### Outputs

The Halmet uses two GPIO outputs to control pump interrupt relays:

- `GPIO 16` = master head interrupt control
- `GPIO 17` = guest head interrupt control

Each pump is wired through an independent **normally-closed** relay.

Relay behavior:

- GPIO output **low** = corresponding pump circuit remains enabled
- GPIO output **high** = corresponding relay opens and interrupts pump operation

Because of this wiring, the software-facing concept of `"enabled"` is the inverse of the physical relay control signal.

## Problem Being Solved

Each Dometic VacuFlush head uses a vacuum pump and vacuum switch.

Under normal operation, the pump runs until the proper vacuum level is established, then the vacuum switch turns the pump off.

If vacuum cannot be established or held, the pump may:

- run continuously, or
- cycle too frequently

Possible causes include:

- debris in a seal
- seal failure
- leaks in the vacuum system
- other faults that prevent vacuum from being maintained

This device is intended to:

- monitor pump run state
- report run state to Signal K
- interrupt a pump cycle when it exceeds a configured maximum run time
