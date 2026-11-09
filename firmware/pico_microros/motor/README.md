
# TB6612FNG dual motor driver

`tb6612fng.c` / `tb6612fng.h` drive the two TT gearmotors through a
[TB6612FNG](https://www.toshiba.semicon-storage.com/ap-en/semiconductor/product/linear-ics/detail.TB6612FNG.html)
dual H-bridge. Each channel gets a PWM speed input and two direction pins, and
one shared standby pin enables or disables both bridges.

The driver accepts a signed duty from -1.0 to +1.0 per motor. `cmd_vel.c`
sits above it: it subscribes to `/cmd_vel`, mixes `Twist` into left/right
wheel duty, stops the motors when commands time out, and applies a local IR
reflex brake when the obstacle sensor sees something ahead.

## Wiring

Motor pins live in `../config/pinout.h`; electrical and direction settings live
in `../config/motor_config.h`.

| TB6612FNG | Pico 2 W | Notes |
|---|---|---|
| VCC | 3V3 | logic supply, 2.7-5.5 V |
| GND | GND | common ground with the Pico and motor battery |
| VM | battery + | motor supply, 3-15 V, never the Pico 3V3 |
| PWMA | GPIO16 | left PWM |
| AIN1 | GPIO18 | left direction 1 |
| AIN2 | GPIO17 | left direction 2 |
| PWMB | GPIO22 | right PWM |
| BIN1 | GPIO20 | right direction 1 |
| BIN2 | GPIO21 | right direction 2 |
| STBY | GPIO19 | high enables both bridges |
| AO1 / AO2 | left motor | motor winding only |
| BO1 / BO2 | right motor | motor winding only |

The two PWM pins use different slices so each channel can be configured
independently.

```c
#define MOTOR_LEFT_PWM_PIN 16
#define MOTOR_LEFT_IN1_PIN 18
#define MOTOR_LEFT_IN2_PIN 17
#define MOTOR_RIGHT_PWM_PIN 22
#define MOTOR_RIGHT_IN1_PIN 20
#define MOTOR_RIGHT_IN2_PIN 21
#define MOTOR_STANDBY_PIN 19
```

Add a 0.1 uF decoupling capacitor across VCC/GND and a bulk capacitor
(10-100 uF) across VM/GND near the driver. Keep motor leads short and tie the
motor battery ground to the Pico ground. The TB6612FNG includes flyback diodes,
but it does not protect against a reversed supply or a motor connected to the
Pico pins.

## Truth table

The driver uses the standard TB6612FNG control table for each channel:

| STBY | IN1 | IN2 | PWM | Left/right output |
|---|---|---|---|---|
| 0 | x | x | x | outputs disabled (high impedance) |
| 1 | 0 | 0 | x | coast (high impedance) |
| 1 | 1 | 0 | high | forward |
| 1 | 1 | 0 | low | short brake |
| 1 | 0 | 1 | high | reverse |
| 1 | 0 | 1 | low | short brake |
| 1 | 1 | 1 | x | active short brake |

The low part of each PWM period is a short brake, which is the usual
slow-decay drive for these motors. `tb6612fng_set_speed()` applies PWM only
for forward and reverse. A speed near zero sets both direction pins low, which
coasts. `tb6612fng_stop()` selects coast or active brake explicitly.

## Configuration

`config/motor_config.h` is committed; edit it in place. Important values:

| Setting | Default | Meaning |
|---|---|---|
| `MOTOR_PWM_FREQUENCY_HZ` | 20000 | PWM frequency for both channels |
| `MOTOR_MAX_DUTY` | 1.0f | duty limit applied to every command |
| `MOTOR_LEFT_INVERT` / `MOTOR_RIGHT_INVERT` | 0 | flip a side's direction |

20 kHz is above the audible range and well below the TB6612FNG limit. Lower it
(for example to 1000) only if a motor whines or the driver runs warm. The
RP2350 PWM divider limits the minimum to roughly 9 Hz with the default 150 MHz
system clock.

`MOTOR_MAX_DUTY` is clamped to 0.0-1.0. Use it to limit top speed or torque
while tuning; a lower value also reduces battery current and heat.

## Direction calibration

The sign convention depends on how the motor leads are connected and how the
gearbox is mounted. To calibrate:

1. Flash the firmware and open the serial/UDP log.
2. Command a small positive speed on one side (for example from a temporary
   test call in `main`, since there is no ROS command path yet).
3. The robot should move forward for a positive speed on both sides. If a side
   runs backwards, set `MOTOR_LEFT_INVERT` or `MOTOR_RIGHT_INVERT` to `1` in
   `config/motor_config.h`, or swap that motor's two leads.

Because the left and right motors are physically mirrored, one side usually
needs inverting for a positive command to mean "both wheels forward". Match the
sign convention used by `encoder/` and the URDF joint names.

## Usage

```c
#include "motor/tb6612fng.h"

tb6612fng_t motors;

if (!tb6612fng_init(&motors)) {
    // GPIO or pin-configuration problem; check config/pinout.h
}

// Signed duty: -MOTOR_MAX_DUTY .. 0 .. +MOTOR_MAX_DUTY.
tb6612fng_set_speed(&motors, TB6612FNG_MOTOR_LEFT, 0.4f);
tb6612fng_set_speed(&motors, TB6612FNG_MOTOR_RIGHT, 0.4f);

// Coast (both direction pins low, PWM off).
tb6612fng_stop_all(&motors, false);

// Active brake (both direction pins high, PWM off).
tb6612fng_stop_all(&motors, true);

// Drop STBY low; both channels coast and further set_speed() calls fail.
tb6612fng_set_enabled(&motors, false);
```

`tb6612fng_init()` configures STBY low first, then both channels, and only then
raises STBY. It leaves both channels coasting, so the robot does not move until
a speed is commanded.

## API

| Function | Purpose |
|---|---|
| `tb6612fng_init()` | configure both channels and the standby pin |
| `tb6612fng_set_speed()` | apply a signed duty to one motor |
| `tb6612fng_set_enabled()` | enable or disable both bridges through STBY |
| `tb6612fng_stop()` | coast or brake one motor |
| `tb6612fng_stop_all()` | coast or brake both motors |
| `tb6612fng_get_speed()` | last logical speed requested for one motor |

`tb6612fng_set_speed()` returns `false` when the driver is uninitialized or
disabled. `tb6612fng_set_enabled(false)` forces both channels to coast before
dropping STBY, so re-enabling never resumes the previous speed.

## Power and safety

- Power the motors from a separate battery through VM. Do not drive them from
  the Pico 3V3 rail; stall current will brown out the RP2350 and the Wi-Fi
  chip.
- Join the battery ground, the TB6612FNG ground, and the Pico ground.
- STBY is pulled low in `tb6612fng_init()` before either bridge is configured
  and again by `tb6612fng_set_enabled(false)`. Call the latter from any fault
  handler or command watchdog.
- `cmd_vel.c` brakes both motors when no `/cmd_vel` message arrives for
  `CMD_VEL_TIMEOUT_MS` (default 500 ms), so a stalled computer or Wi-Fi link
  cannot leave the robot driving.
- The IR reflex brakes on the Pico itself when an obstacle is within the
  sensor's range, independent of the network. It blocks forward commands.
  Reverse is only allowed in small steps: one step drives back at
  `CMD_VEL_IR_REFLEX_REVERSE_SPEED_M_S` for
  `CMD_VEL_IR_REFLEX_REVERSE_STEP_MS`, then the reflex brakes and pauses for
  `CMD_VEL_IR_REFLEX_REVERSE_PAUSE_MS`. A turn-in-place command is allowed
  during that pause, so the robot can back away a little and pivot around.
  There is no unrestricted reverse. Configure it with `CMD_VEL_IR_REFLEX`,
  `CMD_VEL_IR_REFLEX_REVERSE_STEPS`,
  `CMD_VEL_IR_REFLEX_REVERSE_SPEED_M_S`,
  `CMD_VEL_IR_REFLEX_REVERSE_STEP_MS`, and
  `CMD_VEL_IR_REFLEX_REVERSE_PAUSE_MS` in `../config/cmd_vel_config.h`.
- Verify the actual motor supply voltage, motor lead polarity, and wheel
  geometry before running the robot on the floor.

## Tuning

The geometry and speed defaults live in `../config/cmd_vel_config.h`. Measure
the wheel separation and top speed on your own chassis, then adjust
`CMD_VEL_WHEEL_SEPARATION_M` and `CMD_VEL_MAX_WHEEL_SPEED_M_S`. Set
`CMD_VEL_STOP_BRAKE` to 1 for an active brake on timeout or 0 to coast.

The micro-ROS subscription needs one slot; the current `libmicroros.a` is
built with enough slots for the log, IMU, joint-state, and IR publishers plus
this subscription.
