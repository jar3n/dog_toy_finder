# Wiring diagram (text)

Text-based wiring reference for the Dog Toy Finder. Pin numbers are GPIO numbers
for the Pico 2 W unless noted. All grounds must be common.

## Power

```
Battery (motor supply, 3-15 V) ----> TB6612FNG VM
Battery GND -----------------------> TB6612FNG GND, Pico GND (common ground)

Pico 3V3 (pin 36) -----------------> BNO055 VIN, encoder VCC x2, IR sensor VCC,
                                      TB6612FNG VCC (logic), ESP32-CAM 3V3
Pico GND  -------------------------> all GND (common)
```

Do NOT power the motors from the Pico 3V3 rail.

## Pico 2 W <-> BNO055 (I2C1)

```
Pico GPIO14 (phys 19)  --SDA-->  BNO055 SDA
Pico GPIO15 (phys 20)  --SCL-->  BNO055 SCL
Pico 3V3               --VIN-->  BNO055 VIN   (3.3 V only)
Pico GND               --GND-->  BNO055 GND
BNO055 ADDR            --GND-->  (I2C address 0x28)
```

## Pico 2 W <-> wheel encoders (hall quadrature, x2)

```
Left encoder:
  Pico GPIO7  (phys 10) <-- A --  encoder A  (open-drain, internal pull-up)
  Pico GPIO6  (phys 9)  <-- B --  encoder B
  Pico 3V3              --> VCC
  Pico GND              --> GND

Right encoder:
  Pico GPIO9  (phys 12) <-- A --  encoder A
  Pico GPIO8  (phys 11) <-- B --  encoder B
  Pico 3V3              --> VCC
  Pico GND              --> GND
```

## Pico 2 W <-> TB6612FNG

```
Pico 3V3   --> TB6612 VCC     (logic)
Battery +  --> TB6612 VM      (motor supply)
Pico GND   --> TB6612 GND     (common ground)

Pico GPIO16 --> TB6612 PWMA   (left PWM)
Pico GPIO18 --> TB6612 AIN1   (left dir 1)
Pico GPIO17 --> TB6612 AIN2   (left dir 2)

Pico GPIO22 --> TB6612 PWMB   (right PWM)
Pico GPIO20 --> TB6612 BIN1   (right dir 1)
Pico GPIO21 --> TB6612 BIN2   (right dir 2)

Pico GPIO19 --> TB6612 STBY   (high = enabled)

TB6612 A1/AO1 --> left motor  +   ("1" output = positive, see below)
TB6612 A2/AO2 --> left motor  -   ("2" output = negative)
TB6612 B1/BO1 --> right motor +   ("1" output = positive, see below)
TB6612 B2/BO2 --> right motor -   ("2" output = negative)
```

### Motor outputs A1/A2 and B1/B2 -> motor leads

Each TB6612 H-bridge has two motor outputs. Many breakout boards print them as
`A1`/`A2` and `B1`/`B2`; the datasheet names are `AO1`/`AO2` and `BO1`/`BO2`
(identical terminals). Channel A must drive the **left** wheel and channel B the
**right** wheel — the firmware's differential mix assumes this.

| Driver output | Datasheet name | Connects to | Motor lead |
|---|---|---|---|
| A1 | AO1 / OA1 | left motor  | **+** |
| A2 | AO2 / OA2 | left motor  | **-** |
| B1 | BO1 / OB1 | right motor | **+** |
| B2 | BO2 / OB2 | right motor | **-** |

**Why the "1" output is positive:** with STBY high, the firmware commands
forward by setting IN1 = 1 and IN2 = 0 (PWM on). Per the TB6612 truth table that
forces OUT1 high and OUT2 low, so current flows OUT1 -> motor -> OUT2. Wiring
each motor's **+** lead to its "1" output (A1 or B1) and its **-** lead to the
"2" output (A2 or B2) makes a positive `cmd_vel` spin that wheel forward.

If a motor's leads are not marked +/-, identify them before wiring: the "+"
lead is the one that spins that wheel in the robot's forward direction when a
positive voltage is applied to it relative to the other lead. Swapping the two
leads of a motor reverses that wheel's direction.

## Pico 2 W <-> KS0051 IR obstacle sensor

```
Pico 3V3   --> sensor VCC
Pico GND   --> sensor GND
Pico GPIO4  <-- sensor S  (digital signal, active high)
Pico GPIO5  --> sensor EN (high enables)
```

## ESP32-CAM (each eye, x2)

```
ESP32-CAM 3V3 --> 3V3
ESP32-CAM GND --> GND
(no Pico wiring; talks over Wi-Fi)
```

## Pin summary (Pico 2 W)

| GPIO | Function |
|---|---|
| 4 | IR obstacle signal (S) |
| 5 | IR obstacle enable (EN) |
| 6 | Left encoder B |
| 7 | Left encoder A |
| 8 | Right encoder B |
| 9 | Right encoder A |
| 14 | BNO055 SDA (I2C1) |
| 15 | BNO055 SCL (I2C1) |
| 16 | Motor PWMA |
| 17 | Motor AIN2 |
| 18 | Motor AIN1 |
| 19 | Motor STBY |
| 20 | Motor BIN1 |
| 21 | Motor BIN2 |
| 22 | Motor PWMB |
| 23-25, 29 | Reserved (wireless chip) |

## Built-in status LED

No wiring and no Pico GPIO: on the Pico 2 W the LED belongs to the CYW43
wireless chip (wireless GPIO 0). The firmware drives it through the cyw43 API;
it blinks while starting up, stays solid once every device is up, and is off
when the boot sequence failed. See
`firmware/pico_microros/status_led/README.md`.
