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
  Pico GPIO6  (phys 9)  <-- A --  encoder A  (open-drain, internal pull-up)
  Pico GPIO7  (phys 10) <-- B --  encoder B
  Pico 3V3              --> VCC
  Pico GND              --> GND

Right encoder:
  Pico GPIO8  (phys 11) <-- A --  encoder A
  Pico GPIO9  (phys 12) <-- B --  encoder B
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

TB6612 AO1/AO2 --> left motor
TB6612 BO1/BO2 --> right motor
```

## Pico 2 W <-> KS0051 IR obstacle sensor

```
Pico 3V3   --> sensor VCC
Pico GND   --> sensor GND
Pico GPIO10 (phys 16?) <-- sensor S  (digital signal, active high)
Pico GPIO11            --> sensor EN (high enables)
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
| 6 | Left encoder A |
| 7 | Left encoder B |
| 8 | Right encoder A |
| 9 | Right encoder B |
| 10 | IR obstacle signal (S) |
| 11 | IR obstacle enable (EN) |
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