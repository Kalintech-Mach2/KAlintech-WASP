# WASP

**W**ave-**A**ctivated **S**ub-100g **P**latform

*Unofficial expansions, depending on the day: Wildly Ambitious Student Project. We Asked Simulink Politely.*

A tiny quadcopter you fly by waving at it. It weighs less than 100 grams, has no camera on
board, and gets its orders from a separate hand tracker that does all the watching. A KAlintech
project by Nick, Kalis and Ajmal.

We are three students learning how real aerospace engineering is done: requirements first,
simulation second, buying parts third, and flying a distant fourth. Yes, that is the slow way.
It is also the way that stops us setting fire to our own wallets.

## How it works

```
Your hand -> Pi camera -> Raspberry Pi Zero 2 W (hand tracking)
                                 |
                                 | Wi-Fi (UDP commands)
                                 v
  2S LiPo --> 4-in-1 ESC --> 4 brushless motors
     |            ^ DShot
     +--> 5 V regulator --> ESP32-S3 flight controller <-- IMU
```

The drone's brain is an ESP32-S3 that reads a motion sensor, runs the control loops and tells
four motors what to do. The tracker unit is a separate little computer that watches your hand
and turns gestures into commands. Wave left and the drone goes left. That is the whole idea.
The hard part is the bit in between.

## Status

| Stage | Status |
|---|---|
| Requirements and SysML model | not started (it is on the list) |
| Simulation (MATLAB and Simulink) | in progress: 1-D hover with PID works, Simulink rebuild next |
| CAD (Fusion 360) | not started |
| Gesture tracker (laptop prototype) | not started |
| Hardware bring-up | not started |
| First flight | not started (we would like this one to go well) |

## FAQ

**Why a wasp?** Small, quick, and impossible to ignore once it is in the room.

**Does it fly yet?** Only in MATLAB, where there is no wind, no sensor noise and no cat.

**Why not just use a normal remote control?** We could. That would be much easier. Where is
the fun in that?

**Why is the repo public?** Because a build log with real mistakes in it is worth more than a
polished project that appeared from nowhere.

## How we are working

We model and simulate first and buy parts later, so we only spend money on what the models say
we need. The [build log](docs/build-log) has what we did and what we learned, in order,
including the parts where we were wrong.

## Planned parts

| Part | Choice |
|---|---|
| Motors | 4x 1104 7500KV brushless |
| Propellers | 3-inch 3-blade, 1.5 mm hole (Gemfan Hurricane 3018 style) |
| ESC | 4-in-1, 15 A, DShot, 20x20 mm, no built-in 5 V output |
| Flight controller | ESP32-S3 SuperMini |
| IMU | MPU-6050 |
| Battery | 2S LiPo, 300 to 450 mAh, XT30 |
| Power for the ESP32 | 5 V buck regulator off the battery |
| Tracker | Raspberry Pi Zero 2 W and OV5647 camera (laptop webcam first) |

Targets: total mass under 100 g and a drone parts budget under 100 GBP. Both numbers are
stricter than they sound.

## Repo layout

- `00-requirements/` what it has to do, and why
- `01-simulation/` MATLAB and Simulink models
- `02-cad/` Fusion 360 exports
- `03-firmware/` ESP32 flight controller code
- `04-tracker/` gesture tracker code
- `docs/build-log/` dated notes

## Safety (the serious bit)

Props come off for every bench test. LiPo batteries are charged in a LiPo bag, on a
non-flammable surface, and never left unattended. Nobody waves at it indoors until it has been
tuned, and not at all near people, pets or anything fragile.
