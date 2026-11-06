# Vibro-Compaction for Legged Locomotion

**Pre-stance vibration for improved quadruped locomotion on granular terrain.**
This project investigates whether a short vibration burst at foot contact can reduce slip and improve forward motion before a robot enters its propulsive stance phase. It combines a modified Buehler-clock gait, a four-actuator quadruped, and experimental analysis on a granular substrate.

**Final paper:** *Pre-Stance Vibro-Compaction Improves Legged Locomotion on Granular Substrates*  
**Authors:** Arad Firouzkouhi, Aryan Barik, and Jacinta Menezes

## Overview
Legged robots can lose traction on loose terrain when the substrate yields beneath their feet. Instead of producing forward motion, the legs displace grains, sink, and slip.

Our approach inserts a brief vibration phase immediately before slow stance. The hypothesis is that this agitation helps the local granular bed settle into a more supportive configuration before the foot applies propulsive shear.

The robot follows three functional gait phases:

1. **Fast swing:** return the leg toward contact.
2. **Pre-stance vibration:** pause the nominal leg angle while applying a sinusoidal angular command.
3. **Slow stance:** push against the substrate to generate forward motion.

The gait period and slow-stance duration remain fixed as vibration duration changes; vibration time is taken from the available fast-swing interval.

## Experimental platform
| Component | Description |
| --- | --- |
| Robot | Four-actuator quadruped |
| Actuators | DYNAMIXEL XL-320 servos |
| Structure | Lego Technic components |
| Leg geometry | Bent legs with an approximately 135-degree interior knee angle |
| Feet | Long, wide, flat feet for increased substrate contact area |
| Substrate | Shallow bed of 6 mm plastic BB pellets |
| Motion measurement | Side-view video tracking of two red chassis masts |

BB pellets provide a controlled granular testbed. They are a surrogate for unstable granular terrain, not natural sand.

## Results
The final paper is the reference for the experimental methods and reported results.

The best tested condition in the base-incline frequency–amplitude sweep was:
| Parameter | Setting |
| --- | --- |
| Vibration frequency | **6 Hz** |
| Commanded angular amplitude | **3 degrees** |
| Vibration duration | **1.4 s** |
| Gait period | **4.0 s** |
| Slow-stance duty factor | **0.45** |

Relative to the no-vibration baseline, this condition produced:
- **39.5% greater displacement per gait cycle.**
- **68% lower slip.**

These results apply to the experiments described in the paper. The condition is the best tested configuration within that study, rather than a universal optimum for every substrate or incline.

## Experiments
| Experiment | Varied parameters | Fixed settings |
| --- | --- | --- |
| Duration sweep | Duration from 0.2 to 1.4 s in 0.2 s increments; frequencies of 6 and 10 Hz | Amplitude of 2.5 degrees |
| Frequency–amplitude sweep | Frequencies of 6, 8, and 10 Hz; amplitudes of 2, 3, and 4 degrees | Duration of 1.4 s |
| Incline sweep | Approximately 2, 10, 15, 20, and 25 degrees; frequencies of 6, 8, and 10 Hz | Amplitude of 3 degrees; duration of 1.4 s |

A no-vibration condition served as the baseline. The primary metrics were displacement per gait cycle and slip; pitch, sinkage, velocity, and course completion time supplied additional diagnostics.

## Firmware and repository layout
The controller was used with different settings during testing. A saved sketch's parameter values represent one configuration, not every experiment in the paper. The documented `VibroCompactionCode.ino` now defaults to the paper's best tested vibration condition; its existing gait and motor-control logic are retained.

The intended layout is shown below. Paths reflect the organization being applied to the repository; some supporting files may still need to be moved or added.

| Path | Purpose |
| --- | --- |
| `VibroCompactionCode/VibroCompactionCode.ino` | Main documented controller, with experiment configuration comments. |
| `archive/Checkpoint2/Checkpoint2.ino` | Earlier controller checkpoint. |
| `archive/Project_Code.txt` | Earlier implementation with different vibration/stance timing. |
| `archive/Lab1_submitted.ino` | Early development sketch with different calibration and motor mapping. |
| `archive/Project-Code-Ckp3.txt` | Suggested archive location for the previous checkpoint 3 snapshot. |
| `positionmode.ino` | Suggested location for the servo-position diagnostic. |
| `data/sample_angles.txt` | Suggested location for the serial log. |
| `docs/final_paper.pdf` | Intended location for the final paper. |

Archive files preserve development history. They are not interchangeable with the main controller or guaranteed to reproduce the final experimental configuration.

Each runnable Arduino sketch belongs in its own folder, with the primary `.ino` filename matching that folder. For the main controller, use **`VibroCompactionCode/VibroCompactionCode.ino`**.

## Requirements
### Hardware
- A compatible Arduino board and DYNAMIXEL Shield.
- Four DYNAMIXEL XL-320 servos with configured IDs.
- The robot's leg and foot assembly.
- A suitable servo power supply and correctly connected DYNAMIXEL bus.

The exact Arduino board model is not recorded in the supplied project files. Select the board actually used in your setup and verify the corresponding serial routing.

### Software
- Arduino IDE and the support package for the selected board.
- ROBOTIS **DynamixelShield** library.
- Its **Dynamixel2Arduino** dependency.

Install the libraries through Arduino IDE's Library Manager. Record the working board package and library versions when validating the hardware setup.

## Getting started
### 1. Clone the repository
```bash
git clone https://github.com/rose08mnzs/Vibro-compaction_Legged_Locomotion.git
cd Vibro-compaction_Legged_Locomotion
```

### 2. Open the controller
Open the following sketch in Arduino IDE after placing the documented controller at that path:
```text
VibroCompactionCode/VibroCompactionCode.ino
```
Keep other complete sketches in their own folders so they are not combined into the same Arduino build.

### 3. Check the hardware configuration
Confirm the motor IDs, directions, and calibration offsets against the physical robot before uploading.

The controller uses:
| Leg index | Leg | Servo ID | Direction flag | Zeroing offset |
| --- | --- | --- | --- | --- |
| 0 | Right front (RF) | 3 | 1 | 52 degrees |
| 1 | Left front (LF) | 2 | 0 | 69 degrees |
| 2 | Left rear (LR) | 30 | 0 | 64 degrees |
| 3 | Right rear (RR) | 4 | 1 | 55 degrees |

The code describes direction flag `0` as CCW and `1` as CW. These values and offsets are specific to the original assembly.

Communication settings:
| Setting | Value |
| --- | --- |
| DYNAMIXEL protocol | 2.0 |
| DYNAMIXEL bus baud rate | 1,000,000 bit/s |
| Debug baud rate | 115,200 baud |

### 4. Compile and upload
1. Select the actual board and serial port in Arduino IDE.
2. Verify the sketch and resolve any board/library compatibility errors.
3. Follow the DYNAMIXEL Shield's UART-switch instructions for uploading and running.
4. Support the robot with its feet clear of the ground for the initial motor-mapping check.
5. Upload the sketch, then verify the direction and calibration of each leg.

The controller enables torque during setup and starts commanding motion automatically. It has no operator start/stop command.

### 5. Read debugging output
Debug output depends on the board branch compiled into the sketch:
| Board branch | Debug interface |
| --- | --- |
| AVR Uno / Mega 2560 | `SoftwareSerial` on pins 7 (RX) and 8 (TX) |
| SAM Due / SAM Zero | `SerialUSB` |
| Other branch | `Serial` |

For the Uno/Mega branch, output is not automatically routed to the standard USB Serial Monitor. Use the connection appropriate to the selected debug interface.

## Experiment configuration
Change the following variables near the top of `VibroCompactionCode.ino`, then compile and upload for each condition.

### Best tested vibration condition
```cpp
bool use_vibration = true;
float vibrate_duration = 1.4;   // seconds
float vibrate_amplitude = 3.0;  // degrees
float vibrate_frequency = 6.0;  // Hz
```

Amplitude is the commanded sinusoidal amplitude, not peak-to-peak motion or a measurement of actual foot displacement.

### No-vibration baseline
```cpp
bool use_vibration = false;
float vibrate_duration = 0.0;
float vibrate_amplitude = 3.0;  // unused with no vibration interval
float vibrate_frequency = 0.0;
```
Set both `use_vibration = false` and `vibrate_duration = 0.0`. Disabling only the flag leaves vibration-dependent timing calculations active.

### Gait timing
```cpp
float phi_s = 1.1;       // slow-phase angular extent, radians
float phi_0 = 2.64;      // slow-phase center angle, radians
float d_c = 0.45;        // slow-stance duty factor
float clock_period = 4;  // seconds
```

The controller computes:
```text
slow_stance_time = d_c * clock_period
fast_swing_time = clock_period - slow_stance_time - vibrate_duration
```

The fast-swing time must remain positive. With the current period and duty factor, vibration duration must be less than **2.2 s**. This is a mathematical limit, not a validated hardware operating limit; longer vibration also requires faster swing return.

### Leg phase ordering
The code preserves the original mapping:
| Leg order | RF | LF | LR | RR |
| --- | --- | --- | --- | --- |
| Phase offset | 0 | 0.5 | 0.25 | 0.75 |

The paper lists `[0, 0.25, 0.5, 0.75]` without explicitly assigning that sequence to the same leg order. Check the experiment records before changing the firmware mapping. Selecting the reported vibration settings alone does not resolve this ordering detail.

## Motion analysis and data
The paper describes a separate side-view computer-vision pipeline using two red chassis masts. It estimates displacement, pitch, front/back sinkage, velocity, and per-cycle performance. Slip is calculated relative to a rigid-ground reference velocity of **0.048 m/s**.

The supplied repository files do not include that analysis implementation or the complete experimental datasets. Servo command logs alone cannot reproduce the paper's displacement and slip measurements.

The controller prints cycle-relative time in seconds, leg index, and desired angle in degrees, mixed with diagnostic text. The printed angles are commands before applying the calibration offset, not measured body or foot motion.

## Limitations and Future Work
- The testbed was manually reset between trials, and raking may not completely remove substrate history.
- The 6 mm BB pellets are an imperfect surrogate for finer natural sand.
- Side-view tracking provides useful motion estimates but does not fully capture three-dimensional body motion or foot–substrate interactions.
- XL-320 servo accuracy, compliance, backlash, and loading can cause actual vibration to differ from commanded motion.
- Increasing vibration duration shortens the available fast-swing interval, introducing actuator timing constraints.
- Future work includes testing finer sand, using more precise actuators, directly measuring substrate yield-force changes, and expanding the parameter sweeps.

## Authors
- Arad Firouzkouhi
- Aryan Barik
- Jacinta Menezes

## Resources
- Final paper: *Pre-Stance Vibro-Compaction Improves Legged Locomotion on Granular Substrates* — intended path: `docs/final_paper.pdf`.
- [DYNAMIXEL Shield documentation](https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/)
- [XL-320 documentation](https://emanual.robotis.com/docs/en/dxl/x/xl320/)
- [Arduino sketch specification](https://docs.arduino.cc/arduino-cli/sketch-specification)
