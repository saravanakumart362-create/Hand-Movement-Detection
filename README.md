# Hand Mobility Assessment Glove

**Arduino-Based Real-Time Hand Movement Detection and Alert System for Coma Patients**

A wearable glove with an IMU on the back of the hand. An Arduino reads the sensor over I²C 50 times a second, works out how much the hand is moving and how it is tilted, and sounds a buzzer and lights an LED when the movement crosses a threshold. Each movement is labelled short, long or shaking, and small or big.

## What is in this folder

| Path | What it is |
|---|---|
| `index.html` | Live demo page. Simulates the glove in the browser, or shows a real glove connected over USB. |
| `firmware/HandMobilityGlove/HandMobilityGlove.ino` | The Arduino sketch. Needs only the built-in `Wire` and `EEPROM` libraries. |
| `docs/TECHNICAL_DOCUMENT.md` | Full technical document with six diagrams: design, I²C, firmware, detection, testing and limitations. |
| `README.md` | This document. |

## Try the live demo

**Live demo: <https://saravanakumart362-create.github.io/Hand-Movement-Detection/>**

The link opens the demo in any browser. To run it offline, open `index.html` (double-click it). No hardware or install is needed.

The page starts with a simulated glove that is already monitoring. From there:

1. Press **Finger twitch**. The alert fires within a fraction of a second and clears 3 seconds after the hand is still.
2. Press **Faint finger flex**. It is missed. Lower the rotation threshold to 1.0 °/s and press it again.
3. Raise **Sensor noise** to 6 × typical. False alerts start. Raise the thresholds until they stop.
4. Drag the hand to one side and hold it. The alert clears, because a tilted hand that is still is not moving.
5. Press **Tremor**, then **Wrist turn**. The log names the first Shaking and the second a Long movement, and the bigger one beeps faster.

What the page shows:

- **The glove**: a hand you can drag to tilt, six movements to play (finger twitch, faint finger flex, tremor, wrist turn, hand lift, bed bump), and the hand's tilt as roll and pitch angles.
- **The alarm**: monitoring state, the movement's type and strength, LED and buzzer, event count, and the caregiver button.
- **Signal chain**: the five stages of the system with live values, including the 14 bytes read over I²C.
- **Live traces**: raw accelerometer and gyroscope axes, and the two processed movement values against their thresholds.
- **Detection settings**: sliders for the thresholds and timing. Changes are saved automatically.
- **Movement log**: every detected event with its time, type, strength, duration, peak values and which sensor tripped it. **Download as a file** saves the log as a spreadsheet file (CSV).

The simulated glove runs the same detection steps, with the same constants, as the Arduino sketch.

## Build the glove

### Parts

- Arduino Uno or Nano
- MPU-6050 IMU breakout (GY-521)
- Active buzzer (5 V)
- LED and a 220 Ω resistor (the on-board LED on pin 13 also works)
- Push button (optional, the caregiver button)
- Glove, jumper wires, USB cable or a 5 V supply
- Optional: a second MPU-6050 for a finger

### Wiring

| Part | Pin on the part | Arduino pin |
|---|---|---|
| MPU-6050 (GY-521) | VCC | 5V |
| | GND | GND |
| | SDA | A4 |
| | SCL | A5 |
| Active buzzer | + | D8 |
| | − | GND |
| LED | anode, through 220 Ω | D13 |
| | cathode | GND |
| Push button (optional) | one leg | D2 |
| | other leg | GND |

Fix the MPU-6050 flat on the back of the hand so that it cannot shift inside the glove. The demo page draws the sensor's X, Y and Z axes on the hand.

A second MPU-6050 on a finger is wired to the same four pins, with its AD0 pin tied to 3.3 V so that it answers at address 0x69. The sketch finds it at power-up and uses whichever sensor reports the larger movement.

### Upload the sketch

1. Open `firmware/HandMobilityGlove/HandMobilityGlove.ino` in the Arduino IDE.
2. Choose the board (Arduino Uno or Nano) and its port.
3. Upload.
4. Lay the gloved hand still. The buzzer gives a short beep, the LED stays on for two seconds while the sensor calibrates, then goes off. The glove is now monitoring.

For a passive piezo buzzer, set `BUZZER_IS_PASSIVE` to `1` near the top of the sketch.

### What the LED and buzzer mean

| State | LED | Buzzer |
|---|---|---|
| Calibrating (2 s after power-up) | on | silent |
| Monitoring | off | silent |
| Movement detected, small | on | slow beeps: one short beep every 0.8 s |
| Movement detected, big | on | fast beeps: four a second |
| Paused for care | short blink every 2 s | silent |
| Sensor fault (IMU not answering) | fast blink | chirp every 2 s |

### The caregiver button

Pressing the button on D2 silences an alert and pauses monitoring for 30 seconds, so that attending the patient does not set the alarm off. Pressing it again resumes monitoring straight away.

### What the glove reports about each movement

| Report | Values | How it is decided |
|---|---|---|
| Type | Short, long or shaking | Short is under half a second. Long is half a second or more. Shaking lasts at least a second and changes direction 6 times or more. |
| Strength | Small or big | Big means a movement value reached 4 times its threshold. |
| Tilt | Roll and pitch, in degrees | Worked out from where gravity points on the IMU sensor. Roll is the tilt to the side; pitch is the fingers lifting or dropping. |

### Settings are saved in the glove

Settings changed from the demo page or by serial command are saved in the Arduino's permanent memory two seconds after the last change. They are still there after the glove is switched off and on. The command `D`, or **Reset to defaults** on the page, goes back to the values in the sketch.

### Connect the real glove to the demo page

1. Open the [live demo](https://saravanakumart362-create.github.io/Hand-Movement-Detection/) or `index.html` in Chrome or Edge on a computer.
2. Close the Arduino Serial Monitor if it is open.
3. Press **Arduino over USB** and pick the Arduino's port.

The page now shows the real glove's readings, tilts the drawn hand to match it, and sets its sliders to the settings the glove is using. Moving a slider changes that setting on the glove at once, and the glove saves it.

---

## 1. Project Overview

The Hand Mobility Assessment Glove is a wearable embedded system developed to monitor and detect hand movements in coma patients. The main objective is to identify physical hand movements and automatically generate an alert to notify caregivers or healthcare professionals.

Patients in a coma may exhibit involuntary, reflexive, or spontaneous movements. Continuous observation is important, but healthcare professionals cannot always monitor every patient movement manually.

To address this, the project uses an **Arduino microcontroller and Inertial Measurement Unit (IMU) sensors** integrated into a wearable glove.

The IMU sensors measure acceleration and angular movement. Arduino continuously collects the sensor readings through the **I²C communication protocol**, processes the movement data, and determines whether significant hand movement has occurred.

When the measured motion exceeds a predefined detection threshold, the system activates an alert mechanism.

This allows caregivers to be informed when movement is detected without requiring continuous direct observation of the patient's hand.

## 2. Project Objectives

The primary objectives are:

- Develop a wearable glove for continuous hand movement monitoring.
- Use IMU sensors to measure acceleration and rotational movement.
- Interface IMU sensors with Arduino using I²C communication.
- Process sensor readings to distinguish meaningful movement from normal sensor fluctuations.
- Detect hand movement in real time.
- Automatically trigger an alert when movement is detected.
- Provide a lightweight, low-cost embedded monitoring solution.

## 3. Hardware Components

### Arduino Microcontroller

Arduino acts as the central processing unit of the system.

Its responsibilities include:

- Initializing the IMU sensors.
- Continuously reading sensor measurements.
- Processing acceleration and gyroscope data.
- Evaluating changes in hand motion.
- Checking whether the detected movement exceeds a predefined threshold.
- Controlling the connected alert mechanism.

### IMU Sensors

An IMU is an electronic sensor module that measures motion and orientation. This project uses the MPU-6050, which contains two sensors:

**Accelerometer**

Measures acceleration along three axes:

- X-axis: across the hand.
- Y-axis: toward the fingers.
- Z-axis: out of the back of the hand.

The accelerometer helps identify movement and changes in the orientation of the glove. It is set to a range of ±2 g.

**Gyroscope**

Measures angular velocity around the same three axes, at a range of ±250 °/s.

This helps detect rotation, tilting, and changes in hand orientation.

By combining accelerometer and gyroscope information, the system can detect hand movements more effectively than by using a single measurement alone.

### Wearable Glove

The glove provides a wearable platform for attaching the sensor hardware to the patient's hand.

Its main purpose is to maintain a consistent sensor position while allowing motion measurements.

### Alert Mechanism

A buzzer on pin D8 and an LED on pin D13 are activated whenever the motion detection condition is satisfied.

### Power Supply

USB or a 5 V supply powers the Arduino, which in turn powers the sensor, buzzer and LED.

## 4. System Architecture

The system operates through five main stages:

**Stage 1 – Motion Sensing**

The IMU mounted on the glove continuously measures hand acceleration and angular velocity.

**Stage 2 – Data Acquisition**

The Arduino reads measurements from the IMU sensor through I²C communication.

**Stage 3 – Data Processing**

The received measurements are analyzed to identify changes in movement and orientation.

**Stage 4 – Movement Detection**

The processed motion measurements are compared against predefined detection thresholds.

**Stage 5 – Alert Generation**

If movement exceeds the configured threshold, the Arduino activates the alert output to notify caregivers.

The demo page shows these five stages side by side with their live values.

## 5. I²C Communication

The system uses the Inter-Integrated Circuit (I²C) protocol to establish communication between the Arduino microcontroller and the IMU sensor.

I²C is a synchronous serial communication protocol commonly used in embedded sensor systems.

It uses two main communication lines:

- **SDA (Serial Data):** Transfers sensor measurements and configuration information. Arduino pin A4.
- **SCL (Serial Clock):** Synchronizes communication between the Arduino and IMU. Arduino pin A5.

Arduino requests data from the IMU, receives the acceleration and angular velocity measurements, and uses the information for real-time motion analysis.

The protocol reduces wiring complexity and supports multiple compatible devices on the same communication bus, provided their I²C addresses do not conflict.

What the sketch sends and reads, at 400 kHz:

| Register | Value | Purpose |
|---|---|---|
| `0x6B` PWR_MGMT_1 | `0x01` | Wake the sensor |
| `0x19` SMPLRT_DIV | `19` | 50 samples a second |
| `0x1A` CONFIG | `0x04` | Built-in low-pass filter, about 20 Hz |
| `0x1B` GYRO_CONFIG | `0x00` | ±250 °/s, 131 counts per °/s |
| `0x1C` ACCEL_CONFIG | `0x00` | ±2 g, 16384 counts per g |
| `0x3B` ACCEL_XOUT_H | read 14 bytes | Acceleration X Y Z, temperature, rotation X Y Z, two bytes each, high byte first |

## 6. Working Principle

The glove operates continuously while being worn by the patient.

**Step 1: System Initialization**

When the Arduino is powered on, it initializes communication with the IMU sensors.

The program configures the sensors and calibrates for two seconds with the hand still: the average gyroscope reading becomes the bias to subtract, and the average accelerometer reading becomes the starting estimate of gravity. If the hand moves during calibration, it starts again.

**Step 2: Continuous Monitoring**

The IMU continuously measures the position-related motion of the patient's hand.

Even when the hand is stationary, the accelerometer reports gravitational acceleration and small amounts of sensor noise.

**Step 3: Sensor Data Processing**

Arduino reads the three-axis accelerometer and gyroscope values.

The measurements are processed into two movement values (see section 7), one from each sensor.

**Step 4: Motion Threshold Evaluation**

Each movement value is compared with its threshold.

If both stay under their thresholds, the system continues normal monitoring.

If either stays above its threshold for several readings in a row, the system identifies a movement event.

**Step 5: Alert Activation**

When a valid movement event is identified, Arduino turns the LED on and beeps the buzzer.

The notification informs the caregiver that the patient's hand has moved.

**Step 6: Continued Monitoring**

The alert stays on for a few seconds after the hand is still, so that a burst of movement counts as one event rather than many. The device then continues monitoring for additional movement.

## 7. Movement Detection Algorithm

Every 20 ms the sketch does the following:

1. Read acceleration `a` (in g) and rotation `w` (in °/s, bias removed).
2. **Acceleration change** = the size of `a − gravity`, where `gravity` is a slow average of `a`. After the subtraction, `gravity` moves 4% of the way toward `a`. A hand that is tilted but still reads zero, because the average catches up with the new tilt in about a second.
3. **Rotation rate** = the size of `w` across the three axes.
4. Smooth both values (each moves 25% of the way toward its new reading), which removes single-sample spikes.
5. If the acceleration change is above `accelThresholdG`, or the rotation rate is above `gyroThresholdDps`, count the reading. Otherwise reset the count.
6. When the count reaches `confirmSamples`, the hand is moving: start the alert.
7. End the alert once `alertHoldMs` has passed with no movement.

While an alert is on, the sketch also labels the movement:

- **Type.** Short if the movement has lasted under half a second, long if half a second or more. It becomes shaking once it has lasted a second and changed direction 6 times or more on any axis.
- **Strength.** Small at first. It becomes big when either movement value reaches 4 times its threshold. A big movement beeps faster.

On every reading the sketch also works out the **tilt** of the hand from the gravity estimate: roll = atan2(−gx, gz) and pitch = atan2(gy, √(gx² + gz²)), in degrees, where gx, gy and gz here are the three parts of the gravity estimate.

### Settings

| Setting | Default in the sketch | Starting value | Meaning |
|---|---|---|---|
| `accelThresholdG` | `DEFAULT_ACCEL_THRESHOLD_G` | 0.030 g | Acceleration change that counts as movement |
| `gyroThresholdDps` | `DEFAULT_GYRO_THRESHOLD_DPS` | 3.0 °/s | Rotation rate that counts as movement |
| `confirmSamples` | `DEFAULT_CONFIRM_SAMPLES` | 3 (60 ms) | Readings in a row needed to confirm movement |
| `alertHoldMs` | `DEFAULT_ALERT_HOLD_MS` | 3000 ms | How long the alert stays on after movement stops |
| `carePauseMs` | `DEFAULT_CARE_PAUSE_MS` | 30000 ms | How long the caregiver button pauses monitoring |

A changed setting is saved in the Arduino's permanent memory (EEPROM) and used at the next power-up. Editing a `DEFAULT_` value in the sketch and uploading it again discards the saved settings, so the new defaults take effect.

The threshold must be selected carefully. A threshold that is too low can cause false alerts from sensor noise, while a threshold that is too high may fail to detect small movements. The demo page lets you try this: with typical sensor noise, the resting values sit near 0.005 g and 0.1 °/s, so the defaults leave a wide margin.

For dependable monitoring, the system should also distinguish actual patient movement from accidental glove disturbance or movement during caregiving. This project does not do that automatically. The caregiver button covers movement during caregiving; a bumped bed still trips the alert.

### Serial output and commands

The sketch prints one line per reading at 115200 baud, which the Arduino Serial Plotter and the demo page both understand:

```
ax:0.012,ay:-0.004,az:1.002,gx:0.10,gy:-0.20,gz:0.00,am:0.004,gm:0.21,roll:0.7,pitch:-0.2,st:0,ty:0,lv:0
```

`ax ay az` are acceleration in g, `gx gy gz` rotation in °/s, `am` the acceleration change, `gm` the rotation rate, `roll` and `pitch` the tilt in degrees, and `st` the state (0 monitoring, 1 alert, 2 paused, 3 calibrating, 4 sensor fault). During an alert, `ty` is the movement type (1 short, 2 long, 3 shaking) and `lv` its strength (1 small, 2 big).

Lines starting with `#` are messages. When an alert ends, one of them sums it up:

```
# event 3 ended: long movement, big, lasted 1.9 s, peaks 0.306 g and 80.1 deg/s
```

Commands, one per line: `A0.030` and `G3.0` set the two thresholds, `C3` the readings to confirm, `H3000` the alert hold in ms, `P30000` the care pause in ms, `K` presses the caregiver button, `R` recalibrates, `S` saves the settings at once, `D` goes back to the defaults in the sketch, and `?` prints the settings.

## 8. Example Scenario

Consider a patient in a hospital who is unconscious and undergoing observation.

A Hand Mobility Assessment Glove is attached to the patient's hand.

Initially, the hand remains stationary. The Arduino continuously collects IMU readings and determines that no significant movement has occurred.

After some time, the patient's hand unexpectedly moves.

The IMU senses the change in acceleration or rotation.

Arduino processes the sensor data and identifies that the movement exceeds the configured detection threshold.

The system then activates its alert output.

A caregiver can respond to the notification and assess the patient's condition.

The device does not diagnose the cause of the movement or determine whether consciousness has returned. It serves as a movement detection and notification aid.

## 9. Applications

- **Coma patient monitoring:** Detecting hand movements during patient observation.
- **Hospital patient monitoring:** Providing movement notifications to caregivers.
- **Neurological observation:** Supporting the recording or recognition of physical movement events.
- **Assistive healthcare systems:** Serving as a sensing component in broader patient monitoring applications.

## 10. Advantages

- Real-time detection of hand movement.
- Automatic alerts when movement is identified.
- Low-cost microcontroller-based implementation.
- Compact wearable design.
- I²C-based sensor integration.
- Continuous monitoring without requiring constant manual observation of the hand.
- Potential for integration with additional monitoring or notification systems.

## 11. Limitations

The glove detects physical motion but cannot identify whether movement is voluntary, involuntary, or reflexive.

It cannot independently determine the patient's consciousness level, neurological recovery, or medical condition.

A hand-mounted IMU may also fail to detect very small individual finger movements. Such movements may require additional finger-mounted sensors. The sketch supports one extra sensor on a finger.

Sensor noise, accidental glove displacement, and external movement can cause false detections.

For real clinical use, the system would require controlled testing, validated detection accuracy, appropriate alarm handling, and medical safety evaluation.

## 12. Project Conclusion

The Hand Mobility Assessment Glove demonstrates the application of embedded systems and wearable motion sensors in patient monitoring.

By integrating Arduino with IMU sensors through I²C communication, the system can continuously acquire hand motion data, evaluate movement conditions, and trigger alerts when significant movement is detected.

The project combines sensor interfacing, embedded programming, real-time data acquisition, motion processing, and alert control.

Its primary purpose is to provide an automated method for detecting hand movements in coma patients and notifying caregivers for further assessment.

## What has been tested

- **The sketch compiles** for Arduino Uno and Nano (Arduino AVR core 1.8.8) with no warnings from the sketch itself: 15,008 bytes of program memory and 775 bytes of RAM. It has **not been run on real hardware** yet, so the thresholds are starting values to tune on your own glove. Saving settings to the Arduino's memory is untested for the same reason.
- **The detection logic** was run in simulation with typical MPU-6050 noise: no false alerts across 40 runs of 60 seconds with a still hand; finger twitch, tremor, wrist turn, hand lift and bed bump each detected once in 40 of 40 runs; the faint finger flex missed at the defaults and caught at a 1.0 °/s rotation threshold.
- **The movement labels** came out the same in 40 of 40 simulated runs each: finger twitch and bed bump short and small, tremor shaking and small, wrist turn and hand lift long and big.
- **The tilt angles** matched five held hand positions to within 1 degree in simulation.
- **The demo page** was checked in a browser at desktop and phone widths, including the log download and settings that survive a reload.
- **The USB link** between the page and a real Arduino has not been tested with a board.
