# 3D Spatial Mapping LiDAR System

An embedded LiDAR scanner that maps indoor spaces in 3D. A VL53L1X time-of-flight sensor rides on a
stepper motor, sweeps 360° in 32 steps, and an MSP432E401Y microcontroller streams the distances to a
PC, where a Python script turns them into a 3D point cloud. Stacking scans taken a fixed distance apart
builds up a 3D map of a hallway.

Personal embedded systems project, Winter 2026.

## How it works

```
VL53L1X ToF sensor --I2C--> MSP432E401Y (Arm Cortex-M4F, 26 MHz) --UART 115200 bps--> PC (Python)
        ^                          |                                                    |
        |                     GPIO PH0-PH3                                    polar -> Cartesian
   stepper motor  <---------------+                                          Matplotlib 3D point cloud
```

- **Microcontroller:** TI MSP432E401Y (Arm Cortex-M4F with FPU), bus clock configured to 26 MHz via the PLL.
- **Sensor:** VL53L1X time-of-flight distance sensor on I2C (SDA = PB3, SCL = PB2), driven through ST's
  VL53L1X ultra-lite driver API.
- **Rotation:** stepper motor on GPIO Port H (PH0-PH3), full 360° sweep in 32 steps of 11.25°, pausing at
  each step for a stable reading, then returning to home so the cable doesn't wrap.
- **Controls and status:** push button PM0 starts a scan, PM1 sends the stored scans to the PC.
  LEDs show pre-scan (PF4), each measurement (PF0), and the return rotation (PN1).
- **Data link:** scans are buffered on the board and sent over UART at 115200 bps.
- **Visualization:** `python/plotmaker.py` reads the serial stream with PySerial, converts each
  (angle, distance) pair to (x, y, z), offsets each scan along the walking axis, and plots the point
  cloud with Matplotlib.

## Repository layout

| Path | Contents |
|---|---|
| `firmware/lidar_scanner.c` | Main firmware: I2C and GPIO setup, scan loop, stepper control, UART output |
| `firmware/uart.c`, `PLL.c`, `SysTick.c`, `onboardLEDs.c` | Peripheral drivers (PLL and SysTick adapted from Jonathan Valvano's examples) |
| `firmware/VL53L1X_api.*`, `vl53l1_platform*` | STMicroelectronics VL53L1X ultra-lite driver, plus an MSP432 I2C platform layer by T. Doyle and H. Mousavi (BSD-3-Clause, notices kept) |
| `firmware/RTE/` | MSP432E401Y startup and system files from the Keil device pack |
| `firmware/*.uvprojx`, `*.uvoptx` | Keil µVision project |
| `python/plotmaker.py` | Serial reader and 3D point-cloud plotter |
| `docs/Final_Report.pdf` | Full design report: characteristics, schematic, flowchart, expected output, limitations |

## Running it

1. **Wire it up:** connect the MSP432E401Y over USB, the VL53L1X on PB3/PB2 with 3.3 V and ground, and
   the stepper driver on PH0-PH3.
2. **Flash the firmware:** open `firmware/lidar_scanner.uvprojx` in Keil µVision, build, and download to the board.
3. **Set the COM port:** in Device Manager find "XDS110 Class Application/User UART" and put that port
   (for example `COM4`) in `COM_LINK` at the top of `python/plotmaker.py`. `MOVE_STEP_MM` sets the
   distance between scans (100 mm by default).
4. **Run the plotter:** `pip install pyserial matplotlib`, then `python python/plotmaker.py`. It waits for data.
5. **Scan:** press PM0 for a 360° sweep, move the device forward about 10 cm, and repeat. Press PM1 to
   send the scans; the script draws the 3D map once it receives `ALL SCANS COMPLETE`.

## Credits

The VL53L1X driver is © STMicroelectronics (BSD-3-Clause). PLL and SysTick routines are adapted from
Jonathan W. Valvano's embedded systems examples. The MSP432E401Y startup and system files come from the
Keil device pack. Everything else is my own work.
