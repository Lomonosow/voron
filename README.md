# Voron 2.4 350mm — Klipper Configuration

## Installation

After cloning to `~/printer_data/config`, symlink the Klipper extension:

```bash
ln -sf ~/printer_data/config/extensions/auto_chopper_tune.py \
       ~/klipper/klippy/extras/auto_chopper_tune.py
ln -sf ~/printer_data/config/extensions/chamber_control.py \
       ~/klipper/klippy/extras/chamber_control.py
```

Then restart Klipper.

## Hardware

| Component | Part |
|---|---|
| Frame | Voron 2.4 350mm |
| MCU | BTT Manta M8P (CANbus) |
| Toolhead MCU | BTT EBBCan SB2209 (CANbus) |
| X/Y drivers | TMC2240 (SPI) |
| Z/E drivers | TMC2209 (UART) |
| X homing | Sensorless (TMC2240 SGT) |
| Y homing | Physical endstop |
| Z probe | Voron Tap (nozzle as probe) |
| Extruder thermistor | MAX31865 PT100 |
| Accelerometer | ADXL345 on EBBCan |
| Bed | 350mm heated, 9x9 mesh |
| Kinematics | CoreXY, quad gantry level |
| Filters | Nevermore + Evenmore (VOC) |
| Display | KlipperScreen |
| Stack | Klipper + Moonraker + Mainsail |

## Probe notes

Tap uses the nozzle tip as the probe. The `activate_gcode` in `[probe]` blocks probing above 150C to protect the PEI sheet. All Z homing and bed mesh operations run at the hotend hold temperature (140C). Nozzle thermal expansion between hold temp and print temp is compensated via the `Z_OFFSET` parameter passed from the slicer.

## Slicer start/end gcode

**Start:**
```
PRINT_START BED=[bed_temperature] EXTRUDER=[nozzle_temperature] CHAMBER=[chamber_temperature] Z_OFFSET=0
```

**End:**
```
PRINT_END
```

`Z_OFFSET` accepts a float (e.g. `0.05`) to fine-tune first layer height per material without changing the saved probe offset.

---

## Macros

### Print flow

**`PRINT_START BED= EXTRUDER= CHAMBER= Z_OFFSET=`**
Full start sequence. Heats bed, levels gantry, runs adaptive bed mesh, soaks for 3 minutes for frame thermal expansion, cleans nozzle, then starts the print.

**`PRINT_END`**
Retracts, lifts Z, turns off heaters and fans, parks at purge bucket.

**`PAUSE [Z=20]`**
Pauses the print: short retract, lifts Z, parks at purge bucket, long retract to prevent ooze while idle.

**`RESUME`**
Resumes from pause. Waits for extruder to reach minimum extrude temperature before restoring position.

**`CANCEL_PRINT [Z_LIFT=20]`**
Cancels the print: lifts Z safely within axis limits, turns off all heaters and fans, parks.

---

### Calibration

**`AUTO_CHOPPER_TUNE AXIS=X [SAVE=1]`**
Full automatic TMC chopper register tuning. Phase 1 sweeps speeds to find the resonant peak. Phase 2 sweeps all TBL/TOFF/HSTRT/HEND combinations at that speed. Phase 3 sweeps TPFD (TMC2240/5160 only). Applies the best result live; `SAVE=1` stages values for `SAVE_CONFIG`.

**`CHOPPER_FIND_VIBRATIONS AXIS=X`**
Phase 1 only. Runs with current driver registers and reports the resonant speed.

**`CHOPPER_SWEEP AXIS=X SPEED=55`**
Phase 2 only. Sweeps all register combinations at the given speed. Accepts `TBL_MIN/MAX`, `TOFF_MIN/MAX`, `HSTRT_MIN/MAX`, `HEND_MIN/MAX` to narrow the search range.

**`CHOPPER_APPLY AXIS=X TBL=0 TOFF=8 HSTRT=5 HEND=5 [TPFD=3] [SAVE=1]`**
Apply a specific register set to the driver live without running a sweep.

**`CALIBRATE_ALL [HOTEND=260] [BED=110] [PID=0]`**
First-boot calibration sequence. Optionally runs PID tuning for hotend and bed (`PID=1`), then runs input shaper calibration (ADXL345) and a full 9x9 bed mesh. Saves all results and restarts Klipper.

**`QUAD_GANTRY_LEVEL`**
Levels the gantry using 4-point probing. Skips automatically if already applied in the current session.

**`Z_OFFSET_CALIBRATION [START_BED_TEMP=30] [END_BED_TEMP=110] [BED_TEMP_STEP=10] [PROBE_SAMPLES=10]`**
Measures Z offset drift across a range of bed temperatures. Useful for characterizing frame expansion behavior.

**`HOME`**
Homes all axes. Wrapper around `G28` for console/button use.

---

### Nozzle

**`CLEAN_NOZZLE [EXTRUDE=false]`**
Wipes the nozzle on the purge bucket. Pass `EXTRUDE=true` to purge 20mm of filament before wiping. Safe to run at hold temperature (140C) only — running at print temperature risks PEI damage at Y350.

**`SET_CHAMBER_TEMP TARGET=45`**
Enable chamber temperature control. Runs Nevermore and Evenmore at full speed while below target, holds them at idle speed (filtration) when at target, and opens the exhaust fan proportionally if the chamber overshoots. `TARGET=0` disables control and leaves fans at their current speed.

---

### Parking

**`PARK_WASTE`**
Moves toolhead to the purge bucket (X210 Y350). Used by pause, cancel, and post-mesh positioning. Coordinates mirror `CLEAN_NOZZLE` variables — change the bucket position there.

**`PARK_START`**
Moves toolhead to bed center (X175 Y175 Z20). Used during heat-up.

---

### Fans

**`FAN_ON NAME=`**
Turns a `fan_generic` on at full speed. Use `NAME=nevermore` or `NAME=evenmore`.

**`FAN_OFF NAME=`**
Turns a `fan_generic` off.

**`FAN_TOGGLE NAME=`**
Toggles a `fan_generic` between on and off.

---

### LEDs

Toolhead LED color states:

| Macro | Color | State |
|---|---|---|
| `LED_HOMING` | Blue | Homing / leveling |
| `LED_HEATING` | Red | Waiting for temperature |
| `LED_PRINTING` | White | Printing |
| `LED_COOLING` | Green | Paused / cooling |
| `LED_OFF` | Off | Idle |

**`CHAMBER_LIGHT_ON [VALUE=1.0]`** / **`CHAMBER_LIGHT_OFF`** / **`CHAMBER_LIGHT_TOGGLE`**
Controls the PWM case light. `VALUE` accepts 0.0–1.0 for brightness.
