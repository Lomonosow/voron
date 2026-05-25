# Automatic TMC chopper register tuner - Klipper extension
#
# Sweeps TBL/TOFF/HSTRT/HEND (and TPFD on TMC2240/5160) while measuring
# vibration magnitude via ADXL345. Finds and applies the register set with
# the lowest median RMS magnitude at the motor's resonant speed.
#
# Credits:
#   MRX8024 — chopper-resonance-tuner (original measurement and scoring logic)
#     https://github.com/MRX8024/chopper-resonance-tuner
#   Klipper project — accelerometer and TMC driver APIs
#     https://github.com/Klipper3d/klipper
#
# Install:
#   ln -sf ~/printer_data/config/extensions/auto_chopper_tune.py \
#          ~/klipper/klippy/extras/auto_chopper_tune.py
#
# printer.cfg:
#   [auto_chopper_tune]
#   accel_chip: adxl345
#   min_speed: 25     # mm/s — start of sweep range
#   max_speed: 100    # mm/s — end of sweep range
#   speed_step: 1     # mm/s — step between sweep points
#
# Commands:
#   AUTO_CHOPPER_TUNE AXIS=X [SAVE=1]         - full pipeline
#   CHOPPER_FIND_VIBRATIONS AXIS=X             - phase 1 only
#   CHOPPER_SWEEP AXIS=X SPEED=55             - phase 2 only
#   CHOPPER_APPLY AXIS=X TBL=0 TOFF=8 HSTRT=5 HEND=5 [TPFD=3] [SAVE=1]

import numpy as np

TRIM_FRACTION    = 0.2   # fraction cut from each end of measurement (removes accel/decel transients)
FCLK_MHZ         = 12.0  # TMC internal clock frequency
HSTRT_HEND_MAX   = 16    # TMC hardware constraint: HSTRT + HEND <= 16
TMC_DRIVERS       = ['2240', '5160', '2209', '2208', '2130', '2660']
AXIS_IDX          = {'x': 0, 'y': 1, 'z': 2}


class AutoChopperTune:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode   = self.printer.lookup_object('gcode')

        self.accel_chip_name = config.get('accel_chip', 'adxl345')
        self.measure_time    = config.getfloat('measure_time', 1.25, above=0.)
        self.iterations      = config.getint('iterations', 1, minval=1)
        self.inset           = config.getfloat('inset', 10., above=0.)
        self.min_speed       = config.getfloat('min_speed', 25., above=0.)
        self.max_speed       = config.getfloat('max_speed', 100., above=0.)
        self.speed_step      = config.getfloat('speed_step', 1., above=0.)

        self.gcode.register_command(
            'AUTO_CHOPPER_TUNE', self.cmd_AUTO_CHOPPER_TUNE,
            desc="Full automatic TMC chopper tuning: find resonance, sweep registers, apply best")
        self.gcode.register_command(
            'CHOPPER_FIND_VIBRATIONS', self.cmd_FIND_VIBRATIONS,
            desc="Sweep speeds with current registers to identify the resonant peak speed")
        self.gcode.register_command(
            'CHOPPER_SWEEP', self.cmd_CHOPPER_SWEEP,
            desc="Sweep all TBL/TOFF/HSTRT/HEND combinations at SPEED mm/s")
        self.gcode.register_command(
            'CHOPPER_APPLY', self.cmd_CHOPPER_APPLY,
            desc="Apply TBL/TOFF/HSTRT/HEND[/TPFD] to driver live; SAVE=1 to persist")

        self.printer.register_event_handler('klippy:connect', self._handle_connect)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')

    # -------------------------------------------------------------------------
    # Hardware access
    # -------------------------------------------------------------------------

    def _get_accel_chip(self):
        chip = self.printer.lookup_object(self.accel_chip_name)
        if not hasattr(chip, 'start_internal_client'):
            raise self.gcode.error("'%s' is not an accelerometer" % self.accel_chip_name)
        return chip

    def _detect_tmc(self, stepper_name):
        """Return (driver_string, tmc_object). Tries drivers in priority order."""
        for drv in TMC_DRIVERS:
            obj = self.printer.lookup_object('tmc%s %s' % (drv, stepper_name), None)
            if obj is not None:
                return drv, obj
        raise self.gcode.error("No TMC driver found for %s" % stepper_name)

    def _set_field(self, stepper_name, field, value):
        self.gcode.run_script_from_command(
            'SET_TMC_FIELD STEPPER=%s FIELD=%s VALUE=%d' % (stepper_name, field, value))

    def _set_registers(self, steppers, tbl, toff, hstrt, hend, tpfd=None):
        for stepper in steppers:
            self._set_field(stepper, 'TBL',   tbl)
            self._set_field(stepper, 'TOFF',  toff)
            self._set_field(stepper, 'HSTRT', hstrt)
            self._set_field(stepper, 'HEND',  hend)
            if tpfd is not None:
                self._set_field(stepper, 'TPFD', tpfd)

    def _get_steppers_for_axis(self, axis):
        """CoreXY X/Y moves use both steppers; both must share the same registers."""
        kin = self.toolhead.get_kinematics()
        is_corexy = 'corexy' in type(kin).__name__.lower()
        if is_corexy and axis in ('x', 'y'):
            return ['stepper_x', 'stepper_y']
        return ['stepper_%s' % axis]

    def _get_axis_limits(self, axis):
        """Return (pos_min + inset, pos_max - inset) from static config."""
        cfg = self.printer.lookup_object('configfile').get_status(None)['config']
        s   = cfg['stepper_%s' % axis]
        return float(s['position_min']) + self.inset, float(s['position_max']) - self.inset

    def _get_other_axis_mid(self, axis):
        other = 'y' if axis == 'x' else 'x'
        lo, hi = self._get_axis_limits(other)
        return (lo + hi) / 2.

    # -------------------------------------------------------------------------
    # Motion helpers
    # -------------------------------------------------------------------------

    def _toolhead_info(self):
        systime = self.printer.get_reactor().monotonic()
        return self.toolhead.get_status(systime)

    def _calc_travel(self, speed, pos_min, pos_max):
        """Minimum travel to accelerate, cruise for measure_time, decelerate.
        Formula: v^2/a + v*t (symmetric accel/decel counted as one v^2/a term)."""
        max_accel = self._toolhead_info()['max_accel']
        required  = speed ** 2 / max_accel + speed * self.measure_time
        available = pos_max - pos_min
        if required > available:
            raise self.gcode.error(
                "Speed %.1f mm/s needs %.1f mm of travel, only %.1f mm available. "
                "Lower speed or raise max_accel." % (speed, required, available))
        return required

    def _build_speed_list(self, pos_min, pos_max):
        """All valid speeds (fit within axis travel) from configured speed range."""
        speeds, spd = [], self.min_speed
        while spd <= self.max_speed + 1e-6:
            try:
                self._calc_travel(spd, pos_min, pos_max)
                speeds.append(spd)
            except Exception:
                pass
            spd += self.speed_step
        if not speeds:
            raise self.gcode.error("No valid speeds fit within axis travel range")
        return speeds

    def _position_for_sweep(self, ax_idx, pos_min, axis):
        """Park at sweep start: primary axis at min, other axis at mid."""
        pos            = list(self.toolhead.get_position())
        pos[ax_idx]    = pos_min
        other_idx      = 1 if ax_idx == 0 else 0
        pos[other_idx] = self._get_other_axis_mid(axis)
        self.toolhead.move(pos, self._toolhead_info()['max_velocity'])
        self.toolhead.wait_moves()

    # -------------------------------------------------------------------------
    # Measurement
    # -------------------------------------------------------------------------

    def _measure_static(self, chip):
        """Capture 2-second stationary baseline; returns mean XYZ vector for subtraction."""
        aclient = chip.start_internal_client()
        self.toolhead.dwell(2.)
        aclient.finish_measurements()  # internally calls wait_moves()
        samples = aclient.get_samples()
        if not samples:
            raise self.gcode.error("No accelerometer data during static measurement")
        data = np.array([(s.accel_x, s.accel_y, s.accel_z) for s in samples])
        return data.mean(axis=0)

    def _measure_magnitude(self, chip, ax_idx, pos_start, pos_end,
                           ret_speed, move_speed, static):
        """Average median-RMS magnitude over self.iterations passes at move_speed."""
        magnitudes = []
        for _ in range(self.iterations):
            aclient     = chip.start_internal_client()
            pos         = list(self.toolhead.get_position())
            pos[ax_idx] = pos_end
            self.toolhead.move(pos, move_speed)
            aclient.finish_measurements()

            if not aclient.has_valid_samples():
                raise self.gcode.error("Accelerometer returned no valid samples")

            magnitudes.append(self._calc_magnitude(aclient.get_samples(), static))

            pos[ax_idx] = pos_start
            self.toolhead.move(pos, ret_speed)
            self.toolhead.wait_moves()

        return sum(magnitudes) / len(magnitudes)

    def _calc_magnitude(self, samples, static):
        """Median 3-axis RMS after baseline subtraction and transient trim."""
        data  = np.array([(s.accel_x, s.accel_y, s.accel_z) for s in samples])
        data -= static
        trim  = max(1, int(len(data) * TRIM_FRACTION))
        data  = data[trim:-trim]
        if len(data) == 0:
            return float('inf')
        return float(np.median(np.linalg.norm(data, axis=1)))

    # -------------------------------------------------------------------------
    # Chopper helpers
    # -------------------------------------------------------------------------

    def _chop_freq(self, toff, tbl):
        """Theoretical chopper switching frequency from TOFF and TBL (TMC datasheet)."""
        return 1. / (
            2. * (12. + 32. * toff) / (FCLK_MHZ * 1e6)
          + 2. * 16. * (1.5 ** tbl)  / (FCLK_MHZ * 1e6)
        )

    def _register_combos(self, gcmd):
        """Build all (TBL, TOFF, HSTRT, HEND) combinations within configured ranges."""
        tbl_min        = gcmd.get_int('TBL_MIN',        0)
        tbl_max        = gcmd.get_int('TBL_MAX',        3)
        toff_min       = gcmd.get_int('TOFF_MIN',       1)
        toff_max       = gcmd.get_int('TOFF_MAX',       8)
        hstrt_min      = gcmd.get_int('HSTRT_MIN',      0)
        hstrt_max      = gcmd.get_int('HSTRT_MAX',      7)
        hend_min       = gcmd.get_int('HEND_MIN',       2)
        hend_max       = gcmd.get_int('HEND_MAX',       15)
        hstrt_hend_max = gcmd.get_int('HSTRT_HEND_MAX', HSTRT_HEND_MAX)
        return [
            (tbl, toff, hstrt, hend)
            for tbl   in range(tbl_min,   tbl_max   + 1)
            for toff  in range(toff_min,  toff_max  + 1)
            for hstrt in range(hstrt_min, hstrt_max + 1)
            for hend  in range(hend_min,  hend_max  + 1)
            if hstrt + hend <= hstrt_hend_max
        ]

    # -------------------------------------------------------------------------
    # Utility
    # -------------------------------------------------------------------------

    def _disable_input_shaper(self):
        shaper = self.printer.lookup_object('input_shaper', None)
        if shaper is not None:
            shaper.disable_shaping()
        return shaper

    def _restore_input_shaper(self, shaper):
        if shaper is not None:
            shaper.enable_shaping()

    def _ensure_homed(self, gcmd):
        homed = self._toolhead_info()['homed_axes']
        if not all(ax in homed for ax in 'xyz'):
            gcmd.respond_info("Homing all axes...")
            self.gcode.run_script_from_command('G28')

    def _save_registers(self, axis, regs, tpfd=None):
        primary         = 'stepper_%s' % axis
        driver, _       = self._detect_tmc(primary)
        section         = 'tmc%s %s' % (driver, primary)
        configfile      = self.printer.lookup_object('configfile')
        configfile.set(section, 'driver_TBL',   str(regs['tbl']))
        configfile.set(section, 'driver_TOFF',  str(regs['toff']))
        configfile.set(section, 'driver_HSTRT', str(regs['hstrt']))
        configfile.set(section, 'driver_HEND',  str(regs['hend']))
        if tpfd is not None:
            configfile.set(section, 'driver_TPFD', str(tpfd))
        self.gcode.respond_info("Registers staged. Run SAVE_CONFIG to persist.")

    # -------------------------------------------------------------------------
    # Tuning phases
    # -------------------------------------------------------------------------

    def _phase_find_vibrations(self, axis, chip, gcmd):
        """Phase 1: sweep speeds with current driver registers; return resonant speed (peak magnitude)."""
        ax_idx  = AXIS_IDX[axis]
        pos_min, pos_max = self._get_axis_limits(axis)
        speeds    = self._build_speed_list(pos_min, pos_max)
        ret_speed = min(self._toolhead_info()['max_velocity'], max(speeds) * 3.)

        self._position_for_sweep(ax_idx, pos_min, axis)
        gcmd.respond_info("Capturing static baseline...")
        static = self._measure_static(chip)

        gcmd.respond_info("Sweeping %d speeds (%.1f - %.1f mm/s)..."
                          % (len(speeds), speeds[0], speeds[-1]))
        results = {}
        for spd in speeds:
            travel       = self._calc_travel(spd, pos_min, pos_max)
            mag          = self._measure_magnitude(chip, ax_idx,
                                                   pos_min, pos_min + travel,
                                                   ret_speed, spd, static)
            results[spd] = mag
            gcmd.respond_info("  %.1f mm/s -> %.1f" % (spd, mag))

        resonant_speed = max(results, key=results.get)
        gcmd.respond_info("Resonant speed: %.1f mm/s (magnitude %.1f)"
                          % (resonant_speed, results[resonant_speed]))
        return resonant_speed

    def _phase_sweep_registers(self, axis, chip, speed, gcmd):
        """Phase 2: brute-force all register combinations; return best as dict."""
        ax_idx   = AXIS_IDX[axis]
        steppers = self._get_steppers_for_axis(axis)
        pos_min, pos_max = self._get_axis_limits(axis)
        travel    = self._calc_travel(speed, pos_min, pos_max)
        ret_speed = min(self._toolhead_info()['max_velocity'], speed * 3.)
        combos    = self._register_combos(gcmd)

        gcmd.respond_info("Sweeping %d combinations at %.1f mm/s..." % (len(combos), speed))

        self._position_for_sweep(ax_idx, pos_min, axis)
        static = self._measure_static(chip)

        best_mag  = float('inf')
        best_regs = {}

        for i, (tbl, toff, hstrt, hend) in enumerate(combos):
            self._set_registers(steppers, tbl, toff, hstrt, hend)
            mag = self._measure_magnitude(chip, ax_idx,
                                          pos_min, pos_min + travel,
                                          ret_speed, speed, static)
            if mag < best_mag:
                best_mag  = mag
                best_regs = {'tbl': tbl, 'toff': toff, 'hstrt': hstrt, 'hend': hend}
                gcmd.respond_info(
                    "[%d/%d] Best: TBL=%d TOFF=%d HSTRT=%d HEND=%d "
                    "freq=%.1fkHz mag=%.1f"
                    % (i + 1, len(combos), tbl, toff, hstrt, hend,
                       self._chop_freq(toff, tbl) / 1000., mag))

        return best_regs

    def _phase_sweep_tpfd(self, axis, chip, speed, best_regs, gcmd):
        """Phase 3: sweep TPFD 0-15 with best TBL/TOFF/HSTRT/HEND locked in.
        Skipped automatically for drivers that don't support TPFD."""
        primary        = 'stepper_%s' % axis
        driver, _      = self._detect_tmc(primary)
        if driver not in ('2240', '5160'):
            gcmd.respond_info("TMC%s does not support TPFD, skipping phase 3" % driver)
            return None

        ax_idx   = AXIS_IDX[axis]
        steppers = self._get_steppers_for_axis(axis)
        pos_min, pos_max = self._get_axis_limits(axis)
        travel    = self._calc_travel(speed, pos_min, pos_max)
        ret_speed = min(self._toolhead_info()['max_velocity'], speed * 3.)
        tpfd_min  = gcmd.get_int('TPFD_MIN', 0)
        tpfd_max  = gcmd.get_int('TPFD_MAX', 15)

        gcmd.respond_info("Sweeping TPFD %d-%d..." % (tpfd_min, tpfd_max))

        self._set_registers(steppers, **best_regs)
        self._position_for_sweep(ax_idx, pos_min, axis)
        static = self._measure_static(chip)

        best_mag  = float('inf')
        best_tpfd = tpfd_min

        for tpfd in range(tpfd_min, tpfd_max + 1):
            for stepper in steppers:
                self._set_field(stepper, 'TPFD', tpfd)
            mag = self._measure_magnitude(chip, ax_idx,
                                          pos_min, pos_min + travel,
                                          ret_speed, speed, static)
            gcmd.respond_info("  TPFD=%d mag=%.1f" % (tpfd, mag))
            if mag < best_mag:
                best_mag  = mag
                best_tpfd = tpfd

        gcmd.respond_info("Best TPFD=%d (mag=%.1f)" % (best_tpfd, best_mag))
        return best_tpfd

    # -------------------------------------------------------------------------
    # Commands
    # -------------------------------------------------------------------------

    def cmd_AUTO_CHOPPER_TUNE(self, gcmd):
        axis = gcmd.get('AXIS', 'X').lower()
        save = gcmd.get_int('SAVE', 0)
        chip = self._get_accel_chip()

        if axis not in AXIS_IDX:
            raise self.gcode.error("AXIS must be X, Y, or Z")

        gcmd.respond_info("=== AUTO_CHOPPER_TUNE %s ===" % axis.upper())
        self._ensure_homed(gcmd)

        shaper = self._disable_input_shaper()
        try:
            gcmd.respond_info("[1/3] Finding resonant speed...")
            resonant_speed = self._phase_find_vibrations(axis, chip, gcmd)

            gcmd.respond_info("[2/3] Sweeping registers at %.1f mm/s..." % resonant_speed)
            best_regs = self._phase_sweep_registers(axis, chip, resonant_speed, gcmd)

            gcmd.respond_info("[3/3] Sweeping TPFD...")
            best_tpfd = self._phase_sweep_tpfd(axis, chip, resonant_speed, best_regs, gcmd)
        finally:
            self._restore_input_shaper(shaper)

        self._set_registers(self._get_steppers_for_axis(axis), tpfd=best_tpfd, **best_regs)

        result = ("Best registers for %s: TBL=%d TOFF=%d HSTRT=%d HEND=%d"
                  % (axis.upper(), best_regs['tbl'], best_regs['toff'],
                     best_regs['hstrt'], best_regs['hend']))
        if best_tpfd is not None:
            result += " TPFD=%d" % best_tpfd
        gcmd.respond_info(result)

        if save:
            self._save_registers(axis, best_regs, best_tpfd)

    def cmd_FIND_VIBRATIONS(self, gcmd):
        axis = gcmd.get('AXIS', 'X').lower()
        chip = self._get_accel_chip()
        self._ensure_homed(gcmd)
        shaper = self._disable_input_shaper()
        try:
            self._phase_find_vibrations(axis, chip, gcmd)
        finally:
            self._restore_input_shaper(shaper)

    def cmd_CHOPPER_SWEEP(self, gcmd):
        axis  = gcmd.get('AXIS', 'X').lower()
        speed = gcmd.get_float('SPEED', None)
        chip  = self._get_accel_chip()
        if speed is None:
            raise self.gcode.error("SPEED is required for CHOPPER_SWEEP")
        self._ensure_homed(gcmd)
        shaper = self._disable_input_shaper()
        try:
            best = self._phase_sweep_registers(axis, chip, speed, gcmd)
        finally:
            self._restore_input_shaper(shaper)
        gcmd.respond_info("Best: TBL=%d TOFF=%d HSTRT=%d HEND=%d"
                          % (best['tbl'], best['toff'], best['hstrt'], best['hend']))

    def cmd_CHOPPER_APPLY(self, gcmd):
        axis  = gcmd.get('AXIS', 'X').lower()
        tbl   = gcmd.get_int('TBL')
        toff  = gcmd.get_int('TOFF')
        hstrt = gcmd.get_int('HSTRT')
        hend  = gcmd.get_int('HEND')
        tpfd  = gcmd.get_int('TPFD', None)
        save  = gcmd.get_int('SAVE', 0)

        regs     = {'tbl': tbl, 'toff': toff, 'hstrt': hstrt, 'hend': hend}
        steppers = self._get_steppers_for_axis(axis)
        self._set_registers(steppers, tpfd=tpfd, **regs)
        gcmd.respond_info("Registers applied to: %s" % ', '.join(steppers))

        if save:
            self._save_registers(axis, regs, tpfd)


def load_config(config):
    return AutoChopperTune(config)
