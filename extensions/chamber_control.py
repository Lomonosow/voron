# Chamber temperature control — Klipper extension
#
# Coordinates heater fans (nevermore, evenmore) and an exhaust fan to
# reach and hold a target chamber temperature.
#
# Control logic:
#   temp < target - deadband  →  heater fans full, exhaust off
#   temp > target + deadband  →  heater fans idle, exhaust proportional to overshoot
#   within deadband           →  heater fans idle, exhaust off
#   target = 0                →  disabled, fans left at current speed
#
# Install:
#   ln -sf ~/printer_data/config/extensions/chamber_control.py \
#          ~/klipper/klippy/extras/chamber_control.py
#
# printer.cfg:
#   [chamber_control]
#   heater_fans: nevermore, evenmore
#   exhaust_fan: chamber_exhaust
#   sensor: chamber
#   target_temp: 0        # 0 = disabled at startup
#   deadband: 2.0         # ±°C around target
#   idle_speed: 0.3       # heater fan speed in hold/cooling mode (filtration)
#   max_speed: 1.0
#   on_disable: default   # fan state when TARGET=0: default (leave as-is), idle, off
#
# Commands:
#   SET_CHAMBER_TEMP TARGET=45   - enable and set target
#   SET_CHAMBER_TEMP TARGET=0    - disable


class ChamberControl:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode   = self.printer.lookup_object('gcode')

        self.heater_fan_names  = [n.strip() for n in config.get('heater_fans').split(',')]
        self.exhaust_fan_name  = config.get('exhaust_fan')
        self.sensor_name       = config.get('sensor')
        self.target_temp       = config.getfloat('target_temp',      0.,  minval=0.)
        self.deadband          = config.getfloat('deadband',          2.,  above=0.)
        self.idle_speed        = config.getfloat('idle_speed',        0.3, minval=0., maxval=1.)
        self.max_speed = config.getfloat('max_speed', 1.0, minval=0., maxval=1.)
        self.update_interval   = config.getfloat('update_interval',   1.0, above=0.)
        on_disable = config.get('on_disable', 'default')
        if on_disable not in ('default', 'idle', 'off'):
            raise config.error("on_disable must be 'default', 'idle', or 'off'")
        self.on_disable = on_disable

        self.sensor = None

        self.gcode.register_command(
            'SET_CHAMBER_TEMP', self.cmd_SET_CHAMBER_TEMP,
            desc="Set chamber target temperature; TARGET=0 disables control")

        self.printer.register_event_handler('klippy:ready', self._handle_ready)

    def _handle_ready(self):
        self.sensor = self.printer.lookup_object(
            'temperature_sensor %s' % self.sensor_name)
        reactor = self.printer.get_reactor()
        reactor.register_timer(self._callback,
                               reactor.monotonic() + self.update_interval)

    def _set_fan(self, name, speed):
        self.gcode.run_script_from_command(
            'SET_FAN_SPEED FAN=%s SPEED=%.3f' % (name, speed))

    def _callback(self, eventtime):
        if self.target_temp <= 0. or self.sensor is None:
            return eventtime + self.update_interval

        try:
            current_temp = self.sensor.get_status(eventtime)['temperature']
        except Exception:
            return eventtime + self.update_interval

        error = self.target_temp - current_temp

        if error > self.deadband:
            undershoot   = error - self.deadband
            heater_speed = min(1., undershoot / self.deadband) * self.max_speed
            for name in self.heater_fan_names:
                self._set_fan(name, heater_speed)
            self._set_fan(self.exhaust_fan_name, self.idle_speed)
        elif error < -self.deadband:
            overshoot = -error - self.deadband
            exhaust   = min(1., overshoot / self.deadband) * self.max_speed
            for name in self.heater_fan_names:
                self._set_fan(name, self.idle_speed)
            self._set_fan(self.exhaust_fan_name, exhaust)
        else:
            for name in self.heater_fan_names:
                self._set_fan(name, self.idle_speed)
            self._set_fan(self.exhaust_fan_name, self.idle_speed)

        return eventtime + self.update_interval

    def cmd_SET_CHAMBER_TEMP(self, gcmd):
        target = gcmd.get_float('TARGET', 0., minval=0.)
        self.target_temp = target
        if target <= 0.:
            if self.on_disable == 'off':
                for name in self.heater_fan_names:
                    self._set_fan(name, 0.)
                self._set_fan(self.exhaust_fan_name, 0.)
            elif self.on_disable == 'idle':
                for name in self.heater_fan_names:
                    self._set_fan(name, self.idle_speed)
                self._set_fan(self.exhaust_fan_name, self.idle_speed)
            gcmd.respond_info("Chamber control disabled")
        else:
            gcmd.respond_info("Chamber target: %.1f C" % target)

    def get_status(self, eventtime):
        current = (self.sensor.get_status(eventtime)['temperature']
                   if self.sensor is not None else 0.)
        return {
            'target':  self.target_temp,
            'current': current,
            'active':  self.target_temp > 0.,
        }


def load_config(config):
    return ChamberControl(config)
