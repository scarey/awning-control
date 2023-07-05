# MIT License (MIT)
# Copyright (c) 2023 Stephen Carey
# https://opensource.org/licenses/MIT

from machine import Pin, PWM
import uasyncio as asyncio


class Motor:

    def __init__(self, r_inh_pin, r_pwm_pin, r_is_pin,
                 l_inh_pin, l_pwm_pin, l_is_pin):
        self._r_is_pin = Pin(r_is_pin, Pin.IN)
        self._r_inh_pin = Pin(r_inh_pin, Pin.OUT)
        self._r_pwm_pin = PWM(Pin(r_pwm_pin, Pin.OUT))
        self._l_is_pin = Pin(l_is_pin, Pin.IN)
        self._l_inh_pin = Pin(l_inh_pin, Pin.OUT)
        self._l_pwm_pin = PWM(Pin(l_pwm_pin, Pin.OUT))
        self._spinning = False

    async def slow_start(self, direction):
        if self._spinning:
            self.rotate(100, direction)
        else:
            for speed in range(0, 400):
                self.rotate(speed / 4, direction)
                await asyncio.sleep(0.01)
            self._spinning = True

    def rotate(self, speed, direction):
        self._r_inh_pin.value(1)
        self._l_inh_pin.value(1)
        pwm_pin1 = self._r_pwm_pin
        pwm_pin2 = self._l_pwm_pin
        if direction != 1:
            pwm_pin1 = self._l_pwm_pin
            pwm_pin2 = self._r_pwm_pin
        pwm_pin1.duty(0)
        if 0 <= speed <= 100:
            pwm_pin2.duty(int(speed * 10.23))

    def stop(self):
        self._r_inh_pin.value(0)
        self._l_inh_pin.value(0)
        self._spinning = False
