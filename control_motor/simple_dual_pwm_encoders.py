#!/usr/bin/env python3
import time
from threading import Lock
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray

# ---------------- LUT quadratura ----------------
LUT = [0, +1, -1, 0,
       -1, 0,  0, +1,
       +1, 0,  0, -1,
        0, -1, +1, 0]

# ---------------- Backends (pigpio / RPi.GPIO) ----------------
_have_pigpio = False
_have_rpi = False
try:
    import pigpio  # preferido
    _have_pigpio = True
except Exception:
    pass

if not _have_pigpio:
    try:
        import RPi.GPIO as RPI  # fallback
        _have_rpi = True
    except Exception:
        pass

# --------- Implementação com pigpio (recomendada) ----------
class QuadPigpio:
    def __init__(self, pi, pin_a, pin_b, invert=False, glitch_us=20, pullup=True):
        self.pi = pi
        self.a, self.b = int(pin_a), int(pin_b)
        self.inv = -1 if invert else 1
        self.pos = 0
        self.lock = Lock()
        self.state = 0

        self.pi.set_mode(self.a, pigpio.INPUT)
        self.pi.set_mode(self.b, pigpio.INPUT)
        if pullup:
            self.pi.set_pull_up_down(self.a, pigpio.PUD_UP)
            self.pi.set_pull_up_down(self.b, pigpio.PUD_UP)

        # filtro anti-ruído (µs)
        self.pi.set_glitch_filter(self.a, int(max(0, glitch_us)))
        self.pi.set_glitch_filter(self.b, int(max(0, glitch_us)))

        self.state = (self.pi.read(self.a) << 1) | self.pi.read(self.b)
        self.cb_a = self.pi.callback(self.a, pigpio.EITHER_EDGE, self._cb)
        self.cb_b = self.pi.callback(self.b, pigpio.EITHER_EDGE, self._cb)

    def _cb(self, gpio, level, tick):
        s = (self.pi.read(self.a) << 1) | self.pi.read(self.b)
        idx = ((self.state << 2) | s) & 0xF
        delta = LUT[idx]
        if delta != 0:
            with self.lock:
                self.pos += self.inv * delta
        self.state = s

    def read(self):
        with self.lock:
            return self.pos

    def cancel(self):
        self.cb_a.cancel()
        self.cb_b.cancel()

class MotorPigpio:
    def __init__(self, pi, pwm_pin, in1, in2, freq_hz=1000.0):
        self.pi = pi
        self.pwm = int(pwm_pin)
        self.in1 = int(in1)
        self.in2 = int(in2)
        self.pi.set_mode(self.pwm, pigpio.OUTPUT)
        self.pi.set_mode(self.in1, pigpio.OUTPUT)
        self.pi.set_mode(self.in2, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(self.pwm, int(freq_hz))
        self.pi.set_PWM_dutycycle(self.pwm, 0)
        self.pi.write(self.in1, 0)
        self.pi.write(self.in2, 0)

    def command(self, duty_percent_signed: float):
        d = max(-100.0, min(100.0, float(duty_percent_signed)))
        if abs(d) < 1e-3:
            self.pi.write(self.in1, 0)
            self.pi.write(self.in2, 0)
            self.pi.set_PWM_dutycycle(self.pwm, 0)
            return
        forward = d >= 0.0
        self.pi.write(self.in1, 1 if forward else 0)
        self.pi.write(self.in2, 0 if forward else 1)
        dc = int(round(abs(d) * 255.0 / 100.0))  # 0..255
        self.pi.set_PWM_dutycycle(self.pwm, dc)

    def stop(self):
        try:
            self.pi.set_PWM_dutycycle(self.pwm, 0)
            self.pi.write(self.in1, 0)
            self.pi.write(self.in2, 0)
        except Exception:
            pass

# --------- Fallback com RPi.GPIO ----------
class QuadRPI:
    def __init__(self, pin_a, pin_b, invert=False, glitch_us=20, pullup=True):
        self.a, self.b = int(pin_a), int(pin_b)
        self.inv = -1 if invert else 1
        self.pos = 0
        self.lock = Lock()
        self.last_state = 0
        self.last_t = 0
        self.glitch_ns = int(glitch_us) * 1000  # manter compat.

        RPI.setmode(RPI.BCM)
        RPI.setup(self.a, RPI.IN, pull_up_down=RPI.PUD_UP if pullup else RPI.PUD_OFF)
        RPI.setup(self.b, RPI.IN, pull_up_down=RPI.PUD_UP if pullup else RPI.PUD_OFF)
        self.last_state = (RPI.input(self.a) << 1) | RPI.input(self.b)
        self.last_t = time.perf_counter_ns()
        RPI.add_event_detect(self.a, RPI.BOTH, callback=self._cb, bouncetime=0)
        RPI.add_event_detect(self.b, RPI.BOTH, callback=self._cb, bouncetime=0)

    def _cb(self, _):
        now = time.perf_counter_ns()
        if now - self.last_t < self.glitch_ns:
            return
        state = (RPI.input(self.a) << 1) | RPI.input(self.b)
        idx = ((self.last_state << 2) | state) & 0xF
        delta = LUT[idx]
        if delta != 0:
            with self.lock:
                self.pos += self.inv * delta
        self.last_state = state
        self.last_t = now

    def read(self):
        with self.lock:
            return self.pos

    def cancel(self):
        try:
            RPI.remove_event_detect(self.a)
            RPI.remove_event_detect(self.b)
        except Exception:
            pass

class MotorRPI:
    def __init__(self, pwm_pin, in1, in2, freq_hz=1000.0):
        self.pwm_pin = int(pwm_pin)
        self.in1 = int(in1)
        self.in2 = int(in2)
        RPI.setmode(RPI.BCM)
        for p in (self.pwm_pin, self.in1, self.in2):
            RPI.setup(p, RPI.OUT)
        self._pwm = RPI.PWM(self.pwm_pin, freq_hz)
        self._pwm.start(0.0)
        RPI.output(self.in1, RPI.LOW)
        RPI.output(self.in2, RPI.LOW)

    def command(self, duty_percent_signed: float):
        d = max(-100.0, min(100.0, float(duty_percent_signed)))
        if abs(d) < 1e-3:
            RPI.output(self.in1, RPI.LOW); RPI.output(self.in2, RPI.LOW)
            self._pwm.ChangeDutyCycle(0.0)
            return
        forward = d >= 0.0
        RPI.output(self.in1, RPI.HIGH if forward else RPI.LOW)
        RPI.output(self.in2, RPI.LOW  if forward else RPI.HIGH)
        self._pwm.ChangeDutyCycle(abs(d))

    def stop(self):
        try:
            self.command(0.0)
            self._pwm.stop()
        except Exception:
            pass

# ---------------- Nó ROS 2 ----------------
class SimpleDualPWMEncoders(Node):
    def __init__(self):
        super().__init__('simple_dual_pwm_encoders')

        self.declare_parameters('', [
            # Backend
            ('gpio_backend', 'auto'),    # 'auto' | 'pigpio' | 'rpi'
            # Motores (SEUS PINOS)
            ('pwm1', 13), ('in1_1', 5), ('in2_1', 6),
            ('pwm2', 19), ('in1_2',16), ('in2_2',26),
            ('pwm_freq_hz', 1000.0),
            # Encoders
            ('enca1',17), ('encb1',18),
            ('enca2',20), ('encb2',21),
            ('invert_left', False), ('invert_right', False),
            ('pullup', True),
            ('glitch_us', 20),           # filtro em µs (pigpio); no RPi.GPIO vira ~ns*1000
            ('publish_rate_hz', 50.0),
        ])

        backend = str(self.get_parameter('gpio_backend').value).lower()
        use_pigpio = False
        if backend == 'pigpio':
            use_pigpio = True
        elif backend == 'rpi':
            use_pigpio = False
        else:  # auto
            use_pigpio = _have_pigpio

        self.get_logger().info(f'GPIO backend selecionado: {"pigpio" if use_pigpio else "RPi.GPIO"}')

        # Motores
        f = float(self.get_parameter('pwm_freq_hz').value)
        if use_pigpio:
            if not _have_pigpio:
                raise RuntimeError("pigpio não disponível. Instale python3-pigpio e inicie pigpiod.")
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("Não conectou ao daemon pigpio. Rode: sudo systemctl start pigpiod")
            self.m1 = MotorPigpio(self.pi, self.get_parameter('pwm1').value,
                                  self.get_parameter('in1_1').value, self.get_parameter('in2_1').value, f)
            self.m2 = MotorPigpio(self.pi, self.get_parameter('pwm2').value,
                                  self.get_parameter('in1_2').value, self.get_parameter('in2_2').value, f)
        else:
            if not _have_rpi:
                raise RuntimeError("RPi.GPIO não disponível. Instale python3-rpi.gpio ou use pigpio.")
            self.m1 = MotorRPI(self.get_parameter('pwm1').value,
                               self.get_parameter('in1_1').value, self.get_parameter('in2_1').value, f)
            self.m2 = MotorRPI(self.get_parameter('pwm2').value,
                               self.get_parameter('in1_2').value, self.get_parameter('in2_2').value, f)

        # Encoders
        invert_left  = bool(self.get_parameter('invert_left').value)
        invert_right = bool(self.get_parameter('invert_right').value)
        pullup = bool(self.get_parameter('pullup').value)
        glitch_us = int(self.get_parameter('glitch_us').value)

        if use_pigpio:
            self.qL = QuadPigpio(self.pi, self.get_parameter('enca1').value, self.get_parameter('encb1').value,
                                 invert=invert_left, glitch_us=glitch_us, pullup=pullup)
            self.qR = QuadPigpio(self.pi, self.get_parameter('enca2').value, self.get_parameter('encb2').value,
                                 invert=invert_right, glitch_us=glitch_us, pullup=pullup)
        else:
            self.qL = QuadRPI(self.get_parameter('enca1').value, self.get_parameter('encb1').value,
                              invert=invert_left, glitch_us=glitch_us, pullup=pullup)
            self.qR = QuadRPI(self.get_parameter('enca2').value, self.get_parameter('encb2').value,
                              invert=invert_right, glitch_us=glitch_us, pullup=pullup)

        # Pub/sub
        self.pub_ticks = self.create_publisher(Int32MultiArray, 'encoder_ticks', 10)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0/max(1e-3, rate), self._tick)

        self.sub_pwm1 = self.create_subscription(Float32, 'pwm1', self._cb_pwm1, 10)
        self.sub_pwm2 = self.create_subscription(Float32, 'pwm2', self._cb_pwm2, 10)

        self.use_pigpio = use_pigpio
        self.get_logger().info('Pronto. Pub: /encoder_ticks | Sub: /pwm1, /pwm2')

    def _cb_pwm1(self, msg: Float32): self.m1.command(msg.data)
    def _cb_pwm2(self, msg: Float32): self.m2.command(msg.data)

    def _tick(self):
        m = Int32MultiArray()
        m.data = [self.qL.read(), self.qR.read()]
        self.pub_ticks.publish(m)

    def destroy_node(self):
        try:
            self.m1.stop(); self.m2.stop()
            if self.use_pigpio:
                self.qL.cancel(); self.qR.cancel()
                self.pi.stop()
            else:
                try:
                    RPI.cleanup()
                except Exception:
                    pass
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = SimpleDualPWMEncoders()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
