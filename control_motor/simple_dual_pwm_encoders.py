#!/usr/bin/env python3
import time
from threading import Lock
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray
import RPi.GPIO as GPIO

# LUT quadratura
LUT = [0, +1, -1, 0,
       -1, 0,  0, +1,
       +1, 0,  0, -1,
        0, -1, +1, 0]

class Quad:
    def __init__(self, pin_a, pin_b, invert=False, glitch_ns=20000, pullup=True):
        self.a, self.b = pin_a, pin_b
        self.inv = -1 if invert else 1
        self.lock = Lock()
        self.pos = 0
        self.last_state = 0
        self.last_t = 0
        self.glitch = glitch_ns

        GPIO.setup(self.a, GPIO.IN, pull_up_down=GPIO.PUD_UP if pullup else GPIO.PUD_OFF)
        GPIO.setup(self.b, GPIO.IN, pull_up_down=GPIO.PUD_UP if pullup else GPIO.PUD_OFF)
        self.last_state = (GPIO.input(self.a) << 1) | GPIO.input(self.b)
        self.last_t = time.perf_counter_ns()
        GPIO.add_event_detect(self.a, GPIO.BOTH, callback=self._cb, bouncetime=0)
        GPIO.add_event_detect(self.b, GPIO.BOTH, callback=self._cb, bouncetime=0)

    def _cb(self, _):
        now = time.perf_counter_ns()
        if now - self.last_t < self.glitch:
            return
        state = (GPIO.input(self.a) << 1) | GPIO.input(self.b)
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

class MotorPWM:
    def __init__(self, pwm_pin, in1, in2, freq_hz=1000.0):
        for p in (pwm_pin, in1, in2):
            GPIO.setup(p, GPIO.OUT)
        self.in1, self.in2 = in1, in2
        self.pwm = GPIO.PWM(pwm_pin, freq_hz)
        self.pwm.start(0.0)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)

    def command(self, duty_signed):
        d = max(-100.0, min(100.0, float(duty_signed)))
        if abs(d) < 1e-3:
            GPIO.output(self.in1, GPIO.LOW); GPIO.output(self.in2, GPIO.LOW)
            self.pwm.ChangeDutyCycle(0.0); return
        forward = d >= 0.0
        GPIO.output(self.in1, GPIO.HIGH if forward else GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW  if forward else GPIO.HIGH)
        self.pwm.ChangeDutyCycle(abs(d))

    def stop(self):
        try:
            self.command(0.0); self.pwm.stop()
        except Exception:
            pass

class SimpleDualPWMEncoders(Node):
    def __init__(self):
        super().__init__('simple_dual_pwm_encoders')
        self.declare_parameters('', [
            ('pwm1',13), ('in1_1',5), ('in2_1',6),
            ('pwm2',19), ('in1_2',16), ('in2_2',26),
            ('pwm_freq_hz',1000.0),
            ('enca1',17), ('encb1',18),
            ('enca2',20), ('encb2',21),
            ('invert_left',False), ('invert_right',False),
            ('pullup',True), ('glitch_ns',20000),
            ('publish_rate_hz',50.0),
        ])

        GPIO.setmode(GPIO.BCM)
        self.m1 = MotorPWM(int(self.get_parameter('pwm1').value),
                           int(self.get_parameter('in1_1').value),
                           int(self.get_parameter('in2_1').value),
                           float(self.get_parameter('pwm_freq_hz').value))
        self.m2 = MotorPWM(int(self.get_parameter('pwm2').value),
                           int(self.get_parameter('in1_2').value),
                           int(self.get_parameter('in2_2').value),
                           float(self.get_parameter('pwm_freq_hz').value))

        self.qL = Quad(int(self.get_parameter('enca1').value),
                       int(self.get_parameter('encb1').value),
                       invert=bool(self.get_parameter('invert_left').value),
                       glitch_ns=int(self.get_parameter('glitch_ns').value),
                       pullup=bool(self.get_parameter('pullup').value))
        self.qR = Quad(int(self.get_parameter('enca2').value),
                       int(self.get_parameter('encb2').value),
                       invert=bool(self.get_parameter('invert_right').value),
                       glitch_ns=int(self.get_parameter('glitch_ns').value),
                       pullup=bool(self.get_parameter('pullup').value))

        self.pub_ticks = self.create_publisher(Int32MultiArray, 'encoder_ticks', 10)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0/max(1e-3, rate), self._tick)

        self.sub_pwm1 = self.create_subscription(Float32, 'pwm1', self._cb_pwm1, 10)
        self.sub_pwm2 = self.create_subscription(Float32, 'pwm2', self._cb_pwm2, 10)
        self.get_logger().info('ON: /pwm1, /pwm2 → motores | Pub: /encoder_ticks')

    def _cb_pwm1(self, msg: Float32): self.m1.command(msg.data)
    def _cb_pwm2(self, msg: Float32): self.m2.command(msg.data)

    def _tick(self):
        m = Int32MultiArray(); m.data = [self.qL.read(), self.qR.read()]
        self.pub_ticks.publish(m)

    def destroy_node(self):
        try:
            self.m1.stop(); self.m2.stop(); GPIO.cleanup()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    n = SimpleDualPWMEncoders()
    try: rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
