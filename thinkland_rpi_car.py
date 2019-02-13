import RPi.GPIO as GPIO
import time
import random


class Car:
    """
    A class that represents a Thinkland Raspberry Pi Car with the
    following I/O components:

    * 2 infrared sensors in the front on the lower deck (left and right)
    * 1 ultrasonic sensor in the front on the upper deck
    * 1 line-tracking sensor in the front below the lower deck
    * 4 DC-motors (2 on the left and 2 on the right)
    * 1 servo panning the ultrasonic sensor (180-degree movement)
    * 1 servo panning the camera (horizontal 180-degree movement)
    * 1 servo tilting the camera (vertical 180-degree movement)
    * 1 LED light mounted on top of ultrasonic sensor
    """

    """
    PIN definition
    """
    PIN_MOTOR_LEFT_FORWARD   = 20  # AIN_2
    PIN_MOTOR_LEFT_BACKWARD  = 21  # AIN_1
    PIN_MOTOR_RIGHT_FORWARD  = 19  # BIN_2
    PIN_MOTOR_RIGHT_BACKWARD = 26  # BIN_1
    PIN_MOTOR_LEFT_SPEED     = 16  # PWM_A
    PIN_MOTOR_RIGHT_SPEED    = 13  # PWM_B

    PIN_INFRARED_LEFT        = 12
    PIN_INFRARED_RIGHT       = 17

    PIN_SERVO_ULTRASONIC     = 23
    PIN_ULTRASONIC_TRIG      = 1
    PIN_ULTRASONIC_ECHO      = 0

    PIN_LED_R                = 22
    PIN_LED_G                = 27
    PIN_LED_B                = 24

    PIN_TRACK_1              = 3   # counting From left, 1
    PIN_TRACK_2              = 5   # 2
    PIN_TRACK_3              = 4   # 3
    PIN_TRACK_4              = 18  # 4

    def __init__(self):
        """
        Constructor

        Initialize the Car
        """
        self.init()

    def init(self):
        """
        Initialize the Car
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)  # ?

        GPIO.setup(Car.PIN_MOTOR_LEFT_FORWARD,   GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(Car.PIN_MOTOR_LEFT_BACKWARD,  GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(Car.PIN_MOTOR_RIGHT_FORWARD,  GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(Car.PIN_MOTOR_RIGHT_BACKWARD, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(Car.PIN_MOTOR_LEFT_SPEED,     GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(Car.PIN_MOTOR_RIGHT_SPEED,    GPIO.OUT, initial=GPIO.HIGH)

        GPIO.setup(Car.PIN_INFRARED_LEFT,        GPIO.IN)
        GPIO.setup(Car.PIN_INFRARED_RIGHT,       GPIO.IN)

        GPIO.setup(Car.PIN_SERVO_ULTRASONIC,     GPIO.OUT)
        GPIO.setup(Car.PIN_ULTRASONIC_TRIG,      GPIO.OUT)
        GPIO.setup(Car.PIN_ULTRASONIC_ECHO,      GPIO.IN)

        GPIO.setup(Car.PIN_LED_R,                GPIO.OUT)
        GPIO.setup(Car.PIN_LED_G,                GPIO.OUT)
        GPIO.setup(Car.PIN_LED_B,                GPIO.OUT)

        GPIO.setup(Car.PIN_TRACK_1,              GPIO.IN)
        GPIO.setup(Car.PIN_TRACK_2,              GPIO.IN)
        GPIO.setup(Car.PIN_TRACK_3,              GPIO.IN)
        GPIO.setup(Car.PIN_TRACK_4,              GPIO.IN)

        self._pwm_left_speed  = GPIO.PWM(Car.PIN_MOTOR_LEFT_SPEED,  2000)
        self._pwm_right_speed = GPIO.PWM(Car.PIN_MOTOR_RIGHT_SPEED, 2000)

        self._pwm_left_speed.start(0)
        self._pwm_right_speed.start(0)

        self._pwm_servo_ultrasonic = GPIO.PWM(Car.PIN_SERVO_ULTRASONIC, 50)
        self._pwm_servo_ultrasonic.start(0)

    def _set_motion(self, left_forward, left_backward,
                    right_forward, right_backward,
                    speed_left, speed_right,
                    duration=0.0):
        """
        Helper function to set car wheel motions

        Parameters
        ----------
        * left_forward   : GPIO.HIGH or LOW
        * left_backward  : GPIO.HIGH or LOW
        * right_forward  : GPIO.HIGH or LOW
        * right_backward : GPIO.HIGH or LOW
        * speed_left     : int
            An integer [0,100] for left motors speed
        * speed_right    : int
            An integer [0,100] for right motors speed
        * duration       : float
            Duration of the motion.
            (default=0.0 - continue indefinitely until called again)
        Raises
        ------
        """
        GPIO.output(Car.PIN_MOTOR_LEFT_FORWARD,   left_forward)
        GPIO.output(Car.PIN_MOTOR_LEFT_BACKWARD,  left_backward)
        GPIO.output(Car.PIN_MOTOR_RIGHT_FORWARD,  right_forward)
        GPIO.output(Car.PIN_MOTOR_RIGHT_BACKWARD, right_backward)
        self._pwm_left_speed.ChangeDutyCycle(speed_left)
        self._pwm_right_speed.ChangeDutyCycle(speed_right)
        if duration > 0.0:
            time.sleep(duration)
            self._pwm_left_speed.ChangeDutyCycle(0)
            self._pwm_right_speed.ChangeDutyCycle(0)

    def stop_all_wheels(self):
        """
        Stop wheel movement
        """
        self._set_motion(GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.LOW, 0, 0)

    def stop_completely(self):
        """
        Completely stop the Car
        """
        self._pwm_left_speed.stop()
        self._pwm_right_speed.stop()
        self._pwm_servo_ultrasonic.stop()
        GPIO.cleanup()

    def run_forward(self, speed=50, duration=0.0):
        """
        Run forward

        Parameters
        ----------
        * speed : int
            - Speed of the motors. Valid range [0, 100]
        * duration : float
            - Duration of the motion.
            (default=0.0 - continue indefinitely until other motions are set)

        Raises
        ------
        """
        self._set_motion(GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW,
                         speed, speed, duration)

    def run_reverse(self, speed=10, duration=0.0):
        """
        Run forward

        Parameters
        ----------
        * speed : int
            - Speed of the motors. Valid range [0, 100]
        * duration : float
            - Duration of the motion.
            (default=0.0 - continue indefinitely until other motions are set)

        Raises
        ------
        """
        self._set_motion(GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH,
                         speed, speed, duration)

    def turn_left(self, speed=10, duration=0.0):
        """
        Turn left - only right-hand-side wheels run forward

        Parameters
        ----------
        * speed : int
            - Speed of the motors. Valid range [0, 100]
        * duration : float
            - Duration of the motion.
            (default=0.0 - continue indefinitely until other motions are set)

        Raises
        ------
        """
        self._set_motion(GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.LOW,
                         0, speed, duration)

    def turn_right(self, speed=10, duration=0.0):
        """
        Turn right - only left-hand-side wheels run forward

        Parameters
        ----------
        * speed : int
            - Speed of the motors. Valid range [0, 100]
        * duration : float
            - Duration of the motion.
            (default=0.0 - continue indefinitely until other motions are set)

        Raises
        ------
        """
        self._set_motion(GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.LOW,
                         speed, 0, duration)

    def spin_left(self, speed=10, duration=0.0):
        """
        Spin to the left in place

        Parameters
        ----------
        * speed : int
            - Speed of the motors. Valid range [0, 100]
        * duration : float
            - Duration of the motion.
            (default=0.0 - continue indefinitely until other motions are set)

        Raises
        ------
        """
        self._set_motion(GPIO.LOW, GPIO.HIGH, GPIO.HIGH, GPIO.LOW,
                         speed, speed, duration)

    def spin_right(self, speed=10, duration=0.0):
        """
        Spin to the left in place

        Parameters
        ----------
        * speed : int
            - Speed of the motors. Valid range [0, 100]
        * duration : float
            - Duration of the motion.
            (default=0.0 - continue indefinitely until other motions are set)

        Raises
        ------
        """
        self._set_motion(GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH,
                         speed, speed, duration)

    def turn_servo_ultrasonic(self, dir='degree', degree=90):
        """
        Turn the servo for ultrasonic sensor

        Parameters
        ----------
        * dir : str
            - one of ['left', 'center', 'right']
            - if dir == 'degree', use degree parameter
        * degree : int
            - the angle to turn, measured in degree [0, 180]
            - if dir is specified other than 'degree', this is ignored
        """
        # 0 degrees :  duty cycle =  2.5% of 20ms
        # 90 degrees:  duty cycle =  7.5% of 20ms
        # 180 degrees: duty cycle = 12.5% of 20ms
        if dir == 'center':
            degree = 90
        elif dir == 'right':
            degree = 0
        elif dir == 'left':
            degree = 180

        for i in range(10):  # do this for multiple times just to make sure
            self._pwm_servo_ultrasonic.ChangeDutyCycle(2.5 + 10 * degree/180)
        time.sleep(0.2)  # give enough time to settle

    def distance_from_obstacle(self):
        """
        Measure the distance between ultrasonic sensor and the obstacle
        that it faces.

        The obstacle should have a relatively smooth surface for this
        to be effective. Distance to fabric or other sound-absorbing
        surfaces is difficult to measure.

        Returns
        -------
        * int
            - Measured in centimeters: valid range is 2cm to 400cm
        """
        # set HIGH at TRIG for 15us to trigger the ultrasonic ping
        GPIO.output(Car.PIN_ULTRASONIC_TRIG, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(Car.PIN_ULTRASONIC_TRIG, GPIO.LOW)

        # wait until echo is received
        while not GPIO.input(Car.PIN_ULTRASONIC_ECHO):
            pass

        # duration of the ECHO being high is how long the sound travels
        t1 = time.time()
        while GPIO.input(Car.PIN_ULTRASONIC_ECHO):
            pass
        t2 = time.time()
        distance = int((t2 - t1) * 343 / 2 * 100)
        print(t2-t1, distance)
        return distance

    def obstacle_status_from_infrared(self):
        """
        Return obstacle status obtained by infrared sensors that
        are situated at the left front and right front of the Car.
        The infrared sensors are located on the lower deck, so they
        have a lower view than the ultrasonic sensor.

        Indicates blockage by obstacle < 20cm away.
        Depending on sensitivity of sensors, the distance of obstacles
        sometimes needs to be as short as 15cm for effective detection

        Returns
        -------
        * str
            - one of ['only_left_blocked', 'only_right_blocked',
                    'blocked', 'clear']
        """
        is_left_clear  = GPIO.input(Car.PIN_INFRARED_LEFT)
        is_right_clear = GPIO.input(Car.PIN_INFRARED_RIGHT)

        if is_left_clear and is_right_clear:
            status = 'clear'
        elif is_left_clear and not is_right_clear:
            status = 'only_right_blocked'
        elif not is_left_clear and is_right_clear:
            status = 'only_left_blocked'
        else:
            status = 'blocked'
        print('Infrared status = {}'.format(status))
        return status

    def obstacle_status_from_ultrasound(self, dir='center'):
        """
        Return obstacle status obtained by ultrasonic sensor that is
        situated in the front of the Car. The ultrasonic sensor is
        located in the upper deck so it has a higher view than the
        infrared sensors.

        Parameters
        ----------
        * dir : str
            - set the ultrasonic sensor to face a direction,
            one of ['center', 'left', 'right']. Default is 'center'

        Returns
        -------
        * str
            - 'blocked' if distance <= 20cm
            - 'approaching_obstacle' if distance is (20, 50]
            - 'clear' if distance > 50cm
        """

        self.turn_servo_ultrasonic(dir)
        distance = self.distance_from_obstacle()
        if distance <= 20:
            status = 'blocked'
        elif distance <= 50:
            status = 'approaching_obstacle'
        else:
            status = 'clear'
        print('Ultrasound status = {}'.format(status))
        return status

    def _led_light(self, r, g, b):
        GPIO.output(Car.PIN_LED_R, r)
        GPIO.output(Car.PIN_LED_G, g)
        GPIO.output(Car.PIN_LED_B, b)

    def led_light(self, color):
        """
        Shine LED light

        Parameters
        ----------
        * color : str
            - one of ['red', 'green', 'blue',
                      'yellow', 'cyan', 'purple'
                      'white', 'off']
        """
        if color == 'red':
            self._led_light(GPIO.HIGH, GPIO.LOW, GPIO.LOW)
        elif color == 'green':
            self._led_light(GPIO.LOW, GPIO.HIGH, GPIO.LOW)
        elif color == 'blue':
            self._led_light(GPIO.LOW, GPIO.LOW, GPIO.HIGH)
        elif color == 'yellow':
            self._led_light(GPIO.HIGH, GPIO.HIGH, GPIO.LOW)
        elif color == 'cyan':
            self._led_light(GPIO.LOW, GPIO.HIGH, GPIO.HIGH)
        elif color == 'purple':
            self._led_light(GPIO.HIGH, GPIO.LOW, GPIO.HIGH)
        elif color == 'white':
            self._led_light(GPIO.HIGH, GPIO.HIGH, GPIO.HIGH)
        else:
            self._led_light(GPIO.LOW, GPIO.LOW, GPIO.LOW)

    def line_tracking_turn_type(self):
        """
        Indicates the type of turn required given current sensor values

        Returns
        -------
        * str
            - one of ['sharp_left_turn', 'sharp_right_turn',
                      'regular_left_turn', 'regular_right_turn',
                      'smooth_left', 'smooth_right',
                      'straight', 'no_line']
        """
        s1_dark = GPIO.input(Car.PIN_TRACK_1) == GPIO.LOW
        s2_dark = GPIO.input(Car.PIN_TRACK_2) == GPIO.LOW
        s3_dark = GPIO.input(Car.PIN_TRACK_3) == GPIO.LOW
        s4_dark = GPIO.input(Car.PIN_TRACK_4) == GPIO.LOW

        if s1_dark and (s3_dark and s4_dark):
            #   1    2    3    4
            # Dark XXXX Dark Dark
            # Dark XXXX Dark Lite
            # Dark XXXX Lite Dark
            # Requires a sharp left turn (line bends at right or acute angle)
            turn = 'sharp_left_turn'
        elif (s1_dark or s2_dark) and s4_dark:
            #   1    2    3    4
            # Dark Dark XXXX Dark
            # Lite Dark XXXX Dark
            # Dark Lite XXXX Dark
            # Requires a sharp right turn (line bends at right or acute angle)
            turn = 'sharp_right_turn'
        elif s1_dark:
            #   1    2    3    4
            # Dark XXXX XXXX XXXX
            # Requires a regular left turn (line bends at obtuse angle)
            turn = 'regular_left_turn'
        elif s4_dark:
            #   1    2    3    4
            # XXXX XXXX XXXX Dark
            # Requires a regular right turn (line bends at obtuse angle)
            turn = 'regular_right_turn'
        elif s2_dark and not s3_dark:
            #   1    2    3    4
            # XXXX Dark Lite XXXX
            # Requires a smooth curve to the left (car veers off to the right)
            turn = 'smooth_left'
        elif not s2_dark and s3_dark:
            #   1    2    3    4
            # XXXX Lite Dark XXXX
            # Requires a smooth curve to the right (car veers off to the left)
            turn = 'smooth_right'
        elif s2_dark and s3_dark:
            #   1    2    3    4
            # XXXX Dark Dark XXXX
            # Requires going straight
            turn = 'straight'
        else:
            #   1    2    3    4
            # Lite Lite Lite Lite
            # Requires maintaining the previous movement
            turn = 'no_line'

        print('Turn type = {}'.format(turn))
        return turn

    @staticmethod
    def demo_cruising():
        """
        Demonstrates a cruising car that avoids obstacles in a room

        * Use infrared sensors and ultrasonic sensor to gauge obstacles
        * Use LED lights to indicate running/turning decisions
        """
        time.sleep(2)
        car = Car()
        try:
            car.init()

            while True:
                obstacle_status_from_infrared = car.obstacle_status_from_infrared()
                should_turn = True
                if obstacle_status_from_infrared == 'clear':
                    should_turn = False
                    obstacle_status_from_ultrasound = \
                        car.obstacle_status_from_ultrasound()
                    if obstacle_status_from_ultrasound == 'clear':
                        car.led_light('green')
                        car.run_forward(speed=10)
                    elif obstacle_status_from_ultrasound == 'approaching_obstacle':
                        car.led_light('yellow')
                        car.run_forward(speed=5)
                    else:
                        should_turn = True
                if should_turn:
                    car.run_reverse(duration=0.02)
                    if obstacle_status_from_infrared == 'only_right_blocked':
                        car.led_light('purple')
                        car.spin_left(duration=random.uniform(0.25, 1.0))
                    elif obstacle_status_from_infrared == 'only_left_blocked':
                        car.led_light('cyan')
                        car.spin_right(duration=random.uniform(0.25, 1.0))
                    else:
                        car.led_light('red')
                        car.spin_right(duration=random.uniform(0.25, 1.0))
        except KeyboardInterrupt:
            car.stop_completely()

    @staticmethod
    def demo_line_tracking(speed=50):
        time.sleep(2)
        car = Car()
        try:
            car.init()
            while True:
                turn = car.line_tracking_turn_type()
                if turn == 'straight':
                    car.run_forward(speed=speed)
                elif turn == 'smooth_left':
                    car.turn_left(speed=speed * 0.75)
                elif turn == 'smooth_right':
                    car.turn_right(speed=speed * 0.75)
                elif turn == 'regular_left_turn':
                    car.spin_left(speed=speed * 0.75)
                elif turn == 'regular_right_turn':
                    car.spin_right(speed=speed * 0.75)
                elif turn == 'sharp_left_turn':
                    car.spin_left(speed=speed)
                elif turn == 'sharp_right_turn':
                    car.spin_right(speed=speed)
        except KeyboardInterrupt:
            car.stop_completely()


def main():
    Car.demo_cruising()


if __name__ == '__main__':
    main()
