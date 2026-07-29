import config
import zmq
import json
import time


class Motor:
    def __init__(self):
        self.STEERING_CENTER_PWM = config.STEERING_CENTER_PWM
        self.STEERING_WIDTH_PWM = config.STEERING_WIDTH_PWM
        self.STEERING_RIGHT_PWM = config.STEERING_RIGHT_PWM
        self.STEERING_LEFT_PWM = config.STEERING_LEFT_PWM
        self.THROTTLE_STOPPED_PWM = config.THROTTLE_STOPPED_PWM
        self.THROTTLE_WIDTH_PWM = config.THROTTLE_WIDTH_PWM
        self.THROTTLE_FORWARD_PWM = config.THROTTLE_FORWARD_PWM
        self.THROTTLE_REVERSE_PWM = config.THROTTLE_REVERSE_PWM

        # ZeroMQ publisher
        self.context = zmq.Context()
        self.publisher = self.context.socket(zmq.PUB)
        self.publisher.setsockopt(zmq.SNDTIMEO, 1000)
        self.publisher.connect("tcp://localhost:5556")

        self.rcinput_data = {
            "sim_time": time.time(),
            "throttle": 0.0,
            "steering": 0.0,
            "braking": False
        }
        self.topic_name = "rcinput"

    def __del__(self):
        self.publisher.close()
        self.context.term()

    def set_steering_pwm_value(self, steering_value):
        if steering_value < -1.0:
            steering_value = -1.0
        elif steering_value > 1.0:
            steering_value = 1.0

        self.rcinput_data["sim_time"] = time.time()
        self.rcinput_data["steering"] = - steering_value
        self.rcinput_data["braking"] = False
        json_data = json.dumps(self.rcinput_data)
        try:
            self.publisher.send_multipart([
                self.topic_name.encode('utf-8'),
                json_data.encode('utf-8')
            ])
        except zmq.Again:
            print("No message sent within the timeout period.")


    def set_throttle_pwm_value(self, throttle_value):
        if throttle_value < -1.0:
            throttle_value = -1.0
        elif throttle_value > 1.0:
            throttle_value = 1.0

        self.rcinput_data["sim_time"] = time.time()
        self.rcinput_data["throttle"] = throttle_value
        self.rcinput_data["braking"] = False
        json_data = json.dumps(self.rcinput_data)
        try:
            self.publisher.send_multipart([
                self.topic_name.encode('utf-8'),
                json_data.encode('utf-8')
            ])
        except zmq.Again:
            print("No message sent within the timeout period.")


    def limit_steering_pwm(self, steering_pwm_value):
        if steering_pwm_value > config.STEERING_HI_LIMIT:
            return config.STEERING_HI_LIMIT
        elif steering_pwm_value < config.STEERING_LO_LIMIT:
            return config.STEERING_LO_LIMIT
        else:
            return steering_pwm_value
        

    def adjust_steering(self):
        return self.STEERING_RIGHT_PWM,self.STEERING_CENTER_PWM,self.STEERING_LEFT_PWM

    def adjust_throttle(self):
        return self.THROTTLE_FORWARD_PWM,self.THROTTLE_STOPPED_PWM,self.THROTTLE_REVERSE_PWM

    def breaking(self):
        self.rcinput_data["sim_time"] = time.time()
        self.rcinput_data["throttle"] = 0.0
        self.rcinput_data["braking"] = True
        json_data = json.dumps(self.rcinput_data)
        try:
            self.publisher.send_multipart([
                self.topic_name.encode('utf-8'),
                json_data.encode('utf-8')
            ])
        except zmq.Again:
            print("No message sent within the timeout period.")

    def cleanup(self):
        self.rcinput_data["steering"] = 0.0
        self.rcinput_data["throttle"] = 0.0
        self.rcinput_data["braking"] = False
        json_data = json.dumps(self.rcinput_data)
        try:
            self.publisher.send_multipart([
                self.topic_name.encode('utf-8'),
                json_data.encode('utf-8')
            ])
        except zmq.Again:
            print("No message sent within the timeout period.")

if __name__ == "__main__":
    import msvcrt

    motor = Motor()

    print("When you want to quit, press 'ESC'")
    while True:
        if msvcrt.kbhit():
            c = msvcrt.getch()
            if c == b'\x1b':
                break
            if c== b'w':
                motor.set_throttle_pwm_value(0.3)
                motor.set_steering_pwm_value(0.0)
            if c== b's':
                motor.breaking()
            if c== b'z':
                motor.set_throttle_pwm_value(-0.3)
                motor.set_steering_pwm_value(0.0)
            if c== b'q':
                motor.set_steering_pwm_value(-1.0)
            if c== b'e':
                motor.set_steering_pwm_value(1.0)

        time.sleep(0.05)

