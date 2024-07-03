#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

class Ultrasonic:
    def __init__(self, gpio_trigger, gpio_echo, range_min=10, range_max=400):
        # Initialise GPIO mode
        GPIO.setmode(GPIO.BCM)
        
        self._gpio_trigger = gpio_trigger
        self._gpio_echo = gpio_echo
        self._range_min = range_min
        self._range_max = range_max
        self._is_reading = False
        
        # Speed of sound in cm/s divided by 2 (round trip)
        self._speed_sound = 17150.0
        
        # Timeout calculation based on max range
        self._timeout = range_max / self._speed_sound * 2

        # Setup GPIO pins
        GPIO.setup(gpio_trigger, GPIO.OUT)
        GPIO.setup(gpio_echo, GPIO.IN)

        # Ensure the trigger is low initially
        GPIO.output(gpio_trigger, GPIO.LOW)
        time.sleep(1)  # Wait for sensor to settle

    def get_range(self):
        self._is_reading = True
        
        # Send a short trigger pulse
        GPIO.output(self._gpio_trigger, GPIO.HIGH)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(self._gpio_trigger, GPIO.LOW)

        # Initialize start and end times
        pulse_start_time = time.time()
        pulse_end_time = time.time()

        # Wait for the echo to start
        while GPIO.input(self._gpio_echo) == 0:
            pulse_start_time = time.time()
            if time.time() - pulse_start_time > self._timeout:
                self._is_reading = False
                return -1  # Timeout

        # Wait for the echo to end
        while GPIO.input(self._gpio_echo) == 1:
            pulse_end_time = time.time()
            if pulse_end_time - pulse_start_time > self._timeout:
                self._is_reading = False
                return -1  # Timeout

        self._last_time_reading = time.time()
        self._is_reading = False

        # Calculate pulse duration and distance
        pulse_duration = pulse_end_time - pulse_start_time
        distance = pulse_duration * self._speed_sound

        # Clamp distance within specified range
        if distance > self._range_max:
            distance = self._range_max
        if distance < self._range_min:
            distance = self._range_min

        return distance

    @property
    def is_reading(self):
        return self._is_reading

if __name__ == "__main__":
    # Define GPIO pins for the ultrasonic sensors
    sensors = [
        {"trigger": 4, "echo": 17, "label": "left"},
        {"trigger": 27, "echo": 22, "label": "center"},
        {"trigger": 6, "echo": 5, "label": "right"}
    ]

    # Create a list to store the Ultrasonic objects
    ultrasonics = []

    # Create an Ultrasonic object for each sensor and store it in the list
    for sensor in sensors:
        ultrasonics.append(Ultrasonic(sensor["trigger"], sensor["echo"]))
    
    # Continuously read and print distance
    try:
        while True:
            for i in range(len(ultrasonics)):
                distance = ultrasonics[i].get_range()
                if distance > 0:
                    print(f"{sensors[i]['label']} sensor distance = {distance:.2f} cm")
            time.sleep(1)  # Wait before next reading
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
