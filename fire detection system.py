import BlynkLib
import serial
import RPi.GPIO as GPIO
import time

# Blynk auth token (replace with your Blynk token)
BLYNK_AUTH = 'TMPL2pKz2-NNo'

# Initialize Blynk
blynk = BlynkLib.Blynk(BLYNK_AUTH)

# Setup GPIO for buzzer, servo motor, and flame sensor
BUZZER_PIN = 18
SERVO_PIN = 17
FLAME_SENSOR_PIN = 27  # GPIO pin connected to the flame sensor

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(FLAME_SENSOR_PIN, GPIO.IN)  # Set flame sensor pin as input

# Initialize serial communication
ser = serial.Serial('/dev/serial0', 9600, timeout=1)
ser.flush()

# Function to control buzzer
def trigger_buzzer(state):
    if state == "ON":
        GPIO.output(BUZZER_PIN, GPIO.HIGH)
    else:
        GPIO.output(BUZZER_PIN, GPIO.LOW)

# Function to control servo motor
def move_servo(angle):
    pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz
    pwm.start(0)
    duty_cycle = (angle / 18) + 2
    GPIO.output(SERVO_PIN, True)
    pwm.ChangeDutyCycle(duty_cycle)
    GPIO.output(SERVO_PIN, False)
    pwm.ChangeDutyCycle(0)
    pwm.stop()

try:
    while True:
        # Run Blynk to keep it updated
        blynk.run()

        # Check flame sensor data
        flame_detected = not GPIO.input(FLAME_SENSOR_PIN)  # Active low

        if flame_detected:
            print("Flame Detected")
            trigger_buzzer("ON")
            move_servo(90)  # Move servo to 90 degrees
            blynk.virtual_write(1, 1)  # Send '1' to virtual pin V1 (flame detected)
        else:
            trigger_buzzer("OFF")
            move_servo(0)  # Move servo back to 0 degrees
            blynk.virtual_write(1, 0)  # Send '0' to virtual pin V1 (no flame)

        # Check for data from ESP8266 (smoke detector)
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            print("Received:", data)

            if data == "SMOKE_DETECTED":
                trigger_buzzer("ON")
                move_servo(90)  # Move servo to 90 degrees
                blynk.virtual_write(2, 1)  # Send '1' to virtual pin V2 (smoke detected)
            elif data == "NO_SMOKE":
                trigger_buzzer("OFF")
                move_servo(0)  # Move servo back to 0 degrees
                blynk.virtual_write(2, 0)  # Send '0' to virtual pin V2 (no smoke)

        # Add a small delay to prevent overwhelming the loop
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
    ser.close()
