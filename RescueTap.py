import smbus2
import math
import time
import RPi.GPIO as GPIO
from twilio.rest import Client
import gpsd

# Connect to the local gpsd instance
gpsd.connect()

# Get the current GPS data
packet = gpsd.get_current()

# Twilio account details
account_sid = ""
auth_token = ""
client = Client(account_sid, auth_token)
whatsapp_from = ""  # Twilio's Sandbox number
whatsapp_to = "whatsapp:"

# MPU-6050 register addresses
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Initialize the I2C bus and MPU-6050 sensor
bus = smbus2.SMBus(1)
address = 0x68

# Wake up the MPU-6050 sensor
bus.write_byte_data(address, PWR_MGMT_1, 0)

# Set the sample rate divider to 50Hz
bus.write_byte_data(address, SMPLRT_DIV, 9)

# Set the gyro and accelerometer ranges
bus.write_byte_data(address, GYRO_CONFIG, 0)
bus.write_byte_data(address, CONFIG, 0)

# Enable the accelerometer interrupts
bus.write_byte_data(address, INT_ENABLE, 0x01)

# Physical button setup (GPIO pin number may vary)
BUTTON_PIN = 16
BUZZER_PIN = 25  # GPIO18 for the buzzer
LED_PIN =17 
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUZZER_PIN, GPIO.OUT)  # Set up the buzzer pin as an output
GPIO.setup(LED_PIN, GPIO.OUT)  # Set up the led pin as an output

# Set the threshold for a fall detection
threshold = 0.5  # m/s^2
gyro_threshold = 2  # Degrees/second

# Variables to handle button press
#last_button_press_time = time.time()
last_button_press_time = 0
button_press_start_time = 0
button_press_count = 0
long_press_duration = 5  # 5 seconds for a long press

def sound_buzzer(duration):
    """Turn the buzzer on for a specified duration."""
    GPIO.output(BUZZER_PIN, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(BUZZER_PIN, GPIO.LOW)


# Main loop
while True:
    packet = gpsd.get_current()

    #Extract the latitude and longitude values
    latitude = packet.lat
    longitude = packet.lon

    # Convert the latitude and longitude to decimal degrees
    latitude = int(latitude) + (latitude - int(latitude)) * 100 / 60
    longitude = int(longitude) + (longitude - int(longitude)) * 100 / 60
    print("Latitude: {:.6f}, Longitude: {:.6f}".format(latitude, longitude))
    
    # Read the acceleration values in X, Y, and Z axes
    raw_accel_x = bus.read_byte_data(address, ACCEL_XOUT_H)
    raw_accel_y = bus.read_byte_data(address, ACCEL_YOUT_H)
    raw_accel_z = bus.read_byte_data(address, ACCEL_ZOUT_H)
    # Read the gyroscope values in X, Y, and Z axes
    raw_gyro_x = bus.read_byte_data(address, GYRO_XOUT_H) 
    raw_gyro_y = bus.read_byte_data(address, GYRO_YOUT_H) 
    raw_gyro_z = bus.read_byte_data(address, GYRO_ZOUT_H) 

    # Convert the raw values to m/s^2
    accel_x = (raw_accel_x / 16384.0) * 9.81
    accel_y = (raw_accel_y / 16384.0) * 9.81
    accel_z = (raw_accel_z / 16384.0) * 9.81
    # Convert the raw values to degrees/second
    gyro_x = (raw_gyro_x / 131.0)
    gyro_y = (raw_gyro_y / 131.0)
    gyro_z = (raw_gyro_z / 131.0)

    # Calculate the total acceleration
    acceleration = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
    # Calculate the total rotational velocity
    rotational_velocity = math.sqrt(gyro_x**2 + gyro_y**2 + gyro_z**2)

    print (acceleration)
    print (threshold)
    print (rotational_velocity)
    print (gyro_threshold)

    # Check if the acceleration exceeds the threshold or rotational velocity indicates a fall
    if acceleration > threshold or rotational_velocity > gyro_threshold:
        for _ in range(25):  # Loop 5 times
            GPIO.output(LED_PIN, GPIO.HIGH)  # Turn LED on
            time.sleep(1)  # Wait for 1 second
            GPIO.output(LED_PIN, GPIO.LOW)   # Turn LED off
            time.sleep(1)  # Wait for 1 second
        # Send a WhatsApp message with the fall location
        message = "Possible fall detected by sensor at: https://www.google.com/maps?q="
        message += str(latitude) + "," + str(longitude)
        client.messages.create(body=message, from_=whatsapp_from, to=whatsapp_to)
        sound_buzzer(1)  # Sound the buzzer for 1 second when a fall is detected
        # Wait for 10 seconds before taking another reading
        time.sleep(10)
        continue

    # Check if the physical button is pressed
    button_state = GPIO.input(BUTTON_PIN)
    current_time = time.time()

    if button_state == GPIO.LOW:
        if button_press_start_time is None:
            # Button was just pressed
            button_press_start_time = current_time
        elif current_time - button_press_start_time >= long_press_duration:
            # Button has been pressed long enough
            message = "Fall alert cancelled."
            client.messages.create(body=message, from_=whatsapp_from, to=whatsapp_to)
            GPIO.output(LED_PIN, GPIO.HIGH)  # Turn LED on
            time.sleep(10)  # Wait for 10 second
            GPIO.output(LED_PIN, GPIO.LOW)   # Turn LED off
            time.sleep(1)  # Wait for 1 second
            sound_buzzer(3)  # Sound the buzzer for 3 second when a fall alert is canceled
            button_press_start_time = None  # Reset button press start time after cancellation
            time.sleep(1)  # Debounce delay
    else:
        if button_press_start_time is not None:
            # Button was pressed but is now released
            press_duration = current_time - button_press_start_time
            if press_duration < long_press_duration:
                message = "Possible fall detected by push button at: https://www.google.com/maps?q="
                message += str(latitude) + "," + str(longitude)
                client.messages.create(body=message, from_=whatsapp_from, to=whatsapp_to)
                for _ in range(25):  # Loop 5 times
                    GPIO.output(LED_PIN, GPIO.HIGH)  # Turn LED on
                    time.sleep(1)  # Wait for 1 second
                    GPIO.output(LED_PIN, GPIO.LOW)   # Turn LED off
                    time.sleep(1)  # Wait for 1 second
                sound_buzzer(1)  # Sound the buzzer for 1 second when a fall is detected
                time.sleep(10)  # Wait before taking another reading
            button_press_start_time = None  # Reset button press start time

    # Wait for 1 second before taking another reading
    time.sleep(1)
