import RPi.GPIO as gpio
from datetime import datetime, timedelta
import os
import subprocess
from time import sleep

#Air temperature nad humidity
import dht11

# Camera
import cv2
import numpy as np
from picamzero import Camera

# ADC-soil sensor
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

base_ratio, base_yellow = 0, 0
firstImage = True

# Day counters
temp_day = 0
temp_day_updated = False
hum_day = 0
hum_day_updated = False
soil_day = 0
soil_day_updated = False

rainYesterday = False   # memory whether or not it rained / soil moisture was too high yesterday
needPhoto = False       # boolean for taking a photo at maximum once a day

# Set GPIO mode to BCM (this will apply to both `gpio` and `gpio`)
gpio.setmode(gpio.BCM)

# Temp and humidity sensor setup (DHT11 on GPIO pin 19 in BCM mode)
gpio.setup(26, gpio.OUT)  # power to air temp and hum
gpio.setup(27, gpio.OUT)  # power to buzzer
gpio.setup(22, gpio.OUT)  # power to ADC and soil sensor

instance = dht11.DHT11(pin=19)  # 19 corresponds to pin 35 in BOARD mode

# Grove buzzer setup on GPIO pin 13 in BCM mode
gpio.setup(13, gpio.OUT)  # 13 corresponds to pin 33 in BOARD mode
pwm = gpio.PWM(13, 10)

# ADC setup for soil sensor
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D5)
mcp = MCP.MCP3008(spi, cs)
channel = AnalogIn(mcp, MCP.P0)

# Configuration parameters
config = {}

def sleep_until_next_noon():
    now = datetime.now()
    # Calculate the next noon
    next_noon = (now + timedelta(days=1)).replace(hour=12, minute=0, second=0, microsecond=0)
    # Calculate the sleep duration
    sleep_duration = (next_noon - now).total_seconds()
    sleep(sleep_duration)

def take_photo():
    try:
        #SETUP
        # Initialize camera
        picam2 = Camera()
        picam2.still_size = (2048, 1440)
        sleep(0.2)
        original_image = picam2.capture_array()
        image = cv2.resize(original_image, None, fx=0.5, fy=0.5)
        
        output_path = 'original.png'
        cv2.imwrite(output_path, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

        # Convert image to HSV
        image_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        # Define range for yellow in HSV (including darker shades)
        lower_yellow = np.array([10, 70, 20])  # Lower bound for dark yellow
        upper_yellow = np.array([25, 255, 255])  # Upper bound for bright yellow

        # Define range for green in HSV (including darker shades)
        lower_green = np.array([30, 50, 30])  # Lower bound for dark green
        upper_green = np.array([90, 255, 255])  # Upper bound for bright green

        # Create a mask for yellow (includes dark and bright yellow)
        yellow_mask = cv2.inRange(image_hsv, lower_yellow, upper_yellow)

        # Create a mask for green (includes dark and bright green)
        green_mask = cv2.inRange(image_hsv, lower_green, upper_green)
        log_event(f"Image captured: {output_path}")
        analyze_masks(yellow_mask, green_mask)
        
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        # Ensure the camera is released
        del picam2  

def analyze_masks(yellow_mask, green_mask):
    try:
        global base_ratio, base_yellow
        global firstImage
    
        # Calculate number of pixels of each colour
        yellow_count = np.sum(yellow_mask > 0)
        green_count = np.sum(green_mask > 0)
        

        # Define baseline counts
        if firstImage:
            base_yellow = yellow_count
            ratio = yellow_count/green_count
            firstImage = False
            return

        # Calculate differences
        current_ratio = yellow_count/green_count
        

        ratio_diff = current_ratio - base_ratio

        print(f'Ratio difference: {ratio_diff}%')

        # Check if difference detected is significant or not
        change = False
        if ratio_diff > 10:
            log_event(f'Significant change detected in leaves color ratio! ({ratio_diff}%)')
            change = True
            
        # Decide what to do when significant change is detected
        if change == True:
          buzzer(100, 1, 1)
          
    except Exception as e:
        print(f"Error: {e}")

def read_temp_humidity():
    #Turn on the sensor, take measurements, turn it back off
    gpio.output(26, gpio.HIGH)
    sleep(1)
    result = instance.read()
    gpio.output(26, gpio.LOW)
    
    
    if result.is_valid():
        temperature, humidity = result.temperature, result.humidity
        log_event(f"air_temperature = {temperature}")
        log_event(f"air_humidity = {humidity}%")
        return temperature, humidity
    else:
        log_event("Error reading temperature and humidity")
        return None, None

def turn_on_buzzer():
    gpio.output(27, gpio.HIGH)
    sleep(0.1) # let buzzer boot up
def turn_off_buzzer():
    # turn buzzer off to minimize consumption
    gpio.output(27, gpio.LOW)
def buzzer(f, length, n):
    turn_on_buzzer()
    for i in range(n):
        pwm.start(0)
        pwm.ChangeFrequency(f)
        pwm.ChangeDutyCycle(95)
        sleep(length)
        pwm.ChangeDutyCycle(0)
        sleep(0.2)
    turn_off_buzzer()

def read_soil():
    #Turn on the sensor, take measurements, turn it back off
    gpio.output(22, gpio.HIGH)
    sleep(0.1)
    moisture = channel.value
    gpio.output(22, gpio.LOW)
    log_event(f"soil_humidity = {moisture}")
    return moisture


def log_event(message):
    #Write events to events_log.txt.
    with open("events_log.txt", "a") as log_file:
        timestamp = datetime.now().strftime("%a %b %d %H:%M:%S %Y")
        log_file.write(f"{timestamp}: {message}\n")

def load_config():
    #Load buzzer triggers from configuration.txt file
    global config
    with open("configuration.txt", "r") as file:
        for line in file:
            try:
                index, condition = line.strip().split(": ")
                index = int(index)
                if index not in config:
                    config[index] = []  # Initialize list if index not present
                config[index].append(condition)  # Add condition to list
            except ValueError:
                continue

def change_day_count(condition, addage):
    global temp_day, hum_day, soil_day, rainYesterday, temp_day_updated, hum_day_updated, soil_day_updated, needPhoto
    # Counts days of measurements being too high, if it gets to threshold it takes a photo
    if "air_humidity" in condition and not temp_day_updated:
        temp_day += addage
        temp_day_updated = True
        if temp_day < 0:
            temp_day = 0
        elif temp_day >= 2:
            temp_day = 2
            #SOMETHING WRONG TAKE A PHOTO
            needPhoto = True
            
    elif "air_temperature" in condition and not hum_day_updated:
        hum_day += addage
        hum_day_updated = True
        if hum_day < 0:
            hum_day = 0
        elif hum_day >= 5:
            hum_day = 5
            #SOMETHING WRONG TAKE A PHOTO
            needPhoto = True
            
    elif "soil_humidity" in condition and not soil_day_updated:
        soil_day += addage
        soil_day_updated = True
        if addage > 0:
            rainYesterday = True
        if soil_day < 0:
            soil_day = 0
        elif soil_day >= 3:
            soil_day = 3
            #SOMETHING WRONG TAKE A PHOTO
            needPhoto = True

def plant_unstable_conditions():
    take_photo()
    

def check_conditions(temperature, humidity, soil_moisture):
    #Check if any conditions match the configuration and buzz if it does
    for count, conditions in config.items():
        for condition in conditions:
            try:
                # if the condiitons were not satisfied it should take/add day cound (environment is stable) - unless it didn't get measured
                if ("air_humidity" in condition and humidity is None) or ("air_temperature" in condition and temperature is None) or ("soil_humidity" in condition and soil_moisture is None):
                    continue

                # check all the conditions from configurations.txt
                if eval(condition.replace("air_humidity", str(humidity))
                               .replace("air_temperature", str(temperature))
                               .replace("soil_humidity", str(soil_moisture))):
                    # if condition is met, buzz, log the buzzer event, add to day count of incompatable environment
                    buzzer(1000, 0.1, count)
                    log_event(f"buzzer {count}")
                    change_day_count(condition, 1)
                    sleep(1)
                    
            except Exception as e:
                log_event(f"Condition not set: '{condition}': {e}")



try:
    #on first wake-up load configuration and take a starting/reference photo
    load_config()
    take_photo()

    
    while True:
        #set variables for a new day
        temp_day_updated = False
        hum_day_updated = False
        soil_day_updated = False
    
    
        # Read sensor values
        temp, hum = read_temp_humidity()
        if temp is None or hum is None:
            continue
        soil_moisture = None

        # Read soil moisture only if air humidity is high - higher chance there was rain
        if hum >= 80 or rainYesterday:
            soil_moisture = read_soil()
        

        # Check conditions and trigger buzzer if necessary
        check_conditions(temp, hum, soil_moisture)

        # all day countages that did not get any extra days today (due to environment being suitable) get a day down
        change_day_count("air_humidity", -1)
        change_day_count("air_temperature", -1)
        change_day_count("soil_humidity", -1)

        if needPhoto:
            take_photo()
            needPhoto = False

        # Sleep before the next reading
        sleep(10)
        #sleep_until_next_noon()

except KeyboardInterrupt:
    # program is shutting down
    print("Program interrupted. Cleaning up...")
finally:
    # Clean up resources
    pwm.stop()
    gpio.cleanup()
    print("Resources released successfully.")

