import os
import glob
import time
import datetime
import pyrebase
from gpiozero import InputDevice
import RPi.GPIO as GPIO
import smbus
import posix
from fcntl import ioctl
from bmp280 import BMP280
import requests

config = {
    "apiKey": "AIzaSyDQYG0L0nNJJpkuPXUMILnZyktXH9PFTc4",
    "authDomain": "smart-plantcareai-system.firebaseapp.com",
    "databaseURL": "https://smart-plantcareai-system-default-rtdb.europe-west1.firebasedatabase.app/Users",
    "storageBucket": "smart-plantcareai-system.appspot.com"
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()

DEVICE = 0x23  # adresa default a senzorului bh1750
ONE_TIME_HIGH_RES_MODE_1 = 0x20  # selecteaza modul de operare pentru bh1750
bus = smbus.SMBus(1)
bmp280now = datetime.datetime.now()
bmp280 = BMP280(i2c_dev=bus)  # detecteaza senzorul bmp280

no_rain = InputDevice(18)
sol_umed = InputDevice(21)
WATERPUMP_PIN = 13
LIGHTBOOST_PIN = 20

GPIO.setmode(GPIO.BCM)
GPIO.setup(WATERPUMP_PIN, GPIO.OUT)
GPIO.setup(LIGHTBOOST_PIN, GPIO.OUT)


class AM2320:
    I2C_ADDR = 0x5c
    I2C_SLAVE = 0x0703  # adresa default a senzorului AM2320

    def __init__(self, i2cbus=1):
        self._fd = posix.open("/dev/i2c-%d" % i2cbus, posix.O_RDWR)

        ioctl(self._fd, self.I2C_SLAVE, self.I2C_ADDR)

    def __del__(self):
        posix.close(self._fd)

    @staticmethod
    def _calc_crc16(data):  # cyclic redundancy check
        crc = 0xFFFF
        for x in data:
            crc = crc ^ x
            for bit in range(0, 8):
                if (crc & 0x0001) == 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    @staticmethod
    def _combine_bytes(msb, lsb):
        return msb << 8 | lsb

    def readSensor(self):
        # trezeste senzorul AM2320, iar apoi il trece in standby pentru a nu afecta
        # citirea temperaturii prin caldura generata
        try:
            posix.write(self._fd, b'\0x00')
        except:
            pass
        time.sleep(0.001)

        # scrie la adresa 0x03, start reg = 0x00, num regs = 0x04 */
        posix.write(self._fd, b'\x03\x00\x04')
        time.sleep(0.0016)  # asteapta pentru rezultat

        # Citeste 8 biti de date
        data = bytearray(posix.read(self._fd, 8))

        # Verifica data[0] si data[1]
        if data[0] != 0x03 or data[1] != 0x04:
            raise Exception("Eroare la primii 2 biti")

        # verificare CRC
        if self._calc_crc16(data[0:6]) != self._combine_bytes(data[7], data[6]):
            raise Exception("CRC esuat")

        temp = self._combine_bytes(data[4], data[5])

        if temp & 0x8000:
            temp = -(temp & 0x7FFF)

        temp /= 10.0
        humi = self._combine_bytes(data[2], data[3]) / 10.0
        return (temp, humi)


def get_ip():
    response = requests.get('https://api64.ipify.org?format=json').json()
    return response["ip"]


def convertToNumber(data):
    # Functie pentru a converti 2 bytes de date
    # intr-un numar zecimal
    result = (data[1] + (256 * data[0])) / 1.2
    return (result)


def readLight(addr=DEVICE):
    data = bus.read_i2c_block_data(addr, ONE_TIME_HIGH_RES_MODE_1)
    return convertToNumber(data)


def watering():
    now = datetime.datetime.now()
    interval = db.child("interval").get()
    if now.minute == 0:
        if interval.val() == 8:
            if now.hour % 7 == 0:
                if sol_umed.is_active:
                    GPIO.output(WATERPUMP_PIN, GPIO.LOW)
                    time.sleep(10)
                    GPIO.output(WATERPUMP_PIN, GPIO.HIGH)
                else:
                    if interval.val() == 12:
                        if now.hour % 11 == 0:
                            if sol_umed.is_active:
                                GPIO.output(WATERPUMP_PIN, GPIO.LOW)
                                time.sleep(10)
                                GPIO.output(WATERPUMP_PIN, GPIO.HIGH)
                    else:
                        if now.hour == 7:
                            if sol_umed.is_active:
                                GPIO.output(WATERPUMP_PIN, GPIO.LOW)
                                time.sleep(10)
                                GPIO.output(WATERPUMP_PIN, GPIO.HIGH)


def boostLight():
    boost = db.child("boost").get()
    okb = 0
    if boost.val() == 1:
        okb = 1
    else:
        okb = 0
    light_pure = readLight()
    light = round(light_pure, 2)
    now = datetime.datetime.now()
    if now.hour > 5 and now.hour < 21:
        print(okb)
        if okb == 1 and light < 1000:
            GPIO.output(LIGHTBOOST_PIN, GPIO.HIGH)
        else:
            GPIO.output(LIGHTBOOST_PIN, GPIO.LOW)
    else:
        GPIO.output(LIGHTBOOST_PIN, GPIO.LOW)


latitude = 0
longitude = 0
if latitude == 0 and longitude == 0:
    ip_address = get_ip()
    response = requests.get(f'https://ipapi.co/{ip_address}/json/').json()  # preia locatia de la API
    latitude = response.get("latitude")
    longitude = response.get("longitude")

ID = 1
name = "Oregano"

while True:

    boostLight()

    light_pure = readLight()
    light = round(light_pure, 2)

    if not no_rain.is_active:
        rain = 1
    else:
        rain = 0

    pressure = round(bmp280.get_pressure(), 2)

    temperature1 = bmp280.get_temperature()

    try:
        am2320 = AM2320(1)
        (temperature2, humidity) = am2320.readSensor()
        temperature = round((temperature1 + temperature2) / 2, 1)
        print(temperature)
    except:
        temperature = temperature1
        print("eroare senzor am2320")

    temperature = round(temperature, 2)

    boost = db.child("boost").get()
    if boost.val() == 1:
        okb = 1
    else:
        okb = 0
    interval = 8
    inter = db.child("interval").get()
    if inter.val() == 8:
        interval = 8
    if inter.val() == 12:
        interval = 12
    if inter.val() == 24:
        interval = 24

    data = {
        "id": ID,
        "name": name,
        "boost": okb,
        "temperature": temperature,
        "rain": rain,
        "light": light,
        "humidity": humidity,
        "pressure": pressure,
        "latitude": latitude,
        "longitude": longitude,
        "interval": interval
    }

    db.set(data)
    time.sleep(5)
