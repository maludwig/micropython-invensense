from machine import Pin, SPI

from devices import device_name_by_who_am_i
from mpu9255 import MPU9255
from time import sleep_ms

ONE_MHZ = 10000000
SAMPLE_RATE_DIVIDER = 5

sck_pin = Pin(18)
mosi_pin = Pin(23)
miso_pin = Pin(19)

cs_pin_num = 5
cs_pin = Pin(cs_pin_num)
cs_pin_mask = 1 << cs_pin_num

hspi = SPI(1, ONE_MHZ, sck=sck_pin, mosi=mosi_pin, miso=miso_pin)
mpu = MPU9255(spi=hspi, cs_pin=cs_pin)
mpu.begin()
who_am_i = mpu.who_am_i()
print("Who Am I: {} aka {}".format(who_am_i, device_name_by_who_am_i(who_am_i)))

mpu.set_sample_rate_divider(SAMPLE_RATE_DIVIDER)
mpu.enable_fifo()
fifo_buf_discard = mpu.get_fifo_data()
print(list(fifo_buf_discard))


mag_max, mag_min, mag_last_good = [0, 0, 0], [0, 0, 0], [0, 0, 0]
raw_y, raw_x, raw_z = mpu.read_ak8963_sensors()
raw_z *= -1
print("Magnetometer Readings (X,Y,Z) ({},{},{})".format(raw_x, raw_y, raw_z))


def pretty_print_sensors():
    acc_x, acc_y, acc_z, temp, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = mpu.get_pretty_sensor_data()
    print(
        "Readings (X,Y,Z) Accelerometer (g): ({},{},{}), Temp (C): {}, Gyroscope (deg/s): ({},{},{}), Magnetometer (uT): ({},{},{})".format(
            acc_x, acc_y, acc_z, temp, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z
        )
    )


def loop_pretty_sensors():
    while True:
        sleep_ms(100)
        pretty_print_sensors()
