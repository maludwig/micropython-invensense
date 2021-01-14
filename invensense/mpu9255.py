from machine import Pin, SPI

from devices import device_name_by_who_am_i
from mpu9250_registers import MPU9250Registers
from ustruct import unpack
from utime import sleep_ms, sleep_us


AK8963_DATA_OK = 0x10

WHOAMI_READ = 0x75 | 0x80
SPI_READ = 0x80

ACCEL_16G_SCALE = 16.0 / 32768.0
TEMP_SCALE = 333.87
TEMP_OFFSET_C = 21.0
GYRO_2000DPS_SCALE = 2000.0 / 32768.0
MAG_SCALE = 4912.0 / 32768.0


class UnexpectedDataException(Exception):
    pass


class MPU9255:
    SPI_SLOW_CLOCK = 1_000_000
    SPI_FAST_CLOCK = 20_000_000
    FIFO_SIZE = 21

    def __init__(self, spi: SPI, cs_pin: Pin):
        self.gyro_scale = 2000.0 / 32768.0
        self.accel_scale = 16.0 / 32768.0
        self.mag_x_adj = MAG_SCALE
        self.mag_y_adj = MAG_SCALE
        self.mag_z_adj = MAG_SCALE
        self.spi = spi
        self.cs_pin = cs_pin
        self.cs_pin.init(mode=Pin.OUT)
        self.set_baudrate(MPU9255.SPI_SLOW_CLOCK)
        self.cs_pin.on()
        sleep_us(1)
        self.cs_pin.off()
        sleep_us(1)
        self.cs_pin.on()

    def begin(self):
        # Select Phase-Locked Loop clock source
        self.write_register(MPU9250Registers.PWR_MGMNT_1, MPU9250Registers.CLOCK_SEL_PLL)
        # Enable I2C slave driver
        self.write_register(MPU9250Registers.USER_CTRL, MPU9250Registers.I2C_MST_EN)
        # 400KHz I2C passthrough
        self.write_register(MPU9250Registers.I2C_MST_CTRL, MPU9250Registers.I2C_MST_CLK)

        # Power down magnetometer
        self.write_ak8963_register(MPU9250Registers.AK8963_CNTL1, MPU9250Registers.AK8963_PWR_DOWN)
        # Reset MPU9250
        self.write_register(MPU9250Registers.PWR_MGMNT_1, MPU9250Registers.PWR_RESET)
        sleep_ms(1)
        # Reset magnetometer
        self.write_ak8963_register(MPU9250Registers.AK8963_CNTL2, MPU9250Registers.AK8963_RESET)
        # Clock source to gyro
        self.write_register(MPU9250Registers.PWR_MGMNT_1, MPU9250Registers.CLOCK_SEL_PLL)

        # Check whoami byte
        device_model = self.device_model()
        if device_model == "MPU9250" or device_model == "MPU9255":
            print("Found {}".format(device_model))
        else:
            whoami = self.who_am_i()
            raise UnexpectedDataException("Unknown device, WHO_AM_I {} {}, Model: {}".format(whoami, hex(whoami), device_model))

        # Enable measurements on all axes
        self.write_register(MPU9250Registers.PWR_MGMNT_2, MPU9250Registers.ENABLE_ALL_AXES)
        # Set accelerometer range to 16G
        self.write_register(MPU9250Registers.ACCEL_CONFIG, MPU9250Registers.ACCEL_FS_SEL_16G)
        # Set gyro range to 2000 deg/s
        self.set_gyro_range(2000)
        # 184Hz accelerometer DLPF
        self.write_register(MPU9250Registers.ACCEL_CONFIG2, MPU9250Registers.ACCEL_DLPF_184)
        # 184Hz gyro DLPF
        self.write_register(MPU9250Registers.CONFIG, MPU9250Registers.GYRO_DLPF_184)
        # self.write_register(MPU9250Registers.CONFIG, MPU9250Registers.CONFIG_FIFO_MODE_OVERWRITE | MPU9250Registers.GYRO_DLPF_184)

        # Set sample rate divider to 1 (1kHz sample rate)
        self.set_sample_rate_divider(1)

        # Enable I2C slave driver
        self.write_register(MPU9250Registers.USER_CTRL, MPU9250Registers.I2C_MST_EN)
        # 400KHz I2C passthrough
        self.write_register(MPU9250Registers.I2C_MST_CTRL, MPU9250Registers.I2C_MST_CLK)

        who_is_ak = self.read_ak8963_register(MPU9250Registers.AK8963_WHO_AM_I)
        if who_is_ak != 0x48:
            raise UnexpectedDataException("Unknown device found in place of AK8963: WHO_AM_I {} {}".format(who_is_ak, hex(who_is_ak)))

        # Power down magnetometer
        self.write_ak8963_register(MPU9250Registers.AK8963_CNTL1, MPU9250Registers.AK8963_PWR_DOWN)
        sleep_ms(100)  # Long wait for AK8963 mode changes
        # Access FUSE ROM
        self.write_ak8963_register(MPU9250Registers.AK8963_CNTL1, MPU9250Registers.AK8963_FUSE_ROM)
        sleep_ms(100)  # Long wait for AK8963 mode changes
        # Read magnetometer axis scale adjustment registers and compute magnetometer scale factors
        axis_scale_adj_buf = self.read_ak8963_registers(MPU9250Registers.AK8963_ASA, 3)
        # print(axis_scale_adj_buf)
        self.mag_x_adj = self.calc_mag_axis_scale_adj(axis_scale_adj_buf[0])
        self.mag_y_adj = self.calc_mag_axis_scale_adj(axis_scale_adj_buf[1])
        self.mag_z_adj = self.calc_mag_axis_scale_adj(axis_scale_adj_buf[2])
        # print("X: {}, Y: {}. Z: {}".format(mag_x_adj, mag_y_adj, mag_z_adj))
        # Power down magnetometer
        self.write_ak8963_register(MPU9250Registers.AK8963_CNTL1, MPU9250Registers.AK8963_PWR_DOWN)
        sleep_ms(100)  # Long wait for AK8963 mode changes
        # Set magnetometer to 16 bit resolution, 100 Hz update rate
        self.write_ak8963_register(MPU9250Registers.AK8963_CNTL1, MPU9250Registers.AK8963_CNT_MEAS2)
        sleep_ms(100)  # Long wait for AK8963 mode changes
        # Select Phase-Locked Loop clock source
        self.write_register(MPU9250Registers.PWR_MGMNT_1, MPU9250Registers.CLOCK_SEL_PLL)

        # Get current mag reading
        mag_buf = self.read_ak8963_registers(MPU9250Registers.AK8963_HXL, 7)
        print(list(mag_buf))
        mag_status = self.read_ak8963_register(MPU9250Registers.AK8963_ST1)
        while not (mag_status & (MPU9250Registers.AK8963_DATA_READY | MPU9250Registers.AK8963_DATA_OVERFLOW)):
            mag_status = self.read_ak8963_register(MPU9250Registers.AK8963_ST1)
            sleep_ms(1)
        self.setup_ak8963_slave()
        print(list(mag_buf))

    def device_model(self):
        return device_name_by_who_am_i(self.who_am_i())

    def set_sample_rate_divider(self, div_num):
        # Set sample rate divider to div_num
        self.write_register(MPU9250Registers.SMPDIV, div_num - 1)

    def calc_mag_axis_scale_adj(self, byte_val):
        # See page 53 of the MPU-9255 register map
        return MAG_SCALE * ((((float(byte_val) - 128.0) * 0.5) / 128.0) + 1.0)

    def write_register(self, sub_addr, byte_val):
        self.set_baudrate(baudrate=MPU9255.SPI_SLOW_CLOCK)
        self.cs_pin.off()
        self.spi.write(bytearray([sub_addr, byte_val]))
        self.cs_pin.on()

    def read_register(self, sub_addr: int):
        write_buf = bytearray([sub_addr | SPI_READ, 0])
        read_buf = bytearray(2)
        self.cs_pin.off()
        self.spi.write_readinto(write_buf, read_buf)
        self.cs_pin.on()
        return read_buf[1]

    def read_registers(self, sub_addr, count):
        read_buf = bytearray(count + 1)
        self.read_registers_into(sub_addr, read_buf)
        return read_buf[1:]

    def read_registers_into(self, sub_addr, read_buf):
        write_buf = bytearray(len(read_buf))
        write_buf[0] = sub_addr | SPI_READ
        self.cs_pin.off()
        self.spi.write_readinto(write_buf, read_buf)
        self.cs_pin.on()
        return read_buf[1:]

    def write_ak8963_register(self, sub_addr, byte_val):
        self.write_register(MPU9250Registers.I2C_SLV0_ADDR, MPU9250Registers.AK8963_I2C_ADDR)
        self.write_register(MPU9250Registers.I2C_SLV0_REG, sub_addr)
        self.write_register(MPU9250Registers.I2C_SLV0_DO, byte_val)
        self.write_register(MPU9250Registers.I2C_SLV0_CTRL, MPU9250Registers.I2C_SLV0_EN | 0x01)

    def read_ak8963_register(self, sub_addr):
        return self.read_ak8963_registers(sub_addr, 1)[0]

    def read_ak8963_registers(self, sub_addr, count):
        self.write_register(MPU9250Registers.I2C_SLV0_ADDR, MPU9250Registers.AK8963_I2C_ADDR | MPU9250Registers.I2C_READ_FLAG)
        self.write_register(MPU9250Registers.I2C_SLV0_REG, sub_addr)
        self.write_register(MPU9250Registers.I2C_SLV0_CTRL, MPU9250Registers.I2C_SLV0_EN | count)
        sleep_ms(1)
        reg_vals = self.read_registers(MPU9250Registers.EXT_SENS_DATA_00, count)
        return reg_vals

    def setup_ak8963_slave(self):
        self.write_register(MPU9250Registers.I2C_SLV0_ADDR, MPU9250Registers.AK8963_I2C_ADDR | MPU9250Registers.I2C_READ_FLAG)
        self.write_register(MPU9250Registers.I2C_SLV0_REG, MPU9250Registers.AK8963_HXL)
        # Read at sample rate, Swap bytes, write a register value, swap even indexed bytes with next byte, read 7 bytes
        self.write_register(MPU9250Registers.I2C_SLV0_CTRL, 0b11010111)

    def read_ak8963_slave(self):
        reg_vals = self.read_registers(MPU9250Registers.EXT_SENS_DATA_00, 7)
        if reg_vals[6] == AK8963_DATA_OK:
            mag_xyz = unpack(">hhh", reg_vals[:6])
            return mag_xyz
        else:
            return None

    def read_ak8963_sensors(self):
        self.setup_ak8963_slave()
        sleep_ms(1)
        return self.read_ak8963_slave()

    def set_baudrate(self, baudrate):
        self.spi.init(baudrate=baudrate, polarity=0, phase=0)

    def set_accel_range(self, g_max):
        if g_max == 2:
            self.write_register(MPU9250Registers.GYRO_CONFIG, MPU9250Registers.GYRO_FS_SEL_250DPS)
        elif g_max == 4:
            self.write_register(MPU9250Registers.GYRO_CONFIG, MPU9250Registers.GYRO_FS_SEL_500DPS)
        elif g_max == 8:
            self.write_register(MPU9250Registers.GYRO_CONFIG, MPU9250Registers.GYRO_FS_SEL_1000DPS)
        elif g_max == 16:
            self.write_register(MPU9250Registers.GYRO_CONFIG, MPU9250Registers.GYRO_FS_SEL_2000DPS)
        else:
            raise Exception("Invalid gyro range selection")
        self.accel_scale = float(g_max) / 32768.0

    def set_gyro_range(self, degrees_max):
        if degrees_max == 250:
            self.write_register(MPU9250Registers.GYRO_CONFIG, MPU9250Registers.GYRO_FS_SEL_250DPS)
        elif degrees_max == 500:
            self.write_register(MPU9250Registers.GYRO_CONFIG, MPU9250Registers.GYRO_FS_SEL_500DPS)
        elif degrees_max == 1000:
            self.write_register(MPU9250Registers.GYRO_CONFIG, MPU9250Registers.GYRO_FS_SEL_1000DPS)
        elif degrees_max == 2000:
            self.write_register(MPU9250Registers.GYRO_CONFIG, MPU9250Registers.GYRO_FS_SEL_2000DPS)
        else:
            raise Exception("Invalid gyro range selection")
        self.gyro_scale = float(degrees_max) / 32768.0

    def who_am_i(self):
        return self.read_register(MPU9250Registers.WHO_AM_I)

    def enable_fifo(self):
        self.write_register(MPU9250Registers.USER_CTRL, MPU9250Registers.USER_CTRL_FIFO_EN | MPU9250Registers.I2C_MST_EN)
        self.write_register(MPU9250Registers.FIFO_EN, MPU9250Registers.FIFO_ALL_SENSORS)
        count_buf = self.read_registers(MPU9250Registers.FIFO_COUNT, 2)
        count = unpack(">h", count_buf)[0]
        self.read_registers(MPU9250Registers.FIFO_READ, count)

    def get_fifo_data(self):
        count_buf = self.read_registers(MPU9250Registers.FIFO_COUNT, 2)
        count = unpack(">h", count_buf)[0]
        if count == 0:
            return bytearray(0)
        else:
            fifo_buf = self.read_registers(MPU9250Registers.FIFO_READ, count)
            return fifo_buf

    def get_fifo_data_into(self, buf):
        count_buf = self.read_registers(MPU9250Registers.FIFO_COUNT, 2)
        count = unpack(">h", count_buf)[0]
        self.read_registers_into(MPU9250Registers.FIFO_READ, buf[0:count])
        return count

    def get_raw_sensor_data(self):
        # The registers for all sensors are adjacent to one another, and can be read in sequence
        buf = self.read_registers(MPU9250Registers.ACCEL_OUT, MPU9250Registers.FULL_OUT_SIZE)
        if buf[20] == AK8963_DATA_OK:
            # The magnetometer has different axes, this reorders and fixes them
            acc_x, acc_y, acc_z, temp, gyro_x, gyro_y, gyro_z, mag_y, mag_x, mag_z = unpack(">hhhhhhhhhh", buf)
            mag_z *= -1
            return acc_x, acc_y, acc_z, temp, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z
        else:
            raise Exception("Data from magnetometer was not OK")

    def raw_gyro_to_degrees(self, x, y, z):
        return x * self.gyro_scale, y * self.gyro_scale, z * self.gyro_scale

    def raw_accel_to_gs(self, x, y, z):
        return x * self.accel_scale, y * self.accel_scale, z * self.accel_scale

    def raw_temp_to_c(self, temp):
        return (temp / TEMP_SCALE) + TEMP_OFFSET_C

    def raw_mag_to_ut(self, x, y, z):
        return x * self.mag_x_adj, y * self.mag_y_adj, z * self.mag_z_adj

    def get_pretty_sensor_data(self):
        acc_x, acc_y, acc_z, temp, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = self.get_raw_sensor_data()
        acc_x, acc_y, acc_z = self.raw_accel_to_gs(acc_x, acc_y, acc_z)
        temp = self.raw_temp_to_c(temp)
        gyro_x, gyro_y, gyro_z = self.raw_gyro_to_degrees(gyro_x, gyro_y, gyro_z)
        mag_x, mag_y, mag_z = self.raw_mag_to_ut(mag_x, mag_y, mag_z)
        return acc_x, acc_y, acc_z, temp, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z
