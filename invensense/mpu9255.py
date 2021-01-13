from machine import Pin, SPI
from mpu9250_registers import MPU9250Registers
from ustruct import unpack
from utime import sleep_ms, sleep_us

AK8963_DATA_OK = 0x10

WHOAMI_READ = 0x75 | 0x80
SPI_READ = 0x80

ACCEL_16G_SCALE = 16.0 / 32768.0
GYRO_2000DPS_SCALE = 2000.0 / 32768.0


class MPU9255:
    SPI_SLOW_CLOCK = 1_000_000
    SPI_FAST_CLOCK = 20_000_000
    FIFO_SIZE = 21

    def __init__(self, spi: SPI, cs_pin: Pin):
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
        whoami = self.who_am_i()
        if whoami == 0x71:
            print("Found MPU9250")
        elif whoami == 0x73:
            print("Found MPU9255")
        else:
            if whoami == 0x70:
                print("Found MPU6500 (WHO_AM_I register was 0x70)")
            else:
                print("Unknown device: {} {}".format(whoami, hex(whoami)))

        # Enable measurements on all axes
        self.write_register(MPU9250Registers.PWR_MGMNT_2, MPU9250Registers.ENABLE_ALL_AXES)
        # Set accelerometer range to 16G
        self.write_register(MPU9250Registers.ACCEL_CONFIG, MPU9250Registers.ACCEL_FS_SEL_16G)
        # Set gyro range to 2000 deg/s
        self.write_register(MPU9250Registers.GYRO_CONFIG, MPU9250Registers.GYRO_FS_SEL_2000DPS)
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
        if who_is_ak == 0x48:
            print("Found AK8963")
        else:
            raise Exception("Unknown device found in place of AK8963: {}".format(who_is_ak))

        # Power down magnetometer
        self.write_ak8963_register(MPU9250Registers.AK8963_CNTL1, MPU9250Registers.AK8963_PWR_DOWN)
        sleep_ms(100)  # Long wait for AK8963 mode changes
        # Access FUSE ROM
        self.write_ak8963_register(MPU9250Registers.AK8963_CNTL1, MPU9250Registers.AK8963_FUSE_ROM)
        sleep_ms(100)  # Long wait for AK8963 mode changes
        # Read magnetometer axis scale adjustment registers and compute magnetometer scale factors
        axis_scale_adj_buf = self.read_ak8963_registers(MPU9250Registers.AK8963_ASA, 3)
        # print(axis_scale_adj_buf)
        mag_x_adj = self.calc_mag_axis_scale_adj(axis_scale_adj_buf[0])
        mag_y_adj = self.calc_mag_axis_scale_adj(axis_scale_adj_buf[1])
        mag_z_adj = self.calc_mag_axis_scale_adj(axis_scale_adj_buf[2])
        # print("X: {}, Y: {}. Z: {}".format(mag_x_adj, mag_y_adj, mag_z_adj))
        # # Power down magnetometer
        # self.write_ak8963_register(MPU9250Registers.AK8963_CNTL1, MPU9250Registers.AK8963_PWR_DOWN)
        # sleep_ms(100)  # Long wait for AK8963 mode changes
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

    def set_sample_rate_divider(self, div_num):
        # Set sample rate divider to div_num
        self.write_register(MPU9250Registers.SMPDIV, div_num - 1)

    def calc_mag_axis_scale_adj(self, byte_val):
        return (((float(byte_val) - 128.0) * 0.5) / 128.0) + 1.0

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

    def get_latest_sensor_data(self):
        # The registers for all sensors are adjacent to one another, and can be read in sequence
        buf = self.read_registers(MPU9250Registers.ACCEL_OUT, MPU9250Registers.FULL_OUT_SIZE)
        if buf[20] == AK8963_DATA_OK:
            # The magnetometer has different axes, this reorders and fixes them
            acc_x, acc_y, acc_z, temp, gyro_x, gyro_y, gyro_z, mag_y, mag_x, mag_z = unpack(">hhhhhhhhhh", buf)
            mag_z *= -1
            return acc_x, acc_y, acc_z, temp, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z
        else:
            raise Exception("Data from magnetometer was not OK")
