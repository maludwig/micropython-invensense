who_am_i_map = {
    0x71: "MPU9250",
    0x73: "MPU9255",
    0x70: "MPU6500",
}


def device_name_by_who_am_i(who_am_i):
    if who_am_i in who_am_i_map:
        return who_am_i_map[who_am_i]
    else:
        return "UNKNOWN"


print(device_name_by_who_am_i(115))
