from sub_layers.abstract_sub import AUV
from sub_layers.pin_control import PinControl
from sub_layers.user_sub import UserSub


if __name__ == '__main__':

    sub = UserSub(
        PinControl(
            AUV(
                wanted_depth=10,
                wanted_angle=0,
                motor_min=5,  # minimum motor speed
                Dp=0.6,
                Di=0.1,
                Dd=0.1,
                Rp=0.6,
                Ri=0.0,
                Rd=0.1
            ),
            motor_port='/dev/ttyUSB0',  # example port
            depth_bus=1,  # I2C bus
            imu_port='/dev/ttyUSB1',  # another port
            imu_baud=921600
        ),
        Fp=0.5,
        Fi=0.0,
        Fd=0.1,
        max_stop_speed=0.1
    )

    sub.move_time(10, 10)
