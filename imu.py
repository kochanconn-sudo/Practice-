import smbus
import time
import struct
import math
from collections import deque
from abc import ABC, abstractmethod
import config

def detect_imu_type(cfg):
    """I2CバスをスキャンしてIMUタイプを自動検出する"""
    bus_number = getattr(cfg, 'I2C_BUS', 1)
    try:
        bus = smbus.SMBus(bus_number)
        # BNO085: 0x4A or 0x4B
        for addr in [0x4A, 0x4B]:
            try:
                bus.read_byte(addr)
                bus.close()
                return "BNO085"
            except OSError:
                pass
        # BNO055: 0x28 or 0x29
        for addr in [0x28, 0x29]:
            try:
                bus.read_byte(addr)
                bus.close()
                return "BNO055"
            except OSError:
                pass
        bus.close()
    except Exception:
        pass
    return "NONE"


class IMUBase(ABC):
    """抽象クラス: IMU センサの基本クラス"""
    def __init__(self, memory_size=3):
        self.memory_size = memory_size
        self.imu_acceleration = {axis: deque([0.0] * memory_size, maxlen=memory_size) for axis in "xyz"}  # 加速度データ
        self.imu_angular_velocity = {axis: deque([0.0] * memory_size, maxlen=memory_size) for axis in "xyz"}  # 角速度データ
        self.imu_jerk = {axis: deque([0.0] * memory_size, maxlen=memory_size) for axis in "xyz"}  # ジャークデータ
        self.imu_angle = {"x": 0.0, "y": 0.0, "z": 0.0}  # 積算した角度、BNO055では推定値
        self.imu_velocity = {"x": 0.0, "y": 0.0, "z": 0.0}  # 速度
        self.imu_position = {"x": 0.0, "y": 0.0, "z": 0.0}  # 位置
        self.imu_start_time = time.perf_counter()  # 計測開始時刻

        # バイアス補正用
        self.bias = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.is_initialized = False
        
        # カルマンフィルタのインスタンスを各軸に作成
        self.kalman_filters_acc = {axis: KalmanFilter(1e-4, 1e-2) for axis in "xyz"}
        self.kalman_filters_gyr = {axis: KalmanFilter(1e-4, 1e-2) for axis in "xyz"}

    def apply_kalman_filter(self, data, kalman_filters):
        """
        カルマンフィルタを適用
        Args:
            data (dict): IMUデータ（加速度または角速度）。
            kalman_filters (dict): 対応するカルマンフィルタのインスタンス。
        Returns:
            dict: フィルタリングされたデータ。
        """
        filtered_data = {}
        for axis in "xyz":
            filtered_data[axis] = kalman_filters[axis].update(data[axis][-1])
        return filtered_data

    @abstractmethod
    def get_raw_data(self):
        """センサーから生データを取得するメソッド"""
        pass
            
    def initialize(self, duration=8.0, discard_samples=50):
        """IMUの初期化（バイアス推定）"""
        print("Initializing IMU...")
        start_time = time.perf_counter()
        samples = {"x": [], "y": [], "z": []}
        discarded_count = 0

        while time.perf_counter() - start_time < duration:
            imu_raw_acceleration, _ = self.get_raw_data()

            # 最初のdiscard_samples個のサンプルをスキップ
            if discarded_count < discard_samples:
                discarded_count += 1
            else:
                for i, axis in enumerate("xyz"):
                    samples[axis].append(imu_raw_acceleration[i])

            time.sleep(0.01)  # サンプリング間隔

        # バイアスを計算
        for axis in "xyz":
            if samples[axis]:  # サンプルが存在する場合に計算
                self.bias[axis] = sum(samples[axis]) / len(samples[axis])
            else:
                self.bias[axis] = 0.0  # サンプルがない場合のデフォルト値
        self.is_initialized = True
        # 初期化中に経過した時間をリセットし、最初のmeasure()で巨大なdtにならないようにする
        self.imu_start_time = time.perf_counter()
        print(f"IMU initialized with bias: {self.bias}")

    def calculate_jerk(self, dt):
        """加加速度（ジャーク）の計算"""
        for axis in self.imu_acceleration:
            self.imu_jerk[axis].append(
                (self.imu_acceleration[axis][-1] - self.imu_acceleration[axis][-2]) / dt
            )

    def calculate_angle_from_gyro(self, dt):
        """角速度から角度を積算"""
        for axis in "xyz":
            self.imu_angle[axis] += self.imu_angular_velocity[axis][-1] * dt

    def calculate_velocity_and_position(self, dt):
        """加速度から速度と位置を積算"""
        for axis in "xyz":
            # バイアス補正を適用
            corrected_acceleration = self.imu_acceleration[axis][-1] - self.bias[axis]

            # フィルタリング（移動平均）
            filtered_acceleration = (
                sum(list(self.imu_acceleration[axis])[-self.memory_size:]) / self.memory_size
            )
            
            # バイアス補正後の加速度にフィルタリングを適用（必要に応じて選択）
            # corrected_acceleration or filtered_acceleration
            acceleration_to_use = corrected_acceleration  
            
            # 速度を更新
            #print(self.imu_velocity[axis], " : ",acceleration_to_use * dt)
            self.imu_velocity[axis] += acceleration_to_use * dt


            # 位置を更新
            self.imu_position[axis] += self.imu_velocity[axis] * dt

    def measure(self):
        """データの計測と処理"""
        imu_end_time = time.perf_counter()
        dt = imu_end_time - self.imu_start_time
        imu_raw_acceleration, imu_raw_angular_velocity = self.get_raw_data()

        # データを更新
        for i, axis in enumerate("xyz"):
            self.imu_acceleration[axis].append(imu_raw_acceleration[i])
            self.imu_angular_velocity[axis].append(imu_raw_angular_velocity[i])

        """# データ更新とカルマンフィルタ適用
        filtered_acceleration = {}
        filtered_angular_velocity = {}
        for i, axis in enumerate("xyz"):
            # フィルタリング処理
            filtered_acceleration[axis] = self.kalman_filters_acc[axis].update(imu_raw_acceleration[i])
            filtered_angular_velocity[axis] = self.kalman_filters_gyr[axis].update(imu_raw_angular_velocity[i])

            # フィルタリング後の値をキューに追加
            self.imu_acceleration[axis].append(filtered_acceleration[axis])
            self.imu_angular_velocity[axis].append(filtered_angular_velocity[axis])
        """

        # 各種計算
        self.calculate_jerk(dt)
        self.calculate_angle_from_gyro(dt)
        self.calculate_velocity_and_position(dt)

        # 時間を更新
        self.imu_start_time = imu_end_time
        return self.imu_acceleration, self.imu_angular_velocity, self.imu_angle, self.imu_jerk, self.imu_velocity, self.imu_position

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        """
        カルマンフィルタの初期化
        Args:
            process_variance (float): プロセスノイズ（システムの変化に関する不確実性）。
            measurement_variance (float): 観測ノイズ（センサデータの不確実性）。
        """
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = 0.0  # 初期推定値
        self.error_covariance = 1.0  # 初期誤差共分散

    def update(self, measurement):
        """
        新しい観測値でフィルタを更新
        Args:
            measurement (float): 新しい観測値。
        Returns:
            float: フィルタリングされた値。
        """
        # カルマンゲイン計算
        kalman_gain = self.error_covariance / (self.error_covariance + self.measurement_variance)

        # 推定値の更新
        self.estimate += kalman_gain * (measurement - self.estimate)

        # 誤差共分散の更新
        self.error_covariance = (1 - kalman_gain) * self.error_covariance + self.process_variance

        return self.estimate


# Modified from https://github.com/ghirlekar/bno055-python-i2c.git, MIT lisence
class BNO055(IMUBase):
    BNO055_ADDRESS_A                 = 0x28
    BNO055_ADDRESS_B                 = 0x29
    BNO055_ID                      = 0xA0

    # Power mode settings
    POWER_MODE_NORMAL                   = 0X00
    POWER_MODE_LOWPOWER                 = 0X01
    POWER_MODE_SUSPEND                  = 0X02

    # Operation mode settings
    OPERATION_MODE_CONFIG                 = 0X00
    OPERATION_MODE_ACCONLY                 = 0X01
    OPERATION_MODE_MAGONLY                 = 0X02
    OPERATION_MODE_GYRONLY                 = 0X03
    OPERATION_MODE_ACCMAG                 = 0X04
    OPERATION_MODE_ACCGYRO                 = 0X05
    OPERATION_MODE_MAGGYRO                 = 0X06
    OPERATION_MODE_AMG                 = 0X07
    OPERATION_MODE_IMUPLUS                 = 0X08
    OPERATION_MODE_COMPASS                 = 0X09
    OPERATION_MODE_M4G                 = 0X0A
    OPERATION_MODE_NDOF_FMC_OFF             = 0X0B
    OPERATION_MODE_NDOF                 = 0X0C

    # Output vector type
    VECTOR_ACCELEROMETER                 = 0x08
    VECTOR_MAGNETOMETER                  = 0x0E
    VECTOR_GYROSCOPE                     = 0x14
    VECTOR_EULER                         = 0x1A
    VECTOR_LINEARACCEL                   = 0x28
    VECTOR_GRAVITY                       = 0x2E

    # REGISTER DEFINITION START
    BNO055_PAGE_ID_ADDR                 = 0X07

    BNO055_CHIP_ID_ADDR                 = 0x00
    BNO055_ACCEL_REV_ID_ADDR             = 0x01
    BNO055_MAG_REV_ID_ADDR                 = 0x02
    BNO055_GYRO_REV_ID_ADDR             = 0x03
    BNO055_SW_REV_ID_LSB_ADDR             = 0x04
    BNO055_SW_REV_ID_MSB_ADDR             = 0x05
    BNO055_BL_REV_ID_ADDR                 = 0X06

    # Accel data register 
    BNO055_ACCEL_DATA_X_LSB_ADDR             = 0X08
    BNO055_ACCEL_DATA_X_MSB_ADDR             = 0X09
    BNO055_ACCEL_DATA_Y_LSB_ADDR             = 0X0A
    BNO055_ACCEL_DATA_Y_MSB_ADDR             = 0X0B
    BNO055_ACCEL_DATA_Z_LSB_ADDR             = 0X0C
    BNO055_ACCEL_DATA_Z_MSB_ADDR             = 0X0D

    # Mag data register 
    BNO055_MAG_DATA_X_LSB_ADDR             = 0X0E
    BNO055_MAG_DATA_X_MSB_ADDR             = 0X0F
    BNO055_MAG_DATA_Y_LSB_ADDR             = 0X10
    BNO055_MAG_DATA_Y_MSB_ADDR             = 0X11
    BNO055_MAG_DATA_Z_LSB_ADDR             = 0X12
    BNO055_MAG_DATA_Z_MSB_ADDR            = 0X13

    # Gyro data registers 
    BNO055_GYRO_DATA_X_LSB_ADDR             = 0X14
    BNO055_GYRO_DATA_X_MSB_ADDR             = 0X15
    BNO055_GYRO_DATA_Y_LSB_ADDR             = 0X16
    BNO055_GYRO_DATA_Y_MSB_ADDR             = 0X17
    BNO055_GYRO_DATA_Z_LSB_ADDR             = 0X18
    BNO055_GYRO_DATA_Z_MSB_ADDR             = 0X19
    
    # Euler data registers 
    BNO055_EULER_H_LSB_ADDR             = 0X1A
    BNO055_EULER_H_MSB_ADDR             = 0X1B
    BNO055_EULER_R_LSB_ADDR             = 0X1C
    BNO055_EULER_R_MSB_ADDR             = 0X1D
    BNO055_EULER_P_LSB_ADDR             = 0X1E
    BNO055_EULER_P_MSB_ADDR             = 0X1F

    # Quaternion data registers 
    BNO055_QUATERNION_DATA_W_LSB_ADDR         = 0X20
    BNO055_QUATERNION_DATA_W_MSB_ADDR         = 0X21
    BNO055_QUATERNION_DATA_X_LSB_ADDR         = 0X22
    BNO055_QUATERNION_DATA_X_MSB_ADDR         = 0X23
    BNO055_QUATERNION_DATA_Y_LSB_ADDR         = 0X24
    BNO055_QUATERNION_DATA_Y_MSB_ADDR         = 0X25
    BNO055_QUATERNION_DATA_Z_LSB_ADDR         = 0X26
    BNO055_QUATERNION_DATA_Z_MSB_ADDR         = 0X27

    # Linear acceleration data registers 
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR         = 0X28
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR         = 0X29
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR         = 0X2A
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR        = 0X2B
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR        = 0X2C
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR        = 0X2D

    # Gravity data registers 
    BNO055_GRAVITY_DATA_X_LSB_ADDR             = 0X2E
    BNO055_GRAVITY_DATA_X_MSB_ADDR             = 0X2F
    BNO055_GRAVITY_DATA_Y_LSB_ADDR             = 0X30
    BNO055_GRAVITY_DATA_Y_MSB_ADDR             = 0X31
    BNO055_GRAVITY_DATA_Z_LSB_ADDR             = 0X32
    BNO055_GRAVITY_DATA_Z_MSB_ADDR             = 0X33

    # Temperature data register 
    BNO055_TEMP_ADDR                 = 0X34

    # Status registers 
    BNO055_CALIB_STAT_ADDR                 = 0X35
    BNO055_SELFTEST_RESULT_ADDR             = 0X36
    BNO055_INTR_STAT_ADDR                 = 0X37

    BNO055_SYS_CLK_STAT_ADDR             = 0X38
    BNO055_SYS_STAT_ADDR                 = 0X39
    BNO055_SYS_ERR_ADDR                 = 0X3A

    # Unit selection register 
    BNO055_UNIT_SEL_ADDR                 = 0X3B
    BNO055_DATA_SELECT_ADDR             = 0X3C

    # Mode registers 
    BNO055_OPR_MODE_ADDR                 = 0X3D
    BNO055_PWR_MODE_ADDR                 = 0X3E

    BNO055_SYS_TRIGGER_ADDR             = 0X3F
    BNO055_TEMP_SOURCE_ADDR             = 0X40

    # Axis remap registers 
    BNO055_AXIS_MAP_CONFIG_ADDR             = 0X41
    BNO055_AXIS_MAP_SIGN_ADDR             = 0X42

    # SIC registers 
    BNO055_SIC_MATRIX_0_LSB_ADDR             = 0X43
    BNO055_SIC_MATRIX_0_MSB_ADDR             = 0X44
    BNO055_SIC_MATRIX_1_LSB_ADDR             = 0X45
    BNO055_SIC_MATRIX_1_MSB_ADDR             = 0X46
    BNO055_SIC_MATRIX_2_LSB_ADDR             = 0X47
    BNO055_SIC_MATRIX_2_MSB_ADDR             = 0X48
    BNO055_SIC_MATRIX_3_LSB_ADDR             = 0X49
    BNO055_SIC_MATRIX_3_MSB_ADDR             = 0X4A
    BNO055_SIC_MATRIX_4_LSB_ADDR             = 0X4B
    BNO055_SIC_MATRIX_4_MSB_ADDR             = 0X4C
    BNO055_SIC_MATRIX_5_LSB_ADDR             = 0X4D
    BNO055_SIC_MATRIX_5_MSB_ADDR             = 0X4E
    BNO055_SIC_MATRIX_6_LSB_ADDR             = 0X4F
    BNO055_SIC_MATRIX_6_MSB_ADDR             = 0X50
    BNO055_SIC_MATRIX_7_LSB_ADDR             = 0X51
    BNO055_SIC_MATRIX_7_MSB_ADDR             = 0X52
    BNO055_SIC_MATRIX_8_LSB_ADDR             = 0X53
    BNO055_SIC_MATRIX_8_MSB_ADDR             = 0X54
    
    # Accelerometer Offset registers     
    ACCEL_OFFSET_X_LSB_ADDR             = 0X55
    ACCEL_OFFSET_X_MSB_ADDR             = 0X56
    ACCEL_OFFSET_Y_LSB_ADDR             = 0X57
    ACCEL_OFFSET_Y_MSB_ADDR             = 0X58
    ACCEL_OFFSET_Z_LSB_ADDR             = 0X59
    ACCEL_OFFSET_Z_MSB_ADDR             = 0X5A

    # Magnetometer Offset registers 
    MAG_OFFSET_X_LSB_ADDR                 = 0X5B
    MAG_OFFSET_X_MSB_ADDR                 = 0X5C
    MAG_OFFSET_Y_LSB_ADDR                 = 0X5D
    MAG_OFFSET_Y_MSB_ADDR                 = 0X5E
    MAG_OFFSET_Z_LSB_ADDR                 = 0X5F
    MAG_OFFSET_Z_MSB_ADDR                 = 0X60

    # Gyroscope Offset registers
    GYRO_OFFSET_X_LSB_ADDR                 = 0X61
    GYRO_OFFSET_X_MSB_ADDR                 = 0X62
    GYRO_OFFSET_Y_LSB_ADDR                 = 0X63
    GYRO_OFFSET_Y_MSB_ADDR                 = 0X64
    GYRO_OFFSET_Z_LSB_ADDR                 = 0X65
    GYRO_OFFSET_Z_MSB_ADDR                 = 0X66

    # Radius registers 
    ACCEL_RADIUS_LSB_ADDR                 = 0X67
    ACCEL_RADIUS_MSB_ADDR                 = 0X68
    MAG_RADIUS_LSB_ADDR                 = 0X69
    MAG_RADIUS_MSB_ADDR                 = 0X6A

    # REGISTER DEFINITION END

    def __init__(self, sensorId=-1, address=0x28):
        super().__init__()
        print("BNO055, 9axis senser fusion module")
        self._sensorId = sensorId
        self._address = address
        self._mode = BNO055.OPERATION_MODE_NDOF
        self._last_valid_acc = (0.0, 0.0, 9.8)  # 加速度バリデーション用

        print(" Please wait for few secs to initialize...")
        if self.begin(self._mode) is not True:
            raise OSError("BNO055: Error initializing device")
        time.sleep(1)
        self.setExternalCrystalUse(True)


    def begin(self, mode=None):
        if mode is None: mode = BNO055.OPERATION_MODE_NDOF
        # Open I2C bus
        self._bus = smbus.SMBus(config.I2C_BUS)

        # Make sure we have the right device
        if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
            time.sleep(1)    # Wait for the device to boot up
            if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
                return False

        # Switch to config mode
        self.setMode(BNO055.OPERATION_MODE_CONFIG)

        # Trigger a reset and wait for the device to boot up again
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x20])
        time.sleep(1)
        while self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
            time.sleep(0.01)
        time.sleep(0.05)

        ### add fix axis remap for akizuki BNO055 module
        #### def
        #self.writeBytes(0x41, [0x24])
        #self.writeBytes(0x42, [0x00])
        #### remap
        #self.writeBytes(0x41, [0x21])
        #self.writeBytes(0x42, [0x07])

        # Set to normal power mode
        self.writeBytes(BNO055.BNO055_PWR_MODE_ADDR, [BNO055.POWER_MODE_NORMAL])
        time.sleep(0.01)

        self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0])
        time.sleep(0.01)

        # Set the requested mode
        self.setMode(mode)
        time.sleep(0.02)

        return True

    def setMode(self, mode):
        self._mode = mode
        self.writeBytes(BNO055.BNO055_OPR_MODE_ADDR, [self._mode])
        time.sleep(0.03)

    def setExternalCrystalUse(self, useExternalCrystal = True):
        prevMode = self._mode
        self.setMode(BNO055.OPERATION_MODE_CONFIG)
        time.sleep(0.025)
        self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x80] if useExternalCrystal else [0])
        time.sleep(0.01)
        self.setMode(prevMode)
        time.sleep(0.02)

    def getSystemStatus(self):
        self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
        (sys_stat, sys_err) = self.readBytes(BNO055.BNO055_SYS_STAT_ADDR, 2)
        self_test = self.readBytes(BNO055.BNO055_SELFTEST_RESULT_ADDR)[0]
        return (sys_stat, self_test, sys_err)

    def getRevInfo(self):
        (accel_rev, mag_rev, gyro_rev) = self.readBytes(BNO055.BNO055_ACCEL_REV_ID_ADDR, 3)
        sw_rev = self.readBytes(BNO055.BNO055_SW_REV_ID_LSB_ADDR, 2)
        sw_rev = sw_rev[0] | sw_rev[1] << 8
        bl_rev = self.readBytes(BNO055.BNO055_BL_REV_ID_ADDR)[0]
        return (accel_rev, mag_rev, gyro_rev, sw_rev, bl_rev)

    def getCalibration(self):
        calData = self.readBytes(BNO055.BNO055_CALIB_STAT_ADDR)[0]
        return (calData >> 6 & 0x03, calData >> 4 & 0x03, calData >> 2 & 0x03, calData & 0x03)

    def getTemp(self):
        return self.readBytes(BNO055.BNO055_TEMP_ADDR)[0]

    def getVector(self, vectorType):
        buf = self.readBytes(vectorType, 6)
        xyz = struct.unpack('hhh', struct.pack('BBBBBB', buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]))
        if vectorType == BNO055.VECTOR_MAGNETOMETER:    scalingFactor = 16.0
        elif vectorType == BNO055.VECTOR_GYROSCOPE:    scalingFactor = 900.0
        elif vectorType == BNO055.VECTOR_EULER:         scalingFactor = 16.0
        elif vectorType == BNO055.VECTOR_GRAVITY:       scalingFactor = 100.0
        elif vectorType == BNO055.VECTOR_ACCELEROMETER: scalingFactor = 100.0
        elif vectorType == BNO055.VECTOR_LINEARACCEL:   scalingFactor = 100.0
        else:                                            scalingFactor = 1.0
        return tuple([i/scalingFactor for i in xyz])

    def getQuat(self):
        buf = self.readBytes(BNO055.BNO055_QUATERNION_DATA_W_LSB_ADDR, 8)
        wxyz = struct.unpack('hhhh', struct.pack('BBBBBBBB', buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]))
        return tuple([i * (1.0 / (1 << 14)) for i in wxyz])

    def readBytes(self, register, numBytes=1):
        return self._bus.read_i2c_block_data(self._address, register, numBytes)

    def writeBytes(self, register, byteVals):
        return self._bus.write_i2c_block_data(self._address, register, byteVals)
    
    ### add for togikaidrive
    def get_raw_data(self):
        """
        BNO055から加速度および角速度データを取得。
        IMUBaseで継承される measure メソッドが呼び出す。
        """
        try:
            imu_acc = self.getVector(self.VECTOR_ACCELEROMETER)
            # 加速度ノルム検証 (静止時 ~9.8 m/s², 許容 7~13 m/s²)
            mag2 = imu_acc[0]**2 + imu_acc[1]**2 + imu_acc[2]**2
            if not (49.0 < mag2 < 169.0):
                imu_acc = self._last_valid_acc
            else:
                self._last_valid_acc = imu_acc
        except OSError:
            imu_acc = self._last_valid_acc
        try:
            imu_gyr = self.getVector(self.VECTOR_GYROSCOPE)
        except OSError:
            imu_gyr = (0.0, 0.0, 0.0)
        return imu_acc, imu_gyr

    def initialize(self, duration=6.0, discard_samples=50):
        """BNO055用初期化: 加速度バイアス + yawオフセット"""
        super().initialize(duration=duration, discard_samples=discard_samples)
        # Euler角が安定するまで少し読む
        for _ in range(20):
            self.get_raw_data()
            time.sleep(0.02)
        # 現在のHeadingをゼロ基準にする
        imu_orientation = self.getVector(self.VECTOR_EULER)
        self._yaw_offset = imu_orientation[0]
        print(f"  yaw offset: {self._yaw_offset:.2f}°")

    def calculate_angle_from_gyro(self, dt):
        """BNO055では直接角度を取得し、起動時のHeadingをゼロ基準にする"""
        try:
            imu_orientation = self.getVector(self.VECTOR_EULER)
        except OSError:
            return  # I2Cエラー時は前回値を維持
        yaw = imu_orientation[0] - getattr(self, '_yaw_offset', 0.0)
        # -180°〜+180° に正規化
        if yaw > 180.0:
            yaw -= 360.0
        elif yaw < -180.0:
            yaw += 360.0
        self.imu_angle = {
            "x": imu_orientation[2],
            "y": imu_orientation[1],
            "z": yaw
        }


class BNO085(IMUBase):
    """BNO085 9-axis IMU sensor (raw I2C SHTP protocol)"""

    _CHANNEL_CONTROL = 2
    _CHANNEL_INPUT_SENSOR = 3
    _REPORT_SET_FEATURE = 0xFD
    _REPORT_ROTATION_VECTOR = 0x05
    _REPORT_GAME_ROTATION_VECTOR = 0x08
    _REPORT_GYROSCOPE = 0x02
    _REPORT_ACCELEROMETER = 0x01
    _REPORT_LINEAR_ACCELERATION = 0x04

    def __init__(self, bus_number=None, address=0x4A):
        super().__init__()
        print("BNO085, 9axis sensor fusion module")

        import smbus2
        from smbus2 import i2c_msg
        self._smbus2 = smbus2
        self._i2c_msg = i2c_msg

        if bus_number is None:
            bus_number = config.I2C_BUS
        self._bus = smbus2.SMBus(bus_number)
        self._address = address
        self._sequence = [0] * 6
        self._game_quat = (0.0, 0.0, 0.0, 1.0)
        self._gyro_raw = (0.0, 0.0, 0.0)
        self._accel_raw = (0.0, 0.0, 0.0)
        self._linear_accel_raw = (0.0, 0.0, 0.0)

        # yaw補正パラメータ (initialize()で設定)
        self._yaw_offset = 0.0        # 起動時yawをゼロにするオフセット
        self._yaw_drift_rate = 0.0    # ドリフト速度 [°/s]
        self._yaw_calib_time = None   # キャリブレーション完了時刻

        print(" Please wait for few secs to initialize...")
        time.sleep(1.0)
        self._flush_startup()

        # Enable sensor reports (GAME_ROTATION_VECTOR: gyro+accel, no magnetometer interference)
        self._enable_feature(self._REPORT_GAME_ROTATION_VECTOR, interval_us=20000)
        self._enable_feature(self._REPORT_ACCELEROMETER, interval_us=20000)
        self._enable_feature(self._REPORT_GYROSCOPE, interval_us=20000)
        time.sleep(0.5)

    def _raw_read(self, length):
        msg = self._i2c_msg.read(self._address, length)
        self._bus.i2c_rdwr(msg)
        return list(msg)

    def _raw_write(self, data):
        msg = self._i2c_msg.write(self._address, data)
        self._bus.i2c_rdwr(msg)

    def _flush_startup(self):
        for _ in range(10):
            try:
                self._read_packet()
                time.sleep(0.05)
            except Exception:
                time.sleep(0.1)

    def _read_packet(self):
        # 1回のI2Cトランザクションでヘッダー+ペイロードを読む（分割読みによるデータ破損を防止）
        buf = self._raw_read(256)
        length = (buf[1] << 8 | buf[0]) & 0x7FFF
        channel = buf[2]
        if length <= 4 or length > 32762:
            return None, None
        payload = buf[4:min(length, 256)]
        return channel, payload

    def _send_packet(self, channel, data):
        length = len(data) + 4
        packet = [
            length & 0xFF,
            (length >> 8) & 0xFF,
            channel,
            self._sequence[channel],
        ]
        packet.extend(data)
        self._sequence[channel] = (self._sequence[channel] + 1) % 256
        try:
            self._raw_write(packet)
        except Exception as e:
            print(f"BNO085 send error: {e}")

    def _enable_feature(self, report_id, interval_us=20000):
        data = [self._REPORT_SET_FEATURE, report_id, 0, 0, 0]
        data.extend([
            interval_us & 0xFF,
            (interval_us >> 8) & 0xFF,
            (interval_us >> 16) & 0xFF,
            (interval_us >> 24) & 0xFF,
        ])
        data.extend([0] * 8)
        self._send_packet(self._CHANNEL_CONTROL, data)
        time.sleep(0.1)
        for _ in range(5):
            try:
                self._read_packet()
                time.sleep(0.02)
            except Exception:
                pass

    def _parse_sensor_report(self, payload):
        if payload is None or len(payload) < 5:
            return
        i = 0
        while i < len(payload) - 1:
            if payload[i] == 0xFB:
                i += 5
                continue
            report_id = payload[i]
            if report_id == self._REPORT_GAME_ROTATION_VECTOR and i + 12 <= len(payload):
                qi = struct.unpack_from('<h', bytes(payload), i + 4)[0]
                qj = struct.unpack_from('<h', bytes(payload), i + 6)[0]
                qk = struct.unpack_from('<h', bytes(payload), i + 8)[0]
                qw = struct.unpack_from('<h', bytes(payload), i + 10)[0]
                scale = 1.0 / (1 << 14)
                q = (qi * scale, qj * scale, qk * scale, qw * scale)
                norm2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]
                if 0.8 < norm2 < 1.2:
                    self._game_quat = q
                i += 12
            elif report_id == self._REPORT_GYROSCOPE and i + 10 <= len(payload):
                gx = struct.unpack_from('<h', bytes(payload), i + 4)[0]
                gy = struct.unpack_from('<h', bytes(payload), i + 6)[0]
                gz = struct.unpack_from('<h', bytes(payload), i + 8)[0]
                scale = 1.0 / (1 << 9)
                g = (gx * scale, gy * scale, gz * scale)
                # 静止時に±5 rad/s(≈286°/s)を超えることはないので破損データとして棄却
                if all(abs(v) < 5.0 for v in g):
                    self._gyro_raw = g
                i += 10
            elif report_id == self._REPORT_ACCELEROMETER and i + 10 <= len(payload):
                ax = struct.unpack_from('<h', bytes(payload), i + 4)[0]
                ay = struct.unpack_from('<h', bytes(payload), i + 6)[0]
                az = struct.unpack_from('<h', bytes(payload), i + 8)[0]
                scale = 1.0 / (1 << 8)
                a = (ax * scale, ay * scale, az * scale)
                # 加速度ノルムが重力(9.8)から大きく外れる値は破損データとして棄却
                mag2 = a[0]*a[0] + a[1]*a[1] + a[2]*a[2]
                if 70.0 < mag2 < 130.0:   # sqrt: ~8.4 ~ 11.4 m/s²
                    self._accel_raw = a
                i += 10
            elif report_id == self._REPORT_LINEAR_ACCELERATION and i + 10 <= len(payload):
                ax = struct.unpack_from('<h', bytes(payload), i + 4)[0]
                ay = struct.unpack_from('<h', bytes(payload), i + 6)[0]
                az = struct.unpack_from('<h', bytes(payload), i + 8)[0]
                scale = 1.0 / (1 << 8)
                self._linear_accel_raw = (ax * scale, ay * scale, az * scale)
                i += 10
            else:
                i += 1

    def _update_sensor(self):
        try:
            channel, payload = self._read_packet()
            if channel == self._CHANNEL_INPUT_SENSOR and payload:
                self._parse_sensor_report(payload)
                return True
        except OSError:
            pass
        return False

    def _quat_to_yaw(self):
        """現在のクォータニオンからyaw角[°]を計算"""
        qi, qj, qk, qw = self._game_quat
        siny = 2.0 * (qw * qk + qi * qj)
        cosy = 1.0 - 2.0 * (qj * qj + qk * qk)
        return math.degrees(math.atan2(siny, cosy))

    def initialize(self, duration=6.0, discard_samples=50):
        """BNO085用初期化: 加速度バイアス + yawオフセット + ドリフト速度を推定"""
        # 親クラスの加速度バイアス推定を実行
        super().initialize(duration=duration, discard_samples=discard_samples)

        # クォータニオンが安定するまで少し読み出す
        for _ in range(20):
            self.get_raw_data()
            time.sleep(0.02)

        # --- yawドリフト速度を計測 (静止状態で2秒間) ---
        print("  Calibrating yaw drift...")
        calib_duration = 2.0
        calib_start = time.perf_counter()
        self.get_raw_data()
        yaw_start = self._quat_to_yaw()

        while time.perf_counter() - calib_start < calib_duration:
            self.get_raw_data()
            time.sleep(0.02)

        yaw_end = self._quat_to_yaw()
        actual_duration = time.perf_counter() - calib_start

        self._yaw_drift_rate = (yaw_end - yaw_start) / actual_duration
        self._yaw_offset = yaw_end  # 現在のyawをゼロ基準とする
        self._yaw_calib_time = time.perf_counter()
        self.imu_start_time = time.perf_counter()

        print(f"  yaw offset: {self._yaw_offset:.2f}°, drift rate: {self._yaw_drift_rate:.4f}°/s")

    def get_raw_data(self):
        """
        BNO085から加速度および角速度データを取得。
        IMUBaseで継承される measure メソッドが呼び出す。
        """
        for _ in range(2):
            self._update_sensor()
        # acceleration (m/s^2)
        imu_acc = self._accel_raw
        # gyro: rad/s -> deg/s
        imu_gyr = tuple(math.degrees(g) for g in self._gyro_raw)
        return imu_acc, imu_gyr

    def calculate_angle_from_gyro(self, dt):
        """BNO085ではクォータニオンからオイラー角を計算し、yawのオフセットとドリフトを補正"""
        qi, qj, qk, qw = self._game_quat
        # Roll (x)
        sinr = 2.0 * (qw * qi + qj * qk)
        cosr = 1.0 - 2.0 * (qi * qi + qj * qj)
        roll = math.degrees(math.atan2(sinr, cosr))
        # Pitch (y)
        sinp = 2.0 * (qw * qj - qk * qi)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.degrees(math.asin(sinp))
        # Yaw (z): オフセット除去 + ドリフト補正
        raw_yaw = self._quat_to_yaw()
        yaw = raw_yaw - self._yaw_offset
        if self._yaw_calib_time is not None:
            elapsed = time.perf_counter() - self._yaw_calib_time
            yaw -= self._yaw_drift_rate * elapsed
        self.imu_angle = {"x": roll, "y": pitch, "z": yaw}

    def getQuat(self):
        """クォータニオンを取得（BNO055互換: w,x,y,z順）"""
        qi, qj, qk, qw = self._game_quat
        return (qw, qi, qj, qk)

    def close(self):
        self._bus.close()


# ROSモード判定用
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from tf2_ros import TransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    import math

    from tf2_ros import StaticTransformBroadcaster
    ros_available = True

    # ROS2ノードクラス
    class IMUNode(Node):
        def __init__(self):
            super().__init__('imu_node')
            # IMUタイプ自動検出
            imu_type = detect_imu_type(config)
            if imu_type == "BNO085":
                self.imu = BNO085()
            else:
                self.imu = BNO055()
            self.get_logger().info(f"IMU type: {imu_type}")
            self.imu.initialize(duration=6.0)
            # ROSトピックのパブリッシャー
            self.publisher = self.create_publisher(Imu, '/imu/data', 100)
            # Static TF: base_link → imu_link（取り付け位置、固定）
            self.static_tf_broadcaster = StaticTransformBroadcaster(self)
            self._publish_static_tf()
            self.timer = self.create_timer(0.1, self.publish_imu_data)
            self.get_logger().info("IMU Node Initialized")

        def _publish_static_tf(self):
            """IMUの取り付け位置をStaticTFとして発行（姿勢は含めない）"""
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "base_link"
            t.child_frame_id = "imu_link"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.static_tf_broadcaster.sendTransform(t)

        def publish_imu_data(self):
            try:
                # IMUデータを取得
                (
                    imu_acceleration, imu_angular_velocity, imu_angle, _,
                    imu_velocity, imu_position
                ) = self.imu.measure()

                # クォータニオンを取得 (getQuat は w,x,y,z 順)
                qw, qx, qy, qz = self.imu.getQuat()

                # Imuメッセージを作成
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"

                # 加速度データを設定 (m/s^2)
                imu_msg.linear_acceleration.x = imu_acceleration["x"][-1]
                imu_msg.linear_acceleration.y = imu_acceleration["y"][-1]
                imu_msg.linear_acceleration.z = imu_acceleration["z"][-1]

                # 角速度データを設定（deg/s → rad/s に変換）
                imu_msg.angular_velocity.x = math.radians(imu_angular_velocity["x"][-1])
                imu_msg.angular_velocity.y = math.radians(imu_angular_velocity["y"][-1])
                imu_msg.angular_velocity.z = math.radians(imu_angular_velocity["z"][-1])

                # 姿勢データ（クォータニオン）
                imu_msg.orientation.x = qx
                imu_msg.orientation.y = qy
                imu_msg.orientation.z = qz
                imu_msg.orientation.w = qw

                # トピックにパブリッシュ
                self.publisher.publish(imu_msg)

            except Exception as e:
                self.get_logger().error(f"Error while publishing IMU data: {e}")

    def main_ros(args=None):
        """ROSモード実行"""
        rclpy.init(args=args)
        node = IMUNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Shutting down IMU Node")
        finally:
            node.destroy_node()
            rclpy.shutdown()

except ImportError:
    ros_available = False

def _scan_i2c_address(bus_number, address):
    """I2Cバス上に指定アドレスのデバイスが存在するか確認"""
    try:
        bus = smbus.SMBus(bus_number)
        bus.read_byte(address)
        bus.close()
        return True
    except OSError:
        try:
            bus.close()
        except Exception:
            pass
        return False


def detect_imu(bus_number=None):
    """I2Cバスをスキャンし、接続されているIMUセンサーを自動検出して返す"""
    if bus_number is None:
        bus_number = config.I2C_BUS

    # I2Cアドレスとクラスの対応
    sensors = [
        (0x28, "BNO055", BNO055),
        (0x4A, "BNO085", BNO085),
    ]

    print(f"Scanning I2C bus {bus_number} for IMU sensors...")
    for address, name, cls in sensors:
        if _scan_i2c_address(bus_number, address):
            print(f"  Found {name} at 0x{address:02X}")
            return cls()
        else:
            print(f"  0x{address:02X} ({name}): not found")

    print("Error: No IMU sensor detected.")
    exit(1)


def main():
    import csv
    import os
    import time
    from collections import deque

    """スタンドアロンモード"""
    # CSVファイル設定
    csv_file_name = "records/test_imu_measurements.csv"
    os.makedirs(os.path.dirname(csv_file_name), exist_ok=True)
    csv_headers = [
        "time", "acc_x", "acc_y", "acc_z",
        "angle_x", "angle_y", "angle_z",
        "gyro_x", "gyro_y", "gyro_z",
        "vel_x", "vel_y", "vel_z",
        "pos_x", "pos_y", "pos_z"
    ]
    imu = detect_imu()
    imu.initialize(duration=6.0)
    measurement_data = deque(maxlen=100)  # 直近100回分を保存

    # CSVファイルを初期化
    with open(csv_file_name, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(csv_headers)

    try:
        while True:
            # データの測定
            imu_acceleration, imu_angular_velocity, imu_angle, _, imu_velocity, imu_position = imu.measure()

            # 現在時刻を追加
            current_time = time.time()

            # データ整形
            acceleration_values = {axis: list(imu_acceleration[axis])[-1] for axis in "xyz"}
            angular_velocity_values = {axis: round(imu_angular_velocity[axis][-1], 1) for axis in "xyz"}
            angle_values = {axis: round(imu_angle[axis]) for axis in "xyz"}
            velocity_values = {axis: round(imu_velocity[axis], 3) for axis in "xyz"}
            position_values = {axis: round(imu_position[axis], 3) for axis in "xyz"}

            current_measurement = [
                current_time,
                acceleration_values["x"], acceleration_values["y"], acceleration_values["z"],
                angle_values["x"], angle_values["y"], angle_values["z"],
                angular_velocity_values["x"], angular_velocity_values["y"], angular_velocity_values["z"],
                velocity_values["x"], velocity_values["y"], velocity_values["z"],
                position_values["x"], position_values["y"], position_values["z"],
            ]

            # データをdequeに保存
            measurement_data.append(current_measurement)

            # CSVに記録
            with open(csv_file_name, mode="a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(current_measurement)

            # データ表示
            print(
                "加速度[m/s^2]: {0} 角度[°]: {1} 角速度[°/s]: {2} 速度[m/s]: {3} 位置[m]: {4}".format(
                    acceleration_values,
                    angle_values,
                    angular_velocity_values,
                    velocity_values,
                    position_values
                )
            )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print(f"\n計測を終了します。\n{csv_file_name} に記録を保存しました。")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Run IMU as ROS2 node or standalone")
    parser.add_argument('--ros', action='store_true', help="Run as ROS2 node")
    args = parser.parse_args()

    if args.ros and ros_available:
        print("Starting in ROS2 mode...")
        main_ros()
    else:
        print("Starting in standalone mode...")
        main()
