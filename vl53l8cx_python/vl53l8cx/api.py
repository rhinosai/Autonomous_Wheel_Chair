
VL53L8CX_API_REVISION = "VL53L8CX_2.0.0"

VL53L8CX_DEFAULT_I2C_ADDRESS = 0x52 // 2

VL53L8CX_RESOLUTION_4X4 = 16
VL53L8CX_RESOLUTION_8X8 = 64

VL53L8CX_TARGET_ORDER_CLOSEST = 1
VL53L8CX_TARGET_ORDER_STRONGEST = 2

VL53L8CX_RANGING_MODE_CONTINUOUS = 1
VL53L8CX_RANGING_MODE_AUTONOMOUS = 3

VL53L8CX_POWER_MODE_SLEEP = 0
VL53L8CX_POWER_MODE_WAKEUP = 1
# added by jaeho
VL53L8CX_STATUS_TIMEOUT_ERROR = 1
VL53L8CX_POWER_MODE_DEEP_SLEEP = 2
VL53L8CX_STATUS_LASER_SAFETY = 3
VL53L8CX_STATUS_XTALK_FAILED = 4
VL53L8CX_STATUS_FW_CHECKSUM_FAIL = 5




VL53L8CX_STATUS_OK = 0
VL53L8CX_MCU_ERROR = 66
VL53L8CX_STATUS_INVALID_PARAM = 127
VL53L8CX_STATUS_ERROR = 255

VL53L8CX_NVM_DATA_SIZE = 492
VL53L8CX_CONFIGURATION_SIZE = 972
VL53L8CX_OFFSET_BUFFER_SIZE = 488
VL53L8CX_XTALK_BUFFER_SIZE = 776

VL53L8CX_DCI_ZONE_CONFIG = 0x5450
VL53L8CX_DCI_FREQ_HZ = 0x5458
VL53L8CX_DCI_INT_TIME = 0x545C
VL53L8CX_DCI_FW_NB_TARGET = 0x5478
VL53L8CX_DCI_RANGING_MODE = 0xAD30
VL53L8CX_DCI_DSS_CONFIG = 0xAD38
VL53L8CX_DCI_VHV_CONFIG	= 0xAD60
VL53L8CX_DCI_TARGET_ORDER = 0xAE64
VL53L8CX_DCI_SHARPENER = 0xAED8
VL53L8CX_DCI_INTERNAL_CP = 0xB39C
VL53L8CX_DCI_SYNC_PIN = 0xB5F0
VL53L8CX_DCI_MOTION_DETECTOR_CFG = 0xBFAC
VL53L8CX_DCI_SINGLE_RANGE = 0xD964
VL53L8CX_DCI_OUTPUT_CONFIG = 0xD968
VL53L8CX_DCI_OUTPUT_ENABLES = 0xD970
VL53L8CX_DCI_OUTPUT_LIST = 0xD980
VL53L8CX_DCI_PIPE_CONTROL = 0xDB80

VL53L8CX_UI_CMD_STATUS = 0x2C00
VL53L8CX_UI_CMD_START = 0x2C04
VL53L8CX_UI_CMD_END = 0x2FFF