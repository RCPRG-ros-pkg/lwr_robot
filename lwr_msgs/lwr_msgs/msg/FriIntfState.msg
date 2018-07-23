float32 timestamp
uint16 state
uint16 quality
float32 desiredMsrSampleTime
float32 desiredCmdSampleTime
float32 safetyLimits
FriIntfStatistics stat

uint16 FRI_STATE_INVALID = 65535
uint16 FRI_STATE_OFF = 0
uint16 FRI_STATE_MON = 1
uint16 FRI_STATE_CMD = 2

uint16 FRI_QUALITY_INVALID = 65535
uint16 FRI_QUALITY_UNACCEPTABLE = 0
uint16 FRI_QUALITY_BAD = 1
uint16 FRI_QUALITY_OK = 2
uint16 FRI_QUALITY_PERFECT = 3

