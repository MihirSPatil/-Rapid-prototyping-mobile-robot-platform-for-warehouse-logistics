Header header

float32 voltage       # [V] Main 48V bus voltage
float32 user_current  # [A] Total power draw from regulated rails
float32 body_temp_adc # [C] Temperature adc reading of enclosure
bool fans_on        # TRUE when fans on

int16 rc_override   # [us] Pulse length of this channel
uint16 rc_velocity  # [us] Pulse length of this channel
uint16 rc_rotation  # [us] Pulse length of this channel

uint8 error
uint8 ERROR_COMMAND_TIMEOUT=1
uint8 ERROR_UNDERVOLT=2
uint8 ERROR_ESTOP=4
uint8 ERROR_ESTOP_RESET=8
uint8 ERROR_BRK_DET=16
