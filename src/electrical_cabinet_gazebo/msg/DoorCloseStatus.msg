uint8 STATE_UNKNOWN=0
uint8 STATE_CLOSED=1
uint8 STATE_OPEN=2
uint8 STATE_CLOSING_DOOR=3
uint8 STATE_DOOR_RETURNED=4
uint8 STATE_LATCHED_HANDLE_OUT=5
uint8 STATE_RECOVERING_OPEN=6
uint8 STATE_RECOVERING_HANDLE_OUT=7
uint8 STATE_FORCE_RESETTING=8

uint8 RESULT_NONE=0
uint8 RESULT_ACCEPTED=1
uint8 RESULT_SUCCEEDED=2
uint8 RESULT_FAILED=3
uint8 RESULT_REJECTED=4

uint8 CHECK_NONE=0
uint8 CHECK_DOOR_HINGE=1
uint8 CHECK_HANDLE_TURN=2
uint8 CHECK_HANDLE_POP=4

std_msgs/Header header
uint32 request_id
uint8 state
uint8 last_command
uint8 result
uint8 failed_checks
string detail
float64 door_hinge_position
float64 handle_turn_position
float64 handle_pop_position
