;==============================================================================
; centipede_robot/config/robot_config.inc
; Robot Configuration Constants
;==============================================================================

;--- Robot Physical Configuration ---
%define MAX_SEGMENTS          16      ; Maximum number of body segments
%define LEGS_PER_SEGMENT      2       ; Legs per segment (left + right)
%define TOTAL_LEGS            (MAX_SEGMENTS * LEGS_PER_SEGMENT)  ; 32

;--- Segment Dimensions (mm, fixed-point 16.16) ---
%define SEGMENT_LENGTH        0x0003E8  ; 1000 mm (1000 * 65536)
%define SEGMENT_WIDTH         0x0001F4  ; 500 mm
%define SEGMENT_HEIGHT        0x0000C8  ; 200 mm
%define LEG_UPPER_LENGTH      0x000190  ; 400 mm
%define LEG_LOWER_LENGTH      0x000190  ; 400 mm

;--- Servo Configuration ---
%define SERVOS_PER_LEG        3       ; Coxa, Femur, Tibia
%define TOTAL_SERVOS          (TOTAL_LEGS * SERVOS_PER_LEG)  ; 96
%define SERVO_PWM_FREQ        50      ; 50 Hz standard servo
%define SERVO_PWM_MIN_MS      1       ; 1 ms = 0 degrees
%define SERVO_PWM_MAX_MS      2       ; 2 ms = 180 degrees
%define SERVO_PWM_RESOLUTION  4096    ; 12-bit PWM resolution
%define SERVO_UPDATE_RATE     20      ; 20 ms update interval (50 Hz)

;--- Timing Constants ---
%define TICK_RATE_HZ          1000    ; Main control loop rate
%define GAIT_CYCLE_MS         1000    ; Full gait cycle time
%define CONTROL_LOOP_HZ       100     ; Control loop frequency
%define SERIAL_BAUD           115200  ; UART baud rate
%define I2C_SPEED             400000  ; I2C clock speed (400 kHz)

;--- Gait Parameters ---
%define DEFAULT_GAIT          GAIT_WAVE
%define DEFAULT_SPEED         0x0001  ; Speed multiplier (0.1 - 1.0)
%define STANCE_PHASE          0x6000  ; Stance phase (60% of cycle)
%define SWING_PHASE           0x4000  ; Swing phase (40% of cycle)
%define STEP_HEIGHT           0x0014  ; 20 mm step height
%define STEP_LENGTH           0x0032  ; 50 mm step length
%define BODY_HEIGHT           0x0032  ; 50 mm body height from ground

;--- Sensor Thresholds ---
%define COLLISION_THRESHOLD   0x0064  ; 100 mm collision detection
%define GROUND_THRESHOLD      0x0050  ; 80 mm ground proximity
%define TILT_THRESHOLD        0x000A  ; 10 degrees tilt limit
%define BATTERY_LOW           0x0A00  ; 10.0V low battery
%define BATTERY_CRITICAL      0x0900  ; 9.0V critical battery

;--- Memory Layout ---
%define STACK_SIZE            0x0400  ; 1KB stack per segment
%define SHARED_MEM_SIZE       0x1000  ; 4KB shared memory
%define COMM_BUFFER_SIZE      0x0200  ; 512 byte communication buffer
%define SENSOR_BUFFER_SIZE    0x0100  ; 256 byte sensor data buffer

;--- Communication Protocol ---
%define COMM_START_BYTE       0xAA    ; Start of frame marker
%define COMM_END_BYTE         0x55    ; End of frame marker
%define COMM_ESCAPE_BYTE      0xDB    ; Escape byte for byte stuffing

;--- Error Codes ---
%define ERR_SUCCESS           0x00
%define ERR_INVALID_PARAM     0x01
%define ERR_COMM_FAILURE      0x02
%define ERR_SERVO_FAILURE     0x03
%define ERR_SENSOR_FAILURE    0x04
%define ERR_TIMEOUT           0x05
%define ERR_OVERCURRENT       0x06
%define ERR_SEGMENT_FAILURE   0x07
%define ERR_SYSTEM_FAULT      0xFF

;--- Operating Modes ---
%define MODE_IDLE             0x00
%define MODE_STAND            0x01
%define MODE_WALK             0x02
%define MODE_TURN             0x03
%define MODE_SIT              0x04
%define MODE_EMERGENCY        0xFF

;--- Gait Types ---
%define GAIT_WAVE             0x00
%define GAIT_RIPPLE           0x01
%define GAIT_TRIPOD           0x02
%define GAIT_CUSTOM           0x03

