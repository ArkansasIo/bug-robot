;==============================================================================
; centipede_robot/src/sensors.asm
; Sensor Fusion and Processing
;==============================================================================
;
; Manages all robot sensors:
;   - MPU6050 IMU (accelerometer + gyroscope)
;   - HC-SR04 Ultrasonic distance sensors (front/rear)
;   - Force sensing resistors on each leg
;   - Battery voltage monitoring
;   - Temperature sensing
;
; Sensor fusion combines IMU data with complementary filter
; for stable orientation estimation.
;==============================================================================

%include "config/robot_config.inc"
%include "inc/macros.inc"
%include "inc/structs.inc"

section .text
global sensor_update
global sensor_check_obstacle
global sensor_check_tilt
global sensor_check_battery
global sensor_get_terrain_type
global imu_init
global ultrasonic_init
global force_sensor_init
global read_imu
global read_ultrasonic
global read_force_sensor
global read_battery_voltage

extern sensor_data

;==============================================================================
; Sensor Update
; Reads all sensors and fuses data - called each control cycle
;==============================================================================
sensor_update:
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di

    ;--- Read IMU (accelerometer + gyroscope) ---
    call    read_imu

    ;--- Update orientation using complementary filter ---
    call    fuse_imu_data

    ;--- Read distance sensors ---
    call    read_ultrasonic

    ;--- Read force sensors ---
    call    read_force_sensors

    ;--- Read battery voltage ---
    call    read_battery_voltage

    ;--- Read temperature ---
    call    read_temperature

    ;--- Check for collisions ---
    call    check_collisions

    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; IMU Initialization
; Configures MPU6050 sensor via I2C
;==============================================================================
imu_init:
    push    ax
    push    bx
    push    dx

    ; MPU6050 I2C address: 0x68
    mov     al, 0x68

    ; Wake up MPU6050 (clear sleep bit in PWR_MGMT_1 register)
    mov     ah, 0x6B          ; PWR_MGMT_1 register
    mov     dl, 0x00          ; Clear sleep bit
    call    i2c_write_byte

    ; Set gyroscope full scale range to ±250°/s
    mov     ah, 0x1B          ; GYRO_CONFIG register
    mov     dl, 0x00          ; ±250°/s
    call    i2c_write_byte

    ; Set accelerometer full scale range to ±2g
    mov     ah, 0x1C          ; ACCEL_CONFIG register
    mov     dl, 0x00          ; ±2g
    call    i2c_write_byte

    ; Set sample rate divider for 100Hz
    mov     ah, 0x19          ; SMPLRT_DIV register
    mov     dl, 99            ; 1kHz / (1+99) = 10Hz
    call    i2c_write_byte

    ; Configure Digital Low-Pass Filter (DLPF)
    mov     ah, 0x1A          ; CONFIG register
    mov     dl, 0x03          ; DLPF = 44Hz
    call    i2c_write_byte

    debug_str imu_init_msg

    pop     dx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Read IMU
; Reads accelerometer and gyroscope data from MPU6050
;==============================================================================
read_imu:
    push    ax
    push    bx
    push    cx
    push    dx

    ; Read all 14 IMU registers starting at ACCEL_XOUT_H (0x3B)
    mov     al, 0x68          ; I2C address
    mov     ah, 0x3B          ; Starting register
    call    i2c_read_byte     ; ACCEL_XOUT_H
    mov     bh, al
    
    mov     ah, 0x3C
    call    i2c_read_byte     ; ACCEL_XOUT_L
    mov     bl, al
    
    ; Combine high/low bytes (signed 16-bit)
    mov     al, bh
    cbw                       ; Sign extend
    shl     ax, 8
    mov     al, bl
    mov     [sensor_data + SENSOR_FUSION.accel_x], ax

    ; Read Y accelerometer
    mov     ah, 0x3D
    call    i2c_read_byte
    mov     bh, al
    mov     ah, 0x3E
    call    i2c_read_byte
    mov     bl, al
    mov     al, bh
    cbw
    shl     ax, 8
    mov     al, bl
    mov     [sensor_data + SENSOR_FUSION.accel_y], ax

    ; Read Z accelerometer
    mov     ah, 0x3F
    call    i2c_read_byte
    mov     bh, al
    mov     ah, 0x40
    call    i2c_read_byte
    mov     bl, al
    mov     al, bh
    cbw
    shl     ax, 8
    mov     al, bl
    mov     [sensor_data + SENSOR_FUSION.accel_z], ax

    ; Read temperature
    mov     ah, 0x41
    call    i2c_read_byte
    mov     bh, al
    mov     ah, 0x42
    call    i2c_read_byte
    mov     bl, al
    mov     al, bh
    cbw
    shl     ax, 8
    mov     al, bl
    mov     [sensor_data + SENSOR_FUSION.temperature], ax

    ; Read X gyroscope
    mov     ah, 0x43
    call    i2c_read_byte
    mov     bh, al
    mov     ah, 0x44
    call    i2c_read_byte
    mov     bl, al
    mov     al, bh
    cbw
    shl     ax, 8
    mov     al, bl
    mov     [sensor_data + SENSOR_FUSION.gyro_x], ax

    ; Read Y gyroscope
    mov     ah, 0x45
    call    i2c_read_byte
    mov     bh, al
    mov     ah, 0x46
    call    i2c_read_byte
    mov     bl, al
    mov     al, bh
    cbw
    shl     ax, 8
    mov     al, bl
    mov     [sensor_data + SENSOR_FUSION.gyro_y], ax

    ; Read Z gyroscope
    mov     ah, 0x47
    call    i2c_read_byte
    mov     bh, al
    mov     ah, 0x48
    call    i2c_read_byte
    mov     bl, al
    mov     al, bh
    cbw
    shl     ax, 8
    mov     al, bl
    mov     [sensor_data + SENSOR_FUSION.gyro_z], ax

    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Fuse IMU Data
; Complementary filter for orientation estimation
; fusion = alpha * (fusion + gyro * dt) + (1-alpha) * accel_angle
;==============================================================================
fuse_imu_data:
    push    ax
    push    bx
    push    cx
    push    dx

    ; Complementary filter constant (alpha = 0.98)
    ; In fixed 16.16: 0.98 = 0x7D70

    ;--- Roll estimation ---
    ; Accel roll = atan2(accel_y, accel_z)
    mov     ax, [sensor_data + SENSOR_FUSION.accel_y]
    mov     bx, [sensor_data + SENSOR_FUSION.accel_z]
    call    fixed_atan2
    mov     cx, ax            ; CX = accel_roll

    ; Gyro integration
    mov     ax, [sensor_data + SENSOR_FUSION.gyro_x]
    mov     bx, 0x0001        ; dt approximation
    mul     bx
    add     ax, [sensor_data + SENSOR_FUSION.imu_roll]

    ; Complementary filter
    push    ax
    push    cx
    mov     bx, 0x7D70        ; alpha = 0.98
    mul     bx
    pop     cx
    push    dx
    push    ax
    
    mov     ax, cx
    mov     bx, 0x0290        ; 1-alpha = 0x0290 (0.02 * 65536)
    mul     bx
    pop     dx
    add     ax, dx
    pop     dx
    adc     dx, 0
    
    mov     [sensor_data + SENSOR_FUSION.imu_roll], ax

    ;--- Pitch estimation ---
    ; Accel pitch = atan2(-accel_x, sqrt(accel_y^2 + accel_z^2))
    mov     ax, [sensor_data + SENSOR_FUSION.accel_x]
    neg     ax
    push    ax
    mov     ax, [sensor_data + SENSOR_FUSION.accel_y]
    mov     bx, [sensor_data + SENSOR_FUSION.accel_z]
    call    fixed_hypot
    mov     bx, ax
    pop     ax
    call    fixed_atan2
    mov     cx, ax

    ; Gyro integration
    mov     ax, [sensor_data + SENSOR_FUSION.gyro_y]
    mov     bx, 0x0001
    mul     bx
    add     ax, [sensor_data + SENSOR_FUSION.imu_pitch]

    ; Complementary filter
    push    ax
    push    cx
    mov     bx, 0x7D70
    mul     bx
    pop     cx
    push    dx
    push    ax
    
    mov     ax, cx
    mov     bx, 0x0290
    mul     bx
    pop     dx
    add     ax, dx
    pop     dx
    adc     dx, 0
    
    mov     [sensor_data + SENSOR_FUSION.imu_pitch], ax

    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Read Ultrasonic
; Triggers and reads distance from HC-SR04 sensors
;==============================================================================
read_ultrasonic:
    push    ax
    push    bx
    push    cx
    push    dx

    ;--- Front sensor ---
    ; Trigger pulse (10us high on trigger pin)
    call    ultrasonic_trigger_front

    ; Measure echo pulse width (distance = pulse_width / 58 cm)
    call    ultrasonic_measure_echo
    mov     [sensor_data + SENSOR_FUSION.obstacle_front], ax

    ; Convert to mm (cm * 10)
    shl     ax, 3             ; * 8 (approximation)
    mov     [sensor_data + SENSOR_FUSION.obstacle_front], ax

    ;--- Rear sensor ---
    call    ultrasonic_trigger_rear
    call    ultrasonic_measure_echo
    mov     [sensor_data + SENSOR_FUSION.obstacle_rear], ax
    shl     ax, 3
    mov     [sensor_data + SENSOR_FUSION.obstacle_rear], ax

    ;--- Ground sensor (if available) ---
    call    ultrasonic_trigger_ground
    call    ultrasonic_measure_echo
    mov     [sensor_data + SENSOR_FUSION.ground_distance], ax
    shl     ax, 3
    mov     [sensor_data + SENSOR_FUSION.ground_distance], ax

    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Ultrasonic Primitives
;==============================================================================
ultrasonic_init:
    ; Configure trigger pins as outputs, echo pins as inputs
    ret

ultrasonic_trigger_front:
    push    ax
    push    dx
    
    ; 10us trigger pulse
    mov     dx, ultrasonic_port
    mov     al, 1
    out     dx, al
    call    delay_10us
    xor     al, al
    out     dx, al
    
    pop     dx
    pop     ax
    ret

ultrasonic_trigger_rear:
    push    ax
    push    dx
    
    mov     dx, ultrasonic_port + 1
    mov     al, 1
    out     dx, al
    call    delay_10us
    xor     al, al
    out     dx, al
    
    pop     dx
    pop     ax
    ret

ultrasonic_trigger_ground:
    push    ax
    push    dx
    
    mov     dx, ultrasonic_port + 2
    mov     al, 1
    out     dx, al
    call    delay_10us
    xor     al, al
    out     dx, al
    
    pop     dx
    pop     ax
    ret

ultrasonic_measure_echo:
    push    bx
    push    cx
    push    dx

    ; Wait for echo pin high
    mov     dx, echo_port
    xor     cx, cx

.wait_high:
    in      al, dx
    test    al, 1
    jnz     .measure
    loop    .wait_high
    jmp     .timeout

.measure:
    ; Count pulse width
    xor     ax, ax

.count_loop:
    in      al, dx
    test    al, 1
    jz      .done
    inc     ax
    cmp     ax, 0xFFFF
    je      .timeout
    jmp     .count_loop

.timeout:
    xor     ax, ax
    
.done:
    pop     dx
    pop     cx
    pop     bx
    ret

delay_10us:
    push    cx
    mov     cx, 10
.delay:
    loop    .delay
    pop     cx
    ret

;==============================================================================
; Force Sensor Read
; Reads force sensing resistors on each leg
;==============================================================================
read_force_sensors:
    push    ax
    push    bx
    push    cx
    push    di

    xor     cx, cx
    mov     cl, [ACTIVE_SEGMENTS]
    shl     cl, 1             ; Total legs
    xor     bx, bx

.force_loop:
    push    cx
    push    bx

    ; Read ADC for this leg's force sensor
    call    read_adc
    mov     [force_sensor_values + bx], ax

    pop     bx
    pop     cx
    inc     bx
    loop    .force_loop

    pop     di
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Force Sensor Init
;==============================================================================
force_sensor_init:
    ret

;==============================================================================
; Read ADC (analog-to-digital converter)
; Returns: AX = ADC value (0-1023 for 10-bit)
;==============================================================================
read_adc:
    push    dx

    ; Start conversion
    mov     dx, adc_port
    out     dx, al

    ; Wait for conversion complete
    add     dx, 2
.wait_adc:
    in      al, dx
    test    al, 0x80
    jnz     .wait_adc

    ; Read result
    sub     dx, 2
    in      al, dx
    mov     ah, al
    add     dx, 1
    in      al, dx
    mov     al, ah

    pop     dx
    ret

;==============================================================================
; Battery Voltage
;==============================================================================
read_battery_voltage:
    push    ax
    push    dx

    ; Read voltage divider through ADC
    mov     al, 0             ; ADC channel 0 for battery
    call    read_adc

    ; Convert to millivolts
    ; Assuming 10-bit ADC, Vref = 5V, divider ratio = 2
    ; mV = (adc * 5000 * 2) / 1024
    mov     bx, 10000
    mul     bx
    mov     bx, 1024
    div     bx
    mov     [sensor_data + SENSOR_FUSION.voltage], ax

    pop     dx
    pop     ax
    ret

;==============================================================================
; Temperature Read
;==============================================================================
read_temperature:
    push    ax
    push    bx

    ; MPU6050 temperature sensor already read in read_imu
    ; Convert raw value to millicelsius
    ; Temp = (raw_temp / 340) + 36.53  (in celsius)
    ; Temp_mC = (raw_temp * 1000 / 340) + 36530
    mov     ax, [sensor_data + SENSOR_FUSION.temperature]
    mov     bx, 1000
    mul     bx
    mov     bx, 340
    div     bx
    add     ax, 36530         ; 36.53°C offset
    mov     [sensor_data + SENSOR_FUSION.temperature], ax

    pop     bx
    pop     ax
    ret

;==============================================================================
; Sensor Check Functions
;==============================================================================

;--- Check for obstacles ---
sensor_check_obstacle:
    push    bx

    mov     ax, [sensor_data + SENSOR_FUSION.obstacle_front]
    cmp     ax, COLLISION_THRESHOLD
    jb      .no_obstacle

    mov     al, 1
    jmp     .done

.no_obstacle:
    xor     al, al

.done:
    pop     bx
    ret

;--- Check for excessive tilt ---
sensor_check_tilt:
    push    ax
    push    bx

    ; Check roll and pitch angles
    mov     ax, [sensor_data + SENSOR_FUSION.imu_roll]
    abs_word ax
    cmp     ax, TILT_THRESHOLD
    jg      .tilted

    mov     ax, [sensor_data + SENSOR_FUSION.imu_pitch]
    abs_word ax
    cmp     ax, TILT_THRESHOLD
    jg      .tilted

    xor     al, al
    jmp     .done

.tilted:
    mov     al, 1

.done:
    pop     bx
    pop     ax
    ret

;--- Check battery level ---
sensor_check_battery:
    push    ax

    mov     ax, [sensor_data + SENSOR_FUSION.voltage]
    cmp     ax, BATTERY_LOW
    jb      .low_battery

    xor     al, al
    jmp     .done

.low_battery:
    mov     al, 1

.done:
    pop     ax
    ret

;--- Get terrain type ---
sensor_get_terrain_type:
    push    ax
    push    bx

    ; Simple terrain classification based on IMU and distance data
    
    ; Check if slope (pitch > threshold)
    mov     ax, [sensor_data + SENSOR_FUSION.imu_pitch]
    abs_word ax
    cmp     ax, 0x0005        ; 5 degrees
    jg      .slope_terrain

    ; Check if rocky (ground distance varies)
    mov     ax, [sensor_data + SENSOR_FUSION.ground_distance]
    cmp     ax, 0x003C        ; 60mm
    jb      .rocky_terrain

    ; Otherwise flat
    mov     al, TERRAIN_FLAT
    jmp     .done

.slope_terrain:
    mov     al, TERRAIN_SLOPE
    jmp     .done

.rocky_terrain:
    mov     al, TERRAIN_ROCKY

.done:
    pop     bx
    pop     ax
    ret

;==============================================================================
; Collision Detection
;==============================================================================
check_collisions:
    push    ax
    push    bx
    push    cx

    ; Check front obstacle
    mov     ax, [sensor_data + SENSOR_FUSION.obstacle_front]
    cmp     ax, COLLISION_THRESHOLD
    ja      .front_collision

    ; Check rear obstacle
    mov     ax, [sensor_data + SENSOR_FUSION.obstacle_rear]
    cmp     ax, COLLISION_THRESHOLD
    ja      .rear_collision

    ; Check force sensors for unexpected contact
    xor     cx, cx
    mov     cl, [ACTIVE_SEGMENTS]
    shl     cl, 1
    xor     bx, bx

.force_check:
    mov     ax, [force_sensor_values + bx]
    cmp     ax, 0x03E8        ; ~1N force threshold
    jg      .forced_collision
    inc     bx
    loop    .force_check

    ; No collision detected
    mov     byte [collision_active], 0
    jmp     .done

.front_collision:
.rear_collision:
.forced_collision:
    mov     byte [collision_active], 1

.done:
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; External References
;==============================================================================
extern i2c_write_byte
extern i2c_read_byte

;==============================================================================
; Placeholder Math Functions
;==============================================================================
fixed_atan2:
    ret

fixed_hypot:
    ret

;==============================================================================
; Constants
;==============================================================================
TERRAIN_FLAT   equ 0
TERRAIN_ROCKY  equ 1
TERRAIN_SLOPE  equ 2

;==============================================================================
; Data Section
;==============================================================================
section .data

imu_init_msg:   db 'MPU6050 IMU initialized.', 0x0D, 0x0A, 0x00

; Port addresses
ultrasonic_port: dw 0x300      ; Example GPIO port
echo_port:       dw 0x301      ; Example echo input port
adc_port:        dw 0x310      ; Example ADC port

;==============================================================================
; BSS Section
;==============================================================================
section .bss

; Force sensor values (word per leg)
force_sensor_values:
    resw    TOTAL_LEGS

; Collision flag
collision_active: resb 1

; Sensor timing
last_sensor_read: resw 1

