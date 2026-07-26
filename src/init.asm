;==============================================================================
; centipede_robot/src/init.asm
; System Initialization Routines
;==============================================================================

%include "config/robot_config.inc"
%include "inc/macros.inc"
%include "inc/structs.inc"

section .text
global init_system
global init_segments
global init_servos
global init_sensors
global init_communication

extern clear_memory
extern setup_hardware
extern calibrate_servos
extern detect_segments

;==============================================================================
; System Initialization
; Performs:
;   - Hardware detection
;   - Memory initialization
;   - Default configuration loading
;==============================================================================
init_system:
    push    ax
    push    bx
    push    cx
    push    dx

    ; Clear BSS section (zero all global variables)
    call    clear_memory

    ; Setup hardware ports (PIC, PIT, UART, I2C)
    call    setup_hardware

    ; Auto-detect connected segments
    call    detect_segments
    mov     [ACTIVE_SEGMENTS], al

    ; Validate detected segments
    test    al, al
    jz      .no_segments

    ; Initialize segment count
    debug_str segcount_msg
    call    debug_print_hex_byte

    xor     al, al            ; SUCCESS
    jmp     .done

.no_segments:
    debug_str no_seg_msg
    mov     al, ERR_SEGMENT_FAILURE

.done:
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Segment Initialization
; Sets up control blocks for each detected segment
;==============================================================================
init_segments:
    push    ax
    push    bx
    push    cx
    push    di

    xor     cx, cx
    mov     cl, [ACTIVE_SEGMENTS]
    xor     bx, bx            ; Segment index

.seg_loop:
    push    cx

    ; Calculate segment control block address
    mov     di, SEGMENT_TABLE_BASE
    mov     ax, bx
    mov     cx, SEGMENT_CTRL_size
    mul     cx
    add     di, ax

    ; Initialize segment ID
    mov     byte [di + SEGMENT_CTRL.segment_id], bl

    ; Set initial position
    int_to_fixed SEGMENT_CTRL.pos_x, 0
    int_to_fixed SEGMENT_CTRL.pos_y, 0
    int_to_fixed SEGMENT_CTRL.pos_z, SEGMENT_HEIGHT

    ; Set status to idle
    mov     byte [di + SEGMENT_CTRL.status], MODE_IDLE

    ; Initialize leg controllers
    push    di
    call    init_leg_controller
    pop     di

    ; Clear error flags
    mov     byte [di + SEGMENT_CTRL.error_flags], 0

    ; Clear communication buffer
    push    di
    push    cx
    mov     cx, 16
    xor     al, al
    lea     di, [di + SEGMENT_CTRL.comm_buffer]
    rep stosb
    pop     cx
    pop     di

    pop     cx
    inc     bx
    loop    .seg_loop

    debug_str seg_init_msg
    xor     al, al            ; SUCCESS

    pop     di
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Initialize a single leg controller
; IN: di = segment control block address
;==============================================================================
init_leg_controller:
    push    ax
    push    cx
    push    di

    ; Initialize left leg
    lea     di, [di + SEGMENT_CTRL.left_leg]
    mov     cx, LEG_CTRL_size / 2
    xor     ax, ax
    rep stosw

    ; Initialize right leg
    lea     di, [di + SEGMENT_CTRL.right_leg - SEGMENT_CTRL.left_leg]
    mov     cx, LEG_CTRL_size / 2
    xor     ax, ax
    rep stosw

    pop     di
    pop     cx
    pop     ax
    ret

;==============================================================================
; Servo Initialization
; Sets up PWM timers and calibrates servos
;==============================================================================
init_servos:
    push    ax
    push    bx
    push    cx

    ; Configure PWM hardware for servo control
    call    setup_pwm_hardware

    ; Calibrate all servos
    call    calibrate_servos
    test    al, al
    jnz     .failed

    ; Set all servos to neutral position
    mov     cx, TOTAL_SERVOS
    xor     bx, bx

.servo_loop:
    push    cx
    push    bx

    ; Calculate neutral PWM value
    ; Neutral = 1.5ms pulse at 50Hz with 4096 resolution
    ; 1.5ms / 20ms * 4096 = 307
    mov     ax, 307
    call    set_servo_pwm

    pop     bx
    pop     cx
    inc     bx
    loop    .servo_loop

    debug_str servo_init_msg
    xor     al, al
    jmp     .done

.failed:
    debug_str servo_fail_msg
    mov     al, ERR_SERVO_FAILURE

.done:
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Sensor Initialization
; Sets up IMU, distance sensors, and force sensors
;==============================================================================
init_sensors:
    push    ax
    push    bx

    ; Initialize I2C bus for IMU
    call    i2c_init
    test    al, al
    jnz     .failed

    ; Initialize MPU6050 IMU (address 0x68)
    mov     al, 0x68
    call    imu_init
    test    al, al
    jnz     .failed

    ; Initialize ultrasonic distance sensors
    call    ultrasonic_init
    test    al, al
    jnz     .failed

    ; Initialize force sensors on each leg
    call    force_sensor_init
    test    al, al
    jnz     .failed

    debug_str sensor_init_msg
    xor     al, al
    jmp     .done

.failed:
    debug_str sensor_fail_msg
    mov     al, ERR_SENSOR_FAILURE

.done:
    pop     bx
    pop     ax
    ret

;==============================================================================
; Communication Initialization
; Sets up UART for command interface and I2C for segment interconnects
;==============================================================================
init_communication:
    push    ax
    push    bx

    ; Initialize UART for serial debug/command
    call    uart_init
    test    al, al
    jnz     .failed

    ; Initialize I2C for segment-to-segment communication
    call    i2c_init
    test    al, al
    jnz     .failed

    ; Initialize CAN bus if available (future expansion)
    ; call can_init

    debug_str comm_init_msg
    xor     al, al
    jmp     .done

.failed:
    mov     al, ERR_COMM_FAILURE

.done:
    pop     bx
    pop     ax
    ret

;==============================================================================
; Hardware Setup
; Configures PIC, PIT, and other system hardware
;==============================================================================
setup_hardware:
    push    ax

    ; Remap PIC (Programmable Interrupt Controller)
    ; IRQ0-7 -> INT 20h-27h, IRQ8-15 -> INT 28h-2Fh
    mov     al, 0x11          ; ICW1: Edge triggered, cascade, need ICW4
    out     0x20, al          ; Master PIC
    out     0xA0, al          ; Slave PIC

    mov     al, 0x20          ; ICW2: Master base interrupt = 0x20 (32)
    out     0x21, al
    mov     al, 0x28          ; ICW2: Slave base interrupt = 0x28 (40)
    out     0xA1, al

    mov     al, 0x04          ; ICW3: Slave connected to IRQ2
    out     0x21, al
    mov     al, 0x02          ; ICW3: Slave cascade ID = 2
    out     0xA1, al

    mov     al, 0x01          ; ICW4: x86 mode, auto EOI
    out     0x21, al
    mov     al, 0x01
    out     0xA1, al

    ; Mask all interrupts except timer (IRQ0) and keyboard (IRQ1)
    mov     al, 0xFC          ; Enable IRQ0 & IRQ1 only
    out     0x21, al
    mov     al, 0xFF          ; Mask all slave interrupts
    out     0xA1, al

    pop     ax
    ret

;==============================================================================
; Clear Memory / BSS
;==============================================================================
clear_memory:
    push    ax
    push    cx
    push    di
    push    es

    ; Clear from start of BSS to end of stack
    mov     ax, ds
    mov     es, ax
    mov     di, _bss_start
    mov     cx, _bss_end
    sub     cx, di
    shr     cx, 1              ; Word count
    xor     ax, ax
    rep stosw

    pop     es
    pop     di
    pop     cx
    pop     ax
    ret

;==============================================================================
; Segment Detection
; Detects connected segments via I2C bus
; Returns: AL = number of segments found
;==============================================================================
detect_segments:
    push    bx
    push    cx
    push    dx

    xor     bx, bx            ; Segment count
    mov     cl, MAX_SEGMENTS  ; Max segments to check
    mov     dl, 0x30          ; Starting I2C address for segments

.detect_loop:
    push    cx
    push    dx

    ; Try to communicate with segment at I2C address
    call    i2c_probe
    test    al, al
    jnz     .found

    jmp     .next

.found:
    inc     bx
    jmp     .next

.next:
    pop     dx
    pop     cx
    inc     dl
    loop    .detect_loop

    mov     al, bl            ; Return segment count

    pop     dx
    pop     cx
    pop     bx
    ret

;==============================================================================
; PWM Hardware Setup
;==============================================================================
setup_pwm_hardware:
    push    ax
    push    cx

    ; Configure programmable timers for PWM generation
    ; Using Intel 8254 PIT channels 1 and 2
    
    ; Channel 1: Servo PWM base frequency (50 Hz)
    mov     al, 0x76          ; Control word: ch1, lobyte/hibyte, mode 3, binary
    out     0x43, al
    mov     ax, 23863         ; 1193182 / 50 = 23863
    out     0x41, al          ; Low byte
    mov     al, ah
    out     0x41, al          ; High byte

    ; Channel 2: Microsecond timer for pulse width
    mov     al, 0xB6          ; Control word: ch2, lobyte/hibyte, mode 3, binary
    out     0x43, al
    mov     ax, 1193          ; 1193182 / 1000 = 1193 (1kHz)
    out     0x42, al
    mov     al, ah
    out     0x42, al

    pop     cx
    pop     ax
    ret

;==============================================================================
; Data
;==============================================================================
section .data

segcount_msg:    db 'Segments detected: ', 0x00
no_seg_msg:      db 'ERROR: No segments detected!', 0x0D, 0x0A, 0x00
seg_init_msg:    db 'Segment controllers initialized.', 0x0D, 0x0A, 0x00
servo_init_msg:  db 'Servo controllers initialized.', 0x0D, 0x0A, 0x00
servo_fail_msg:  db 'ERROR: Servo calibration failed!', 0x0D, 0x0A, 0x00
sensor_init_msg: db 'Sensor subsystem initialized.', 0x0D, 0x0A, 0x00
sensor_fail_msg: db 'ERROR: Sensor initialization failed!', 0x0D, 0x0A, 0x00
comm_init_msg:   db 'Communication subsystem initialized.', 0x0D, 0x0A, 0x00

;==============================================================================
; Externs from linker
;==============================================================================
extern _bss_start
extern _bss_end

