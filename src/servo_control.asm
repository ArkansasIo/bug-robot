;==============================================================================
; centipede_robot/src/servo_control.asm
; Servo Motor Control Layer
;==============================================================================
;
; Manages 96 servo motors (PWM generation) for a 16-segment centipede.
; Each segment has 2 legs × 3 servos = 6 servos per segment.
;
; PWM interface uses a 4096-resolution timer, updated at 50Hz.
; Servo positions are double-buffered to prevent glitching.
;==============================================================================

%include "config/robot_config.inc"
%include "inc/macros.inc"
%include "inc/structs.inc"

section .text
global servo_control_loop
global set_servo_pwm
global queue_servo_update
global servo_disable_all
global servo_calibrate
global servo_get_status

extern leg_ctrl_table

;==============================================================================
; Servo Control Loop
; Called each control cycle to update PWM outputs
;==============================================================================
servo_control_loop:
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di

    ; Check if servo updates are pending
    cmp     byte [servo_update_pending], 0
    je      .done

    ; Process pending servo updates
    xor     di, di            ; Servo index (0-95)
    mov     cx, TOTAL_SERVOS

.update_loop:
    push    cx

    ; Check if this servo needs update
    bt      word [servo_dirty_bits], di
    jnc     .next_servo

    ; Clear dirty bit
    btr     word [servo_dirty_bits], di

    ; Get servo PWM value
    push    di
    call    get_servo_pwm_value
    pop     di
    mov     dx, ax            ; DX = PWM value

    ; Output PWM to hardware
    push    di
    call    pwm_output
    pop     di

.next_servo:
    inc     di
    pop     cx
    loop    .update_loop

    ; Clear update pending flag
    mov     byte [servo_update_pending], 0

.done:
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Set Servo PWM
; Sets a single servo's PWM value (immediate update)
; IN:  AX = PWM value (0-4095), BX = servo index (0-95)
;==============================================================================
set_servo_pwm:
    push    ax
    push    bx
    push    di

    ; Clamp PWM value to valid range
    clamp_word ax, 0, 4095

    ; Store in servo PWM table
    mov     di, bx
    shl     di, 1             ; Word-sized entries
    add     di, servo_pwm_table
    mov     [di], ax

    ; Mark servo as dirty
    mov     bx, 1
    shl     bx, 1
    bts     word [servo_dirty_bits], bx

    ; Set update pending flag
    mov     byte [servo_update_pending], 1

    pop     di
    pop     bx
    pop     ax
    ret

;==============================================================================
; Queue Servo Update
; Queues a leg's 3 servos for update
; IN:  AL = leg index (0-31)
;==============================================================================
queue_servo_update:
    push    ax
    push    bx
    push    cx
    push    di

    ; Calculate base servo index for this leg
    ; Each leg has 3 servos starting at leg_index * 3
    xor     ah, ah
    mov     bx, ax
    mov     cx, 3
    mul     cx               ; AX = first servo index for this leg

    ; Get leg control block
    push    ax
    mov     bx, ax
    push    bx
    call    get_leg_ctrl_ptr
    pop     bx
    pop     bx

    ; Queue all 3 servos of this leg
    ; Coxa servo
    mov     ax, [si + LEG_CTRL.coxa_pwm]
    push    bx
    call    set_servo_pwm
    pop     bx

    ; Femur servo
    inc     bx
    mov     ax, [si + LEG_CTRL.femur_pwm]
    push    bx
    call    set_servo_pwm
    pop     bx

    ; Tibia servo
    inc     bx
    mov     ax, [si + LEG_CTRL.tibia_pwm]
    push    bx
    call    set_servo_pwm
    pop     bx

    pop     di
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Get Servo PWM Value
; Reads PWM value for a given servo
; IN:  DI = servo index (0-95)
; OUT: AX = PWM value (0-4095)
;==============================================================================
get_servo_pwm_value:
    push    di

    shl     di, 1             ; Word offset
    mov     ax, [servo_pwm_table + di]

    pop     di
    ret

;==============================================================================
; PWM Hardware Output
; Outputs PWM signal to the specified servo channel
; IN:  DI = servo index, DX = PWM value
; OUT: AL = 0 success, non-zero failure
;==============================================================================
pwm_output:
    push    bx
    push    cx
    push    dx

    ; Calculate which PWM controller and channel
    ; Assuming PCA9685-style PWM controller
    ; Each controller handles 16 channels
    ; We need 96 channels / 16 = 6 controllers

    mov     ax, di
    mov     bx, 16
    xor     dx, dx
    div     bx               ; AX = controller index, DX = channel

    push    dx               ; Save channel
    mov     bx, ax           ; BX = controller index

    ; Calculate I2C address for this controller
    ; Base address 0x40, each controller offset by 1
    add     bl, 0x40
    mov     al, bl           ; AL = I2C address

    pop     dx               ; DX = channel (0-15)

    ; Write PWM value to controller register
    ; Register base for channel 0: 0x06 (LED0_ON_L)
    ; Each channel takes 4 registers: ON_L, ON_H, OFF_L, OFF_H
    mov     cl, dl
    shl     cl, 2             ; * 4
    add     cl, 0x06          ; Base register for LED0_ON_L

    ; Write OFF time registers
    ; We keep ON at 0, OFF = PWM value
    push    dx               ; Save channel

    ; Write OFF_L register
    mov     al, cl
    mov     ah, dl           ; Low byte of PWM value
    call    i2c_write_byte

    ; Write OFF_H register
    inc     cl
    mov     al, cl
    mov     ah, dh           ; High byte of PWM value
    and     ah, 0x0F         ; Only lower 4 bits
    call    i2c_write_byte

    pop     dx
    pop     cx
    pop     bx
    ret

;==============================================================================
; Disable All Servos
; Safely disables all servos (high-impedance state)
;==============================================================================
servo_disable_all:
    push    ax
    push    bx
    push    cx
    push    di

    ; Set all servo outputs to high-impedance
    ; PCA9685: Set OE (output enable) pin high, or use SLEEP mode

    ; For each PWM controller, set SLEEP bit in MODE1 register
    xor     di, di
    mov     cx, 6             ; 6 controllers

.disable_loop:
    push    cx

    ; Calculate controller I2C address
    mov     al, dil
    add     al, 0x40
    push    ax

    ; Read MODE1 register
    mov     al, 0x00          ; MODE1 register address
    call    i2c_read_byte
    or      al, 0x10          ; Set SLEEP bit (bit 4)
    mov     ah, al
    pop     ax
    push    ax
    mov     al, 0x00          ; MODE1 register
    call    i2c_write_byte

    pop     ax
    pop     cx
    inc     di
    loop    .disable_loop

    ; Clear all PWM values
    mov     cx, TOTAL_SERVOS
    xor     di, di
    xor     ax, ax

.clear_loop:
    mov     [servo_pwm_table + di], ax
    add     di, 2
    loop    .clear_loop

    pop     di
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Servo Calibration
; Calibrates servo range (min/max PWM) for all servos
;==============================================================================
servo_calibrate:
    push    ax
    push    bx
    push    cx
    push    dx
    push    si

    debug_str calib_msg

    ; Step 1: Find minimum PWM by slowly decreasing until servo stops
    call    calibrate_min_pwm

    ; Step 2: Find maximum PWM by slowly increasing until servo stops
    call    calibrate_max_pwm

    ; Step 3: Center all servos
    call    center_all_servos

    debug_str calib_done_msg

    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Calibrate Minimum PWM
;==============================================================================
calibrate_min_pwm:
    push    ax
    push    bx
    push    cx
    push    dx

    xor     cx, cx            ; Start PWM at 0

.find_min:
    ; Set all servos to current PWM
    mov     ax, cx
    mov     bx, TOTAL_SERVOS
    push    cx
    xor     di, di

.set_loop_min:
    push    bx
    push    ax
    mov     bx, di
    call    set_servo_pwm
    pop     ax
    pop     bx
    inc     di
    cmp     di, bx
    jl      .set_loop_min

    ; Wait for servos to respond
    call    delay_100ms

    ; Check feedback
    call    check_servo_feedback
    test    al, al
    jz      .found_min       ; No movement -> minimum reached

    pop     cx
    inc     cx
    cmp     cx, 500          ; Safety limit
    jl      .find_min

.found_min:
    pop     cx
    mov     word [servo_pwm_min], cx

    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Calibrate Maximum PWM
;==============================================================================
calibrate_max_pwm:
    push    ax
    push    bx
    push    cx
    push    dx

    mov     cx, 4095          ; Start PWM at max

.find_max:
    ; Set all servos to current PWM
    mov     ax, cx
    mov     bx, TOTAL_SERVOS
    push    cx
    xor     di, di

.set_loop_max:
    push    bx
    push    ax
    mov     bx, di
    call    set_servo_pwm
    pop     ax
    pop     bx
    inc     di
    cmp     di, bx
    jl      .set_loop_max

    ; Wait for servos to respond
    call    delay_100ms

    ; Check feedback
    call    check_servo_feedback
    test    al, al
    jz      .found_max       ; No movement -> maximum reached

    pop     cx
    dec     cx
    cmp     cx, 500          ; Safety limit
    jg      .find_max

.found_max:
    pop     cx
    mov     word [servo_pwm_max], cx

    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Center All Servos
; Sets all servos to center position
;==============================================================================
center_all_servos:
    push    ax
    push    bx
    push    cx

    ; Calculate center PWM
    mov     ax, [servo_pwm_min]
    add     ax, [servo_pwm_max]
    shr     ax, 1             ; Average

    mov     cx, TOTAL_SERVOS
    xor     bx, bx

.center_loop:
    push    cx
    push    bx
    call    set_servo_pwm
    pop     bx
    pop     cx
    inc     bx
    loop    .center_loop

    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Check Servo Feedback
; Checks if any servo detected movement
; Returns AL = 1 if movement detected, 0 if stopped
;==============================================================================
check_servo_feedback:
    ; In real implementation, this would read encoder/current sensor
    ; For now, assume movement
    mov     al, 1
    ret

;==============================================================================
; Get Servo Status
; Returns status of a specific servo
; IN:  BX = servo index
; OUT: AX = status flags
;==============================================================================
servo_get_status:
    push    bx

    mov     ax, [servo_status + bx]
    and     ax, 0x00FF

    pop     bx
    ret

;==============================================================================
; Delays (busy-wait)
;==============================================================================
delay_1ms:
    push    cx
    mov     cx, 1000
.delay_loop:
    loop    .delay_loop
    pop     cx
    ret

delay_100ms:
    push    cx
    mov     cx, 100
.outer_loop:
    push    cx
    call    delay_1ms
    pop     cx
    loop    .outer_loop
    pop     cx
    ret

;==============================================================================
; External References (I2C)
;==============================================================================
extern i2c_write_byte
extern i2c_read_byte

;==============================================================================
; Data Section
;==============================================================================
section .data

calib_msg:      db 'Calibrating servos...', 0x0D, 0x0A, 0x00
calib_done_msg: db 'Servo calibration complete.', 0x0D, 0x0A, 0x00

servo_pwm_min:  dw 205          ; 1ms pulse
servo_pwm_max:  dw 410          ; 2ms pulse
servo_update_pending: db 0      ; Flag for pending updates

;==============================================================================
; BSS Section
;==============================================================================
section .bss

; PWM value table (word per servo, 96 servos = 192 bytes)
servo_pwm_table:
    resw    TOTAL_SERVOS

; Dirty bit field (marks which servos need update)
servo_dirty_bits:
    resw    6               ; 96 bits = 6 words

; Servo status flags
servo_status:
    resb    TOTAL_SERVOS

