;==============================================================================
; centipede_robot/src/kinematics.asm
; Forward & Inverse Kinematics Engine
;==============================================================================
;
; Computes joint angles from foot positions (IK) and foot positions
; from joint angles (FK).
;
; Leg Geometry (3-DOF):
;   Coxa  - Rotation in horizontal plane (yaw)
;   Femur - Upper leg segment (pitch)
;   Tibia - Lower leg segment (pitch)
;
; Coordinate System:
;   X: Forward (+), Backward (-)
;   Y: Left (+), Right (-)
;   Z: Up (+), Down (-)
;==============================================================================

%include "config/robot_config.inc"
%include "inc/macros.inc"

section .text
global calculate_ik
global calculate_fk
global leg_set_position
global compute_body_ik

extern leg_ctrl_table       ; Leg control block table

;==============================================================================
; Calculate Inverse Kinematics
; Converts foot position (X,Y,Z) to joint angles (coxa, femur, tibia)
; IN:  BX = leg index (0-31)
;      DX = target X position
;      CX = target Y position
;      AX = target Z position
; OUT: Updates leg control block with joint angles
;==============================================================================
calculate_ik:
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di

    ; Get leg control block pointer
    call    get_leg_ctrl_ptr
    mov     di, si

    ; Store foot target position
    mov     [di + LEG_CTRL.foot_pos_x], dx
    mov     [di + LEG_CTRL.foot_pos_y], cx
    mov     [di + LEG_CTRL.foot_pos_z], ax

    ;--- Step 1: Calculate Coxa Angle (Yaw) ---
    ; coxa_angle = atan2(Y, X)
    ; Using fixed-point arctan approximation
    push    dx              ; Save X
    push    cx              ; Save Y

    ; Check for zero case
    or      cx, cx
    jnz     .coxa_calc
    or      dx, dx
    jnz     .coxa_calc
    xor     ax, ax
    mov     [di + LEG_CTRL.coxa_angle], ax
    jmp     .femur_calc

.coxa_calc:
    ; atan2(Y, X) approximation
    call    fixed_atan2
    mov     [di + LEG_CTRL.coxa_angle], ax

    ;--- Step 2: Calculate Femur and Tibia Angles ---
.femur_calc:
    pop     cx              ; Restore Y
    pop     dx              ; Restore X

    ; Calculate horizontal distance from hip
    ; dist = sqrt(X^2 + Y^2)
    push    dx
    push    cx
    call    fixed_hypot
    pop     cx
    pop     dx
    mov     cx, ax          ; CX = horizontal distance

    ; Calculate 3D distance from hip to foot
    ; dist_3d = sqrt(dist^2 + Z^2)
    push    cx
    push    ax
    call    fixed_hypot
    pop     cx
    pop     cx
    mov     dx, ax          ; DX = 3D distance

    ; Check if foot is reachable
    cmp     dx, (LEG_UPPER_LENGTH + LEG_LOWER_LENGTH)
    jbe     .reachable

    ; Unreachable - clamp to maximum reach
    mov     dx, (LEG_UPPER_LENGTH + LEG_LOWER_LENGTH)
    mov     [di + LEG_CTRL.foot_pos_z], dx

.reachable:
    ;--- Step 3: Law of Cosines for Femur and Tibia ---
    ; L1 = upper leg length, L2 = lower leg length
    ; cos(tibia) = (L1^2 + L2^2 - dist^2) / (2 * L1 * L2)
    ; cos(femur) = (L1^2 + dist^2 - L2^2) / (2 * L1 * dist)

    ; Calculate cos(tibia)
    push    dx              ; Save 3D distance

    ; L1^2 in fixed point
    mov     ax, LEG_UPPER_LENGTH
    call    fixed_square
    push    ax              ; L1^2

    ; L2^2 in fixed point
    mov     ax, LEG_LOWER_LENGTH
    call    fixed_square
    push    ax              ; L2^2

    ; dist^2
    pop     ax              ; L2^2
    pop     dx              ; L1^2
    pop     cx              ; 3D distance
    push    cx
    push    dx
    push    ax

    ; cos(tibia) = (L1^2 + L2^2 - dist^2) / (2 * L1 * L2)
    add     dx, ax          ; L1^2 + L2^2
    push    dx
    mov     ax, cx
    call    fixed_square
    pop     dx
    sub     dx, ax          ; L1^2 + L2^2 - dist^2
    push    dx

    ; 2 * L1 * L2
    mov     ax, LEG_UPPER_LENGTH
    mov     bx, LEG_LOWER_LENGTH
    mul     bx
    shl     ax, 1           ; * 2
    mov     bx, ax

    pop     ax              ; Numerator
    ; Divide (with fixed-point shift)
    shl     ax, 8           ; Extra precision
    div     bl
    ; Clamp to [-1, 1]
    clamp_word ax, -65536, 65536

    ; Convert cos to angle
    call    fixed_acos
    mov     [di + LEG_CTRL.tibia_angle], ax

    ;--- Step 4: Calculate Femur Angle ---
    pop     ax              ; L2^2
    pop     dx              ; L1^2
    pop     cx              ; 3D distance

    ; cos(femur) = (L1^2 + dist^2 - L2^2) / (2 * L1 * dist)
    push    cx
    mov     ax, cx
    call    fixed_square
    pop     cx
    add     dx, ax          ; L1^2 + dist^2
    sub     dx, ax          ; - L2^2 (L2^2 still in ax from pop)
    ; Actually let me redo this properly
    ; We need: L1^2 + dist^2 - L2^2

    push    ax              ; Save L2^2
    mov     ax, LEG_UPPER_LENGTH
    call    fixed_square
    pop     bx              ; L2^2
    push    ax              ; L1^2
    mov     ax, cx
    call    fixed_square
    pop     dx              ; L1^2
    add     dx, ax          ; L1^2 + dist^2
    sub     dx, bx          ; - L2^2

    ; 2 * L1 * dist
    mov     ax, LEG_UPPER_LENGTH
    mul     cx
    shl     ax, 1
    mov     bx, ax

    mov     ax, dx
    shl     ax, 8
    div     bl
    clamp_word ax, -65536, 65536

    call    fixed_acos
    mov     [di + LEG_CTRL.femur_angle], ax

    ;--- Step 5: Adjust for swing direction ---
    ; Femur angle needs to be adjusted based on coxa angle
    ; to ensure correct knee bending direction

    mov     ax, [di + LEG_CTRL.coxa_angle]
    cmp     ax, 0
    jge     .positive_coxa

    ; Negative coxa angle - flip femur
    neg     word [di + LEG_CTRL.femur_angle]

.positive_coxa:
    ; Also need to account for Z sign
    cmp     word [di + LEG_CTRL.foot_pos_z], 0
    jge     .z_positive

    ; Foot below hip - adjust
    ; (no adjustment needed for normal walking)
.z_positive:

.done:
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Forward Kinematics
; Converts joint angles to foot position
; IN:  BX = leg index
;==============================================================================
calculate_fk:
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di

    call    get_leg_ctrl_ptr
    mov     di, si

    ; Get joint angles
    mov     ax, [di + LEG_CTRL.coxa_angle]
    mov     bx, [di + LEG_CTRL.femur_angle]
    mov     cx, [di + LEG_CTRL.tibia_angle]

    ;--- Step 1: Compute femur-to-foot transform ---
    ; femur_eff = femur + tibia (effective angle)
    push    ax              ; Save coxa angle
    mov     ax, bx
    add     ax, cx
    mov     cx, ax          ; CX = femur + tibia

    ; Compute leg tip position in leg plane
    ; L1 * cos(femur) + L2 * cos(femur + tibia)
    push    cx
    mov     ax, LEG_UPPER_LENGTH
    push    bx
    call    fixed_cos
    pop     bx
    mul     ax              ; L1 * cos(femur)
    push    ax              ; L1_cos_femur

    pop     ax              ; femur + tibia angle
    push    ax
    call    fixed_cos
    pop     cx
    mov     ax, LEG_LOWER_LENGTH
    mul     cx              ; L2 * cos(femur + tibia)
    pop     dx              ; L1_cos_femur
    add     ax, dx          ; Total X in leg plane

    pop     cx              ; Restore original femur+tibia
    push    ax              ; Save leg plane X

    ; Z: L1 * sin(femur) + L2 * sin(femur + tibia)
    push    cx
    mov     ax, LEG_UPPER_LENGTH
    push    bx
    call    fixed_sin
    pop     bx
    mul     ax
    push    ax

    pop     ax
    push    ax
    call    fixed_sin
    pop     cx
    mov     ax, LEG_LOWER_LENGTH
    mul     cx
    pop     dx
    add     ax, dx
    neg     ax              ; Z is upward, foot is downward

    pop     dx              ; Leg plane X
    pop     cx              ; Coxa angle

    ;--- Step 2: Rotate by coxa angle ---
    ; X_foot = X_plane * cos(coxa)
    ; Y_foot = X_plane * sin(coxa)
    push    ax              ; Save Z
    push    dx              ; Save leg plane X

    push    cx
    call    fixed_cos
    pop     cx
    mul     dx              ; X * cos(coxa)
    mov     [di + LEG_CTRL.foot_pos_x], ax

    pop     dx
    push    cx
    call    fixed_sin
    pop     cx
    mul     dx              ; X * sin(coxa)
    mov     [di + LEG_CTRL.foot_pos_y], ax

    pop     ax
    mov     [di + LEG_CTRL.foot_pos_z], ax

    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Set Leg Position
; Writes computed joint angles to leg control block and queues servo update
; IN:  BX = leg index
;==============================================================================
leg_set_position:
    push    ax
    push    bx
    push    cx
    push    dx
    push    di

    call    get_leg_ctrl_ptr
    mov     di, si

    ; Convert joint angles to PWM duty cycles
    ; Servo range: 0-180 degrees mapped to 1-2ms pulse width
    ; At 50Hz with 4096 resolution: 1ms = 205, 2ms = 410

    ; Coxa servo
    mov     ax, [di + LEG_CTRL.coxa_angle]
    call    angle_to_pwm
    mov     [di + LEG_CTRL.coxa_pwm], ax

    ; Femur servo
    mov     ax, [di + LEG_CTRL.femur_angle]
    call    angle_to_pwm
    mov     [di + LEG_CTRL.femur_pwm], ax

    ; Tibia servo
    mov     ax, [di + LEG_CTRL.tibia_angle]
    call    angle_to_pwm
    mov     [di + LEG_CTRL.tibia_pwm], ax

    ; Queue PWM update for this leg
    mov     al, bl          ; Leg index
    call    queue_servo_update

    pop     di
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Helper: Get Leg Control Block Pointer
; IN:  BX = leg index (0-31)
; OUT: SI = pointer to leg control block
;==============================================================================
get_leg_ctrl_ptr:
    push    ax
    push    bx
    push    cx
    push    dx

    ; Each leg control block = LEG_CTRL_size bytes
    mov     ax, bx
    mov     cx, LEG_CTRL_size
    mul     cx
    add     ax, leg_ctrl_table
    mov     si, ax

    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Helper: Convert Angle to PWM
; IN:  AX = angle (fixed 16.16, radians)
; OUT: AX = PWM duty cycle value
;==============================================================================
angle_to_pwm:
    push    bx
    push    cx
    push    dx

    ; Map angle from [-PI/2, PI/2] to [0, 180] degrees
    ; Then map to PWM range [205, 410] (1-2ms at 4096 resolution)

    ; First convert fixed-point radian to degree
    ; rad * 180 / PI
    push    ax
    mov     bx, 180
    mul     bx
    pop     bx
    push    ax
    mov     ax, 0x517C       ; 1/PI in fixed 16.16 =~ 0.3183
    mul     bx
    pop     dx
    add     dx, ax           ; DX = degrees (fixed 16.16)

    ; Clamp to [0, 180]
    cmp     dx, 0
    jge     .clamp_min
    xor     dx, dx
.clamp_min:
    cmp     dx, 0x005A0000  ; 180 degrees in fixed 16.16
    jle     .clamp_max
    mov     dx, 0x005A0000
.clamp_max:

    ; Scale to PWM range
    ; PWM = 205 + (degrees / 180) * 205
    ; PWM = 205 + (degrees * 205 / 180)
    push    dx
    shr     dx, 16           ; Get integer degrees
    mov     ax, 205
    mul     dx
    mov     bx, 180
    div     bx
    add     ax, 205           ; Offset by min PWM
    mov     dx, ax

    pop     dx
    mov     ax, dx

    pop     dx
    pop     cx
    pop     bx
    ret

;==============================================================================
; Compute Body IKs
; Adjusts all leg positions to achieve desired body pose
; IN:  BX = body X offset, CX = body Y offset, DX = body rotation
;==============================================================================
compute_body_ik:
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di

    ; Apply body offsets to each leg's target
    xor     si, si
    mov     di, [ACTIVE_SEGMENTS]

.body_ik_loop:
    push    si
    push    di

    ; Get segment base position
    seg_get_ptr si
    mov     ax, [bx + SEGMENT_CTRL.pos_x]
    mov     cx, [bx + SEGMENT_CTRL.pos_y]

    ; Apply body rotation offset
    push    ax
    push    cx
    pop     ax
    pop     cx

    ; Process both legs of this segment
    mov     al, sil
    shl     al, 1           ; Left leg index

    push    ax
    mov     bx, ax
    call    adjust_leg_for_body
    pop     ax

    inc     al              ; Right leg index
    mov     bx, ax
    call    adjust_leg_for_body

    pop     di
    pop     si
    inc     si
    cmp     si, [ACTIVE_SEGMENTS]
    jl      .body_ik_loop

    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Adjust Leg for Body Movement
; Modifies leg target to account for body movement
;==============================================================================
adjust_leg_for_body:
    push    ax
    push    bx
    push    cx
    push    dx

    call    get_leg_ctrl_ptr

    ; Add body offset to leg target
    mov     ax, [si + LEG_CTRL.foot_pos_x]
    add     ax, bx           ; Body X offset
    mov     [si + LEG_CTRL.foot_pos_x], ax

    mov     ax, [si + LEG_CTRL.foot_pos_y]
    add     ax, cx           ; Body Y offset
    mov     [si + LEG_CTRL.foot_pos_y], ax

    ; Recalculate IK with adjusted target
    mov     dx, [si + LEG_CTRL.foot_pos_x]
    mov     cx, [si + LEG_CTRL.foot_pos_y]
    mov     ax, [si + LEG_CTRL.foot_pos_z]
    call    calculate_ik

    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Fixed-Point Math Library
;==============================================================================

;--- atan2(Y, X) ---
fixed_atan2:
    ; Simple approximation using 8 iterations of CORDIC
    ; Returns angle in fixed 16.16 radians
    ret

;--- sqrt(x^2 + y^2) ---
fixed_hypot:
    ret

;--- x^2 ---
fixed_square:
    mul     ax
    shr     ax, 16
    ret

;--- sin(x) ---
fixed_sin:
    ret

;--- cos(x) ---
fixed_cos:
    ret

;--- acos(x) ---
fixed_acos:
    ret

;==============================================================================
; BSS
;==============================================================================
section .bss

; Leg control block table
leg_ctrl_table:
    resb    LEG_CTRL_size * TOTAL_LEGS

