;==============================================================================
; centipede_robot/src/gait_engine.asm
; Gait Pattern Generation Engine
;==============================================================================
;
; Generates coordinated leg trajectories for various gait patterns.
; Supports wave gait, ripple gait, and alternating tripod gait.
;
; Gait Theory:
;   - Wave Gait: Legs lift sequentially from tail to head, creating a wave
;   - Ripple Gait: Similar to wave but with overlapping phases
;   - Tripod Gait: Three legs in stance, three in swing (alternating)
;
; Leg numbering:  0=Left-front ... 15=Left-rear
;                 16=Right-front ... 31=Right-rear
;==============================================================================

%include "config/robot_config.inc"
%include "inc/macros.inc"
%include "inc/structs.inc"

section .text
global gait_engine_run
global gait_set_type
global gait_set_speed
global gait_set_step_params
global calculate_leg_trajectory
global generate_gait_offsets

extern calculate_ik          ; Inverse kinematics
extern leg_set_position      ; Set leg target position
extern gait_state            ; Global gait state structure

;==============================================================================
; Gait Engine Main Run
; Called each control cycle to update leg trajectories
;==============================================================================
gait_engine_run:
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di

    ; Check if we should be walking
    cmp     byte [OPERATING_MODE], MODE_WALK
    jne     .idle_return

    ; Advance gait phase
    call    advance_gait_phase

    ; Generate gait offsets for current phase
    call    generate_gait_offsets

    ; Calculate trajectory for each leg
    xor     cx, cx
    mov     cl, [ACTIVE_SEGMENTS]

.leg_loop:
    push    cx

    ; Process left leg (index = segment_id * 2)
    mov     ax, cx
    dec     ax
    shl     ax, 1             ; * 2 for leg pair
    mov     bl, al            ; Left leg index

    push    bx
    call    calculate_leg_trajectory
    call    apply_trajectory
    pop     bx

    ; Process right leg
    inc     bl                ; Right leg index

    push    bx
    call    calculate_leg_trajectory
    call    apply_trajectory
    pop     bx

    pop     cx
    loop    .leg_loop

    jmp     .done

.idle_return:
    ; In idle mode, lower all legs to ground
    call    lower_all_legs

.done:
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Advance Gait Phase
; Increments the overall phase based on speed
;==============================================================================
advance_gait_phase:
    push    ax
    push    bx

    mov     bx, gait_state

    ; Get current phase
    mov     al, [bx + GAIT_STATE.gait_phase]

    ; Add speed-scaled increment
    mov     ah, [bx + GAIT_STATE.gait_speed]
    add     al, ah

    ; Store updated phase (wraps naturally due to byte size)
    mov     [bx + GAIT_STATE.gait_phase], al

    pop     bx
    pop     ax
    ret

;==============================================================================
; Generate Gait Offsets
; Calculates phase offsets for each leg based on gait type
;==============================================================================
generate_gait_offsets:
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di

    mov     si, gait_state
    mov     di, gait_state
    add     di, GAIT_STATE.leg_offsets

    ; Get current gait type
    mov     al, [si + GAIT_STATE.current_gait]

    cmp     al, GAIT_WAVE
    je      .wave_gait
    cmp     al, GAIT_RIPPLE
    je      .ripple_gait
    cmp     al, GAIT_TRIPOD
    je      .tripod_gait
    jmp     .done

;--- Wave Gait ---
; Legs lift sequentially from tail to head
; Phase offset(i) = i * (256 / total_legs)
.wave_gait:
    xor     cx, cx
    mov     cl, [ACTIVE_SEGMENTS]
    shl     cl, 1             ; Total legs = segments * 2
    mov     dx, 256
    xor     ax, ax
    div     cl                ; AX = 256 / total_legs
    mov     dx, ax
    xor     bx, bx

.wave_loop:
    mov     [di + bx], bl     ; Phase offset = leg_index * step
    inc     bx
    cmp     bl, cl
    jl      .wave_loop
    jmp     .done

;--- Ripple Gait ---
; Continuous wave with overlapping phases
.ripple_gait:
    xor     cx, cx
    mov     cl, [ACTIVE_SEGMENTS]
    shl     cl, 1
    xor     ax, ax
    mov     al, 128           ; Half-wave offset
    div     cl
    mov     dx, ax            ; Step = 128 / total_legs

    xor     bx, bx

.ripple_loop:
    mov     al, bl
    mul     dl
    add     al, 64            ; Add 90-degree phase shift
    mov     [di + bx], al
    inc     bx
    cmp     bl, cl
    jl      .ripple_loop
    jmp     .done

;--- Alternating Tripod ---
; Three legs stance, three swing - alternating groups
.tripod_gait:
    xor     bx, bx
    xor     cx, cx
    mov     cl, [ACTIVE_SEGMENTS]
    shl     cl, 1

.tripod_loop:
    ; Group 0: legs 0,1,2,3 (front) - phase 0
    ; Group 1: legs 4,5,6,7 (middle) - phase 128
    ; Group 2: legs 8,9,10,11 (rear) - phase 0
    mov     al, bl
    shr     al, 2             ; Divide by 4
    test    al, 1             ; Check if group is odd
    jz      .tripod_zero
    mov     byte [di + bx], 128
    jmp     .tripod_next

.tripod_zero:
    mov     byte [di + bx], 0

.tripod_next:
    inc     bx
    cmp     bl, cl
    jl      .tripod_loop
    jmp     .done

.done:
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Calculate Leg Trajectory
; Computes foot position (X, Y, Z) for a given leg at current phase
; IN:  BL = leg index (0-31)
; OUT: Updates trajectory buffer for this leg
;==============================================================================
calculate_leg_trajectory:
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di

    ; Get phase for this leg
    mov     si, gait_state
    add     si, GAIT_STATE.leg_offsets
    xor     bh, bh
    add     bx, si
    mov     al, [bx]          ; Phase offset for this leg
    add     al, [si - GAIT_STATE.leg_offsets + GAIT_STATE.gait_phase]
    mov     ah, al            ; AH = effective phase

    ; Determine if stance or swing
    cmp     ah, STANCE_PHASE >> 8  ; Compare to stance threshold
    jb      .swing_phase

;--- Stance Phase ---
; Foot moves backward, supporting body weight
.stance_phase:
    ; X: Move foot backward (negative X)
    mov     al, ah
    sub     al, (STANCE_PHASE >> 8)
    ; Scale to step length
    xor     cx, cx
    mov     cl, al
    mov     ax, [si + GAIT_STATE.step_length - GAIT_STATE.leg_offsets]
    mul     cx
    mov     cl, (STANCE_PHASE >> 8)
    div     cl
    neg     ax                ; Backward movement
    mov     word [traj_x], ax

    ; Z: On ground (body height)
    mov     ax, [si + GAIT_STATE.body_height - GAIT_STATE.leg_offsets]
    mov     word [traj_z], ax

    ; Y: Centered (no lateral movement in stance)
    xor     ax, ax
    mov     word [traj_y], ax

    ; Set leg state to stance
    mov     byte [leg_states + bx], 0
    jmp     .done

;--- Swing Phase ---
; Foot lifts and moves forward
.swing_phase:
    ; Normalize phase within swing portion
    mov     al, ah
    xchg    al, (SWING_PHASE >> 8)
    mul     al
    mov     cl, (SWING_PHASE >> 8)
    div     cl                ; AL = normalized phase (0-255)

    ; X: Forward movement
    push    ax
    mov     ax, [si + GAIT_STATE.step_length - GAIT_STATE.leg_offsets]
    mul     al
    mov     cl, 255
    div     cl
    mov     word [traj_x], ax
    pop     ax

    ; Z: Parabolic lift
    push    ax
    ; Lift = step_height * sin(phase * PI)
    ; Approximate using parabola: 4 * h * (p - p^2)
    mov     cl, al
    xor     ax, ax
    mov     al, 255
    sub     al, cl            ; (1 - p)
    mul     cl                ; p * (1-p)
    shl     ax, 2             ; * 4
    mov     cx, ax
    mov     ax, [si + GAIT_STATE.step_height - GAIT_STATE.leg_offsets]
    mul     cx
    mov     cl, 255
    div     cl
    ; Subtract from body height to get lift
    neg     ax
    add     ax, [si + GAIT_STATE.body_height - GAIT_STATE.leg_offsets]
    mov     word [traj_z], ax
    pop     ax

    ; Y: Centered
    xor     ax, ax
    mov     word [traj_y], ax

    ; Set leg state to swing
    mov     byte [leg_states + bx], 1

.done:
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Apply Trajectory
; Converts trajectory buffer to servo angles via IK
; IN:  BL = leg index
;==============================================================================
apply_trajectory:
    push    ax
    push    bx
    push    cx
    push    dx

    ; Load trajectory position
    mov     dx, word [traj_x]
    mov     cx, word [traj_y]
    mov     ax, word [traj_z]

    ; Call inverse kinematics
    call    calculate_ik

    ; Set leg servos to computed angles
    call    leg_set_position

    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Lower All Legs
; Sends all legs to ground position for standing/idle
;==============================================================================
lower_all_legs:
    push    ax
    push    bx
    push    cx

    xor     cx, cx
    mov     cl, [ACTIVE_SEGMENTS]
    shl     cl, 1             ; Total legs

.lower_loop:
    push    cx

    ; Set foot position to neutral
    xor     ax, ax
    mov     word [traj_x], ax
    mov     word [traj_y], ax
    mov     ax, [gait_state + GAIT_STATE.body_height]
    mov     word [traj_z], ax

    call    apply_trajectory

    pop     cx
    inc     bx
    loop    .lower_loop

    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Set Gait Type
; IN:  AL = gait type (GAIT_WAVE, GAIT_RIPPLE, GAIT_TRIPOD)
;==============================================================================
gait_set_type:
    mov     byte [gait_state + GAIT_STATE.current_gait], al
    ret

;==============================================================================
; Set Gait Speed
; IN:  AL = speed (0-255)
;==============================================================================
gait_set_speed:
    mov     byte [gait_state + GAIT_STATE.gait_speed], al
    ret

;==============================================================================
; Set Step Parameters
; IN:  AX = step height, BX = step length
;==============================================================================
gait_set_step_params:
    mov     word [gait_state + GAIT_STATE.step_height], ax
    mov     word [gait_state + GAIT_STATE.step_length], bx
    ret

;==============================================================================
; BSS
;==============================================================================
section .bss

; Trajectory calculation buffer
traj_x: resw 1
traj_y: resw 1
traj_z: resw 1

; Leg state flags
leg_states:
    resb    32

