;==============================================================================
; centipede_robot/src/state_machine.asm
; Hierarchical Behavior State Machine
;==============================================================================
;
; Implements the robot's behavior state machine:
;
;               ┌─────────────┐
;               │    IDLE     │
;               └──────┬──────┘
;                      │ cmd: stand
;                      ▼
;               ┌─────────────┐
;               │    STAND    │◄─────────────────────────┐
;               └──────┬──────┘                          │
;                      │ cmd: walk                       │
;                      ▼                                 │
;               ┌─────────────┐   obstacle    ┌──────────┴──┐
;        ┌─────►│    WALK     │──────────────►│   AVOID     │
;        │      └──────┬──────┘               └──────┬──────┘
;        │             │ cmd: turn                   │ clear
;        │             ▼                             │
;        │      ┌─────────────┐                      │
;        │      │    TURN     │──────────────────────┘
;        │      └──────┬──────┘
;        │             │ cmd: sit
;        │             ▼
;        │      ┌─────────────┐
;        └──────┤     SIT     │
;               └─────────────┘
;==============================================================================

%include "config/robot_config.inc"
%include "inc/macros.inc"
%include "inc/structs.inc"

section .text
global state_machine_update
global set_operating_mode
global get_operating_mode
global set_behavior_flags
global check_transition_conditions

extern sensor_data
extern gait_state
extern gait_engine_run
extern gait_set_type

;==============================================================================
; State Machine Update
; Called each control cycle - manages state transitions
;==============================================================================
state_machine_update:
    push    ax
    push    bx
    push    cx
    push    dx

    ; Evaluate transition conditions
    call    check_transition_conditions
    mov     bl, al            ; BL = transition flags

    ; Current state processing
    mov     al, [OPERATING_MODE]

    cmp     al, MODE_IDLE
    je      .state_idle
    cmp     al, MODE_STAND
    je      .state_stand
    cmp     al, MODE_WALK
    je      .state_walk
    cmp     al, MODE_TURN
    je      .state_turn
    cmp     al, MODE_SIT
    je      .state_sit
    cmp     al, MODE_EMERGENCY
    je      .state_emergency
    jmp     .invalid_state

;--- IDLE State ---
.state_idle:
    ; All systems on, but no movement
    ; Expecting "stand" command

    test    bl, TRANSITION_STAND
    jz      .done

    ; Transition to STAND
    mov     byte [OPERATING_MODE], MODE_STAND
    call    on_enter_stand
    jmp     .done

;--- STAND State ---
.state_stand:
    ; Robot is standing on all legs, body at configured height
    ; Check for transitions

    test    bl, TRANSITION_SIT
    jnz     .to_sit

    test    bl, TRANSITION_WALK
    jnz     .to_walk

    test    bl, TRANSITION_EMERGENCY
    jnz     .to_emergency

    ; Stay in stand - maintain position
    call    maintain_stand
    jmp     .done

.to_sit:
    mov     byte [OPERATING_MODE], MODE_SIT
    call    on_enter_sit
    jmp     .done

.to_walk:
    mov     byte [OPERATING_MODE], MODE_WALK
    call    on_enter_walk
    jmp     .done

;--- WALK State ---
.state_walk:
    ; Robot is walking using selected gait
    ; Check for transitions

    test    bl, TRANSITION_STAND
    jnz     .to_stand_from_walk

    test    bl, TRANSITION_TURN
    jnz     .to_turn

    test    bl, TRANSITION_AVOID
    jnz     .to_avoid

    test    bl, TRANSITION_EMERGENCY
    jnz     .to_emergency

    ; Stay in walk - gait engine handles movement
    call    update_walk_parameters
    jmp     .done

.to_stand_from_walk:
    mov     byte [OPERATING_MODE], MODE_STAND
    call    on_enter_stand
    jmp     .done

.to_turn:
    mov     byte [OPERATING_MODE], MODE_TURN
    call    on_enter_turn
    jmp     .done

.to_avoid:
    mov     byte [OPERATING_MODE], MODE_WALK   ; Use walk with avoidance
    mov     byte [AVOIDANCE_ACTIVE], 1
    jmp     .done

;--- TURN State ---
.state_turn:
    ; Robot is turning in place
    test    bl, TRANSITION_WALK
    jnz     .to_walk_from_turn

    test    bl, TRANSITION_EMERGENCY
    jnz     .to_emergency

    call    update_turn
    jmp     .done

.to_walk_from_turn:
    mov     byte [OPERATING_MODE], MODE_WALK
    mov     byte [AVOIDANCE_ACTIVE], 0
    call    on_enter_walk
    jmp     .done

;--- SIT State ---
.state_sit:
    ; Robot is sitting (legs folded)
    test    bl, TRANSITION_STAND
    jnz     .to_stand_from_sit

    test    bl, TRANSITION_EMERGENCY
    jnz     .to_emergency

    jmp     .done

.to_stand_from_sit:
    mov     byte [OPERATING_MODE], MODE_STAND
    call    on_enter_stand
    jmp     .done

;--- EMERGENCY State ---
.state_emergency:
    ; Emergency stop - all systems frozen
    ; Only way out is manual reset
    jmp     .done

.invalid_state:
    ; Unknown state - reset to idle
    mov     byte [OPERATING_MODE], MODE_IDLE

.done:
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Check Transition Conditions
; Evaluates sensor/command inputs for state transitions
; Returns: AL = transition flag bitmask
;==============================================================================
check_transition_conditions:
    push    bx
    push    cx
    push    dx

    xor     bx, bx            ; Clear transition flags

    ;--- Check for external commands (via communication) ---
    call    comm_get_command
    test    al, al
    jz      .check_sensors

    ; Match command to transition
    cmp     al, CMD_STAND
    jne     .check_cmd_walk
    or      bx, TRANSITION_STAND
    jmp     .check_sensors

.check_cmd_walk:
    cmp     al, CMD_WALK
    jne     .check_cmd_TURN
    or      bx, TRANSITION_WALK
    jmp     .check_sensors

.check_cmd_TURN:
    cmp     al, CMD_TURN
    jne     .check_cmd_SIT
    or      bx, TRANSITION_TURN
    jmp     .check_sensors

.check_cmd_SIT:
    cmp     al, CMD_SIT
    jne     .check_cmd_EMERGENCY
    or      bx, TRANSITION_SIT
    jmp     .check_sensors

.check_cmd_EMERGENCY:
    cmp     al, CMD_EMERGENCY
    jne     .check_sensors
    or      bx, TRANSITION_EMERGENCY

;--- Sensor-Based Transitions ---
.check_sensors:
    ; Check obstacle detection
    call    sensor_check_obstacle
    test    al, al
    jz      .check_tilt

    ; Obstacle detected while walking
    cmp     byte [OPERATING_MODE], MODE_WALK
    jne     .check_tilt
    or      bx, TRANSITION_AVOID

.check_tilt:
    ; Check excessive tilt
    call    sensor_check_tilt
    test    al, al
    jz      .check_battery

    ; Excessive tilt - emergency
    or      bx, TRANSITION_EMERGENCY

.check_battery:
    ; Check battery level
    call    sensor_check_battery
    test    al, al
    jz      .check_timeout

    ; Low battery - sit down
    cmp     byte [OPERATING_MODE], MODE_WALK
    je      .low_bat_walk
    cmp     byte [OPERATING_MODE], MODE_STAND
    je      .low_bat_stand
    jmp     .done

.low_bat_walk:
    or      bx, TRANSITION_STAND
    jmp     .done

.low_bat_stand:
    or      bx, TRANSITION_SIT
    jmp     .done

.check_timeout:
    ; No command timeout
    call    check_idle_timeout
    test    al, al
    jz      .done

    ; Timeout - sit down if standing/walking
    cmp     byte [OPERATING_MODE], MODE_STAND
    je      .timeout_sit
    cmp     byte [OPERATING_MODE], MODE_WALK
    je      .timeout_sit
    jmp     .done

.timeout_sit:
    or      bx, TRANSITION_SIT

.done:
    mov     al, bl            ; Return transition flags

    pop     dx
    pop     cx
    pop     bx
    ret

;==============================================================================
; State Entry Actions
;==============================================================================

;--- On Enter STAND ---
on_enter_stand:
    push    ax
    push    bx

    ; Set body to walking height
    mov     word [gait_state + GAIT_STATE.body_height], BODY_HEIGHT

    ; Lower all legs
    call    gait_engine_run   ; Run once to set positions

    pop     bx
    pop     ax
    ret

;--- On Enter WALK ---
on_enter_walk:
    push    ax

    ; Set gait parameters
    mov     al, [CURRENT_GAIT]
    call    gait_set_type

    ; Set default speed
    mov     al, DEFAULT_SPEED
    call    gait_set_speed

    ; Set step parameters
    mov     ax, STEP_HEIGHT
    mov     bx, STEP_LENGTH
    call    gait_set_step_params

    pop     ax
    ret

;--- On Enter TURN ---
on_enter_turn:
    push    ax

    ; Set turning gait
    mov     al, GAIT_TRIPOD
    call    gait_set_type

    pop     ax
    ret

;--- On Enter SIT ---
on_enter_sit:
    push    ax
    push    bx

    ; Lower body and fold legs
    xor     ax, ax
    mov     word [gait_state + GAIT_STATE.body_height], ax

    ; Run gait engine to lower
    call    gait_engine_run

    pop     bx
    pop     ax
    ret

;==============================================================================
; State Stay Actions
;==============================================================================

;--- Maintain Stand ---
maintain_stand:
    push    ax

    ; Keep current position
    ; Gait engine in idle mode handles this
    xor     ax, ax
    call    gait_set_speed

    pop     ax
    ret

;--- Update Walk Parameters ---
update_walk_parameters:
    push    ax

    ; Adjust speed based on terrain
    call    sensor_get_terrain_type
    cmp     al, TERRAIN_FLAT
    je      .flat_terrain
    cmp     al, TERRAIN_ROCKY
    je      .rocky_terrain
    cmp     al, TERRAIN_SLOPE
    je      .slope_terrain
    jmp     .done

.flat_terrain:
    mov     al, DEFAULT_SPEED
    call    gait_set_speed
    jmp     .done

.rocky_terrain:
    mov     al, DEFAULT_SPEED / 2
    call    gait_set_speed

    ; Increase step height
    mov     ax, STEP_HEIGHT + 0x000A  ; +10mm
    mov     bx, STEP_LENGTH / 2
    call    gait_set_step_params
    jmp     .done

.slope_terrain:
    mov     al, DEFAULT_SPEED * 3 / 4
    call    gait_set_speed

    ; Shorter steps
    mov     ax, STEP_HEIGHT
    mov     bx, STEP_LENGTH * 3 / 4
    call    gait_set_step_params
    jmp     .done

.done:
    pop     ax
    ret

;--- Update Turn ---
update_turn:
    push    ax

    ; Apply turning offset to body
    call    calculate_turn_radius

    ; Set differential leg speeds
    call    set_turn_differential

    pop     ax
    ret

;==============================================================================
; Transition Flag Constants
;==============================================================================
TRANSITION_STAND     equ 0x01
TRANSITION_WALK      equ 0x02
TRANSITION_TURN      equ 0x04
TRANSITION_SIT       equ 0x08
TRANSITION_AVOID     equ 0x10
TRANSITION_EMERGENCY equ 0x20

; Command bytes
CMD_STAND     equ 0x10
CMD_WALK      equ 0x11
CMD_TURN      equ 0x12
CMD_SIT       equ 0x13
CMD_EMERGENCY equ 0x14

; Terrain types
TERRAIN_FLAT   equ 0
TERRAIN_ROCKY  equ 1
TERRAIN_SLOPE  equ 2

;==============================================================================
; External References
;==============================================================================
extern OPERATING_MODE
extern CURRENT_GAIT
extern AVOIDANCE_ACTIVE
extern ACTIVE_SEGMENTS
extern comm_get_command
extern sensor_check_obstacle
extern sensor_check_tilt
extern sensor_check_battery
extern sensor_get_terrain_type
extern check_idle_timeout
extern calculate_turn_radius
extern set_turn_differential

;==============================================================================
; BSS
;==============================================================================
section .bss

; State variables
state_timer:        resw    1   ; Time in current state
idle_counter:       resw    1   ; Idle timeout counter
command_counter:    resw    1   ; Command counter
behavior_flags:     resb    1   ; Behavior modification flags
turn_angle:         resd    1   ; Current turn angle target

