;==============================================================================
; centipede_robot/src/main.asm
; Main Entry Point - Centipede Robot Controller
;==============================================================================
;
; This is the main control program for a multi-segment centipede robot.
; It coordinates up to 16 body segments, each with 2 legs (3 DOF per leg),
; for a total of 96 servo motors.
;
; Architecture: x86 Real Mode (for simplicity / embedded target)
; Assembler: NASM
;==============================================================================

%include "config/robot_config.inc"
%include "inc/macros.inc"
%include "inc/structs.inc"

;==============================================================================
; Code Segment
;==============================================================================
section .text
global _start
extern init_system
extern init_segments
extern init_servos
extern init_sensors
extern init_communication
extern gait_engine_run
extern servo_control_loop
extern sensor_update
extern communication_update
extern state_machine_update
extern debug_main_loop

;==============================================================================
; Entry Point
;==============================================================================
_start:
    ; Initialize stack pointer
    mov     ax, cs
    mov     ds, ax
    mov     es, ax
    mov     ss, ax
    mov     sp, STACK_TOP

    ; Clear direction flag
    cld

    ; Step 1: System Initialization
    call    init_system
    test    al, al
    jnz     init_failed

    ; Step 2: Initialize Communication Bus
    call    init_communication
    test    al, al
    jnz     init_failed

    ; Step 3: Initialize Sensor Subsystem
    call    init_sensors
    test    al, al
    jnz     init_failed

    ; Step 4: Initialize Servo Controllers
    call    init_servos
    test    al, al
    jnz     init_failed

    ; Step 5: Initialize Segment Control Blocks
    call    init_segments
    test    al, al
    jnz     init_failed

    ; Step 6: Configure Timer Interrupts
    call    setup_timers
    test    al, al
    jnz     init_failed

    ; Enable interrupts
    sti

    ; Print startup message
    debug_str startup_msg

    ;==========================================================================
    ; Main Control Loop
    ;==========================================================================
main_loop:
    ; 1. Update Sensors (IMU, distance, force)
    call    sensor_update

    ; 2. Run State Machine (behavior decision)
    call    state_machine_update

    ; 3. Run Gait Engine (generate leg trajectories)
    call    gait_engine_run

    ; 4. Run Servo Control Loop (output PWM)
    call    servo_control_loop

    ; 5. Communication Update (sync segments, receive commands)
    call    communication_update

    ; 6. Debug output (if enabled)
    call    debug_main_loop

    ; Synchronize to control loop rate
    call    sync_control_loop

    ; Check for emergency stop
    call    check_emergency
    test    al, al
    jnz     emergency_stop

    jmp     main_loop

;==============================================================================
; Initialization Failed Handler
;==============================================================================
init_failed:
    debug_str init_fail_msg
    jmp     system_halt

;==============================================================================
; Emergency Stop
;==============================================================================
emergency_stop:
    debug_str emerg_msg
    call    emergency_shutdown

;==============================================================================
; System Halt
;==============================================================================
system_halt:
    cli
    hlt
    jmp     system_halt

;==============================================================================
; Timer Setup
;==============================================================================
setup_timers:
    push    ax
    push    cx

    ; Configure PIT (Programmable Interval Timer) channel 0
    ; Mode 2 (rate generator), for control loop timing
    mov     al, 0x34          ; Control word: ch0, lobyte/hibyte, mode 2, binary
    out     0x43, al

    ; Set divisor for 1kHz (1193182 Hz / 1000 = 1193)
    mov     ax, 1193
    out     0x40, al          ; Low byte
    mov     al, ah
    out     0x40, al          ; High byte

    pop     cx
    pop     ax
    xor     al, al            ; Return SUCCESS
    ret

;==============================================================================
; Control Loop Synchronization
;==============================================================================
sync_control_loop:
    push    ax
    push    cx

    ; Wait for timer tick
    mov     cx, 0xFFFF        ; Timeout counter

.wait_tick:
    cli
    cmp     byte [tick_flag], 1
    sti
    je      .tick_received

    loop    .wait_tick
    ; Timeout occurred
    debug_str timeout_msg
    jmp     .done

.tick_received:
    cli
    mov     byte [tick_flag], 0
    sti

.done:
    pop     cx
    pop     ax
    ret

;==============================================================================
; Emergency Check
;==============================================================================
check_emergency:
    push    bx

    ; Check if emergency button pressed
    mov     bx, [emergency_port]
    in      al, dx
    test    al, 1
    jz      .no_emergency

    ; Emergency detected
    mov     al, 1
    jmp     .done

.no_emergency:
    xor     al, al

.done:
    pop     bx
    ret

;==============================================================================
; Emergency Shutdown
;==============================================================================
emergency_shutdown:
    push    ax
    push    cx

    ; Disable all servos (safe state)
    call    servo_disable_all

    ; Broadcast emergency stop to all segments
    call    comm_broadcast_emergency

    ; Set all segment states to idle
    xor     cx, cx
    mov     cl, [ACTIVE_SEGMENTS]

.shutdown_loop:
    push    cx
    seg_get_ptr cx
    mov     byte [bx + SEGMENT_CTRL.status], MODE_EMERGENCY
    pop     cx
    inc     cx
    cmp     cl, [ACTIVE_SEGMENTS]
    jl      .shutdown_loop

    pop     cx
    pop     ax
    ret

;==============================================================================
; Data Section
;==============================================================================
section .data
align 4

; Startup messages
startup_msg:   db 'Centipede Robot v1.0', 0x0D, 0x0A, 'Initializing...', 0x0D, 0x0A, 0x00
init_fail_msg: db 'FATAL: Initialization failed!', 0x0D, 0x0A, 0x00
emerg_msg:     db 'EMERGENCY STOP ACTIVATED!', 0x0D, 0x0A, 0x00
timeout_msg:   db 'WARNING: Control loop timeout!', 0x0D, 0x0A, 0x00

; System state variables
ACTIVE_SEGMENTS: db 8             ; Default 8 segments
OPERATING_MODE:  db MODE_IDLE     ; Current operating mode
CURRENT_GAIT:    db DEFAULT_GAIT  ; Current gait pattern
SYSTEM_FLAGS:    db 0             ; System flags register
ERROR_STATE:     db ERR_SUCCESS   ; Global error state

; Emergency port address
emergency_port: dw 0x379          ; LPT1 status port (example)

; Timer variables
tick_flag:       db 0             ; Timer tick flag
tick_count:      dd 0             ; Global tick counter
loop_count:      dd 0             ; Control loop counter

;==============================================================================
; BSS Section (Uninitialized Data)
;==============================================================================
section .bss
align 4

; Segment Control Table
SEGMENT_TABLE_BASE:
    resb    SEGMENT_CTRL_size * MAX_SEGMENTS

; Global Gait State
gait_state:
    resb    GAIT_STATE_size

; Sensor Fusion Data
sensor_data:
    resb    SENSOR_FUSION_size

; Communication Buffers
comm_tx_buffer:
    resb    COMM_BUFFER_SIZE
comm_rx_buffer:
    resb    COMM_BUFFER_SIZE

; Shared Memory Pool
shared_memory:
    resb    SHARED_MEM_SIZE

; Stack
STACK_BASE:
    resb    STACK_SIZE
STACK_TOP:

