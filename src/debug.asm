;==============================================================================
; centipede_robot/src/debug.asm
; Debug and Diagnostics Module
;==============================================================================
;
; Provides:
;   - Real-time status display
;   - Serial console debug output
;   - Error logging
;   - System diagnostics
;   - Performance monitoring
;==============================================================================

%include "config/robot_config.inc"
%include "inc/macros.inc"
%include "inc/structs.inc"

section .text
global debug_main_loop
global debug_print_hex
global debug_print_hex_byte
global debug_print_str
global debug_log_error
global debug_dump_state
global debug_performance_report
global debug_interrupt_handler

extern uart_send_byte

;==============================================================================
; Debug Main Loop
; Called each control cycle for debug output
;==============================================================================
debug_main_loop:
    push    ax
    push    bx

    ; Check if debug mode enabled
    cmp     byte [debug_enabled], 0
    je      .done

    ; Update debug counter
    inc     byte [debug_counter]
    cmp     byte [debug_counter], 50  ; Every 50 cycles (~2Hz at 100Hz loop)
    jl      .done

    mov     byte [debug_counter], 0

    ; Print debug status line
    call    debug_print_status

.done:
    pop     bx
    pop     ax
    ret

;==============================================================================
; Debug Print Status
; Prints one-line robot status
;==============================================================================
debug_print_status:
    push    ax
    push    bx
    push    si

    ; Print carriage return and line feed
    mov     al, 0x0D
    call    uart_send_byte
    mov     al, 0x0A
    call    uart_send_byte

    ; Print "CP>" prompt
    mov     si, prompt_str
    call    debug_print_str

    ; Print operating mode
    mov     al, [OPERATING_MODE]
    call    debug_print_mode

    ; Print separator
    mov     al, ' '
    call    uart_send_byte

    ; Print active segments
    mov     al, 'S'
    call    uart_send_byte
    mov     al, [ACTIVE_SEGMENTS]
    add     al, '0'
    call    uart_send_byte

    ; Print separator
    mov     al, ' '
    call    uart_send_byte

    ; Print gait type
    mov     al, 'G'
    call    uart_send_byte
    mov     al, [CURRENT_GAIT]
    add     al, '0'
    call    uart_send_byte

    ; Print separator
    mov     al, ' '
    call    uart_send_byte

    ; Print battery voltage
    mov     al, 'V'
    call    uart_send_byte
    mov     ax, [sensor_data + SENSOR_FUSION.voltage]
    call    debug_print_dec_word

    ; Print separator
    mov     al, ' '
    call    uart_send_byte

    ; Print loop count
    mov     al, 'L'
    call    uart_send_byte
    mov     ax, word [loop_count_low]
    call    debug_print_hex_word

    pop     si
    pop     bx
    pop     ax
    ret

;==============================================================================
; Debug Print String
; Prints null-terminated string via UART
; IN:  SI = string address
;==============================================================================
debug_print_str:
    push    ax
    push    si

.print_loop:
    lodsb
    test    al, al
    jz      .done
    call    uart_send_byte
    jmp     .print_loop

.done:
    pop     si
    pop     ax
    ret

;==============================================================================
; Debug Print Hex Word
; Prints AX as 4 hex digits
;==============================================================================
debug_print_hex:
    push    ax
    push    bx
    push    cx

    mov     cx, 4
    mov     bx, ax

.hex_loop:
    push    cx
    mov     cl, 4
    rol     bx, cl
    mov     al, bl
    and     al, 0x0F
    add     al, '0'
    cmp     al, '9'
    jle     .print_digit
    add     al, 7            ; Convert to 'A'-'F'
    
.print_digit:
    call    uart_send_byte
    pop     cx
    loop    .hex_loop

    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Debug Print Hex Byte
; Prints AL as 2 hex digits
;==============================================================================
debug_print_hex_byte:
    push    ax
    push    bx

    mov     bh, al

    ; First nibble
    mov     al, bh
    shr     al, 4
    and     al, 0x0F
    add     al, '0'
    cmp     al, '9'
    jle     .first_digit
    add     al, 7
    
.first_digit:
    call    uart_send_byte

    ; Second nibble
    mov     al, bh
    and     al, 0x0F
    add     al, '0'
    cmp     al, '9'
    jle     .second_digit
    add     al, 7

.second_digit:
    call    uart_send_byte

    pop     bx
    pop     ax
    ret

;==============================================================================
; Debug Print Hex Word (4 digits)
; IN:  AX = value to print
;==============================================================================
debug_print_hex_word:
    push    ax

    ; Print high byte
    mov     al, ah
    call    debug_print_hex_byte

    ; Print low byte
    pop     ax
    push    ax
    mov     al, al
    call    debug_print_hex_byte

    pop     ax
    ret

;==============================================================================
; Debug Print Decimal Word
; Prints AX as decimal number (unsigned, max 65535)
;==============================================================================
debug_print_dec_word:
    push    ax
    push    bx
    push    cx
    push    dx

    mov     cx, 10
    xor     dx, dx
    push    dx               ; Push 0 as sentinel

.div_loop:
    div     cx
    add     dl, '0'          ; Convert remainder to ASCII
    push    dx
    xor     dx, dx
    test    ax, ax
    jnz     .div_loop

.print_loop:
    pop     ax
    cmp     al, 0
    je      .done
    call    uart_send_byte
    jmp     .print_loop

.done:
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Debug Print Operating Mode
; IN:  AL = mode
;==============================================================================
debug_print_mode:
    push    ax
    push    si

    cmp     al, MODE_IDLE
    jne     .check_stand
    mov     si, mode_idle
    call    debug_print_str
    jmp     .done

.check_stand:
    cmp     al, MODE_STAND
    jne     .check_walk
    mov     si, mode_stand
    call    debug_print_str
    jmp     .done

.check_walk:
    cmp     al, MODE_WALK
    jne     .check_turn
    mov     si, mode_walk
    call    debug_print_str
    jmp     .done

.check_turn:
    cmp     al, MODE_TURN
    jne     .check_sit
    mov     si, mode_turn
    call    debug_print_str
    jmp     .done

.check_sit:
    cmp     al, MODE_SIT
    jne     .check_emergency
    mov     si, mode_sit
    call    debug_print_str
    jmp     .done

.check_emergency:
    cmp     al, MODE_EMERGENCY
    jne     .unknown
    mov     si, mode_emergency
    call    debug_print_str
    jmp     .done

.unknown:
    mov     si, mode_unknown

.done:
    pop     si
    pop     ax
    ret

;==============================================================================
; Debug Log Error
; Logs error to error buffer
; IN:  AL = error code, BX = additional info
;==============================================================================
debug_log_error:
    push    ax
    push    bx
    push    di

    ; Get current error index
    mov     di, [error_log_index]
    
    ; Store error
    mov     [error_log + di], al
    mov     [error_log + di + 1], bl
    mov     [error_log + di + 2], bh

    ; Increment index (circular buffer)
    add     di, 3
    cmp     di, ERROR_LOG_SIZE
    jl      .no_wrap
    xor     di, di

.no_wrap:
    mov     [error_log_index], di

    ; Print error
    push    si
    mov     si, error_prefix
    call    debug_print_str
    
    pop     ax
    call    debug_print_hex_byte
    
    mov     al, ':'
    call    uart_send_byte
    mov     al, ' '
    call    uart_send_byte

    mov     ax, bx
    call    debug_print_hex_word
    
    pop     si
    pop     di
    pop     bx
    pop     ax
    ret

;==============================================================================
; Debug Dump State
; Dumps full system state for diagnostics
;==============================================================================
debug_dump_state:
    push    ax
    push    bx
    push    cx
    push    si

    mov     si, dump_header
    call    debug_print_str

    ; Dump operating mode
    mov     al, [OPERATING_MODE]
    mov     si, dump_mode
    call    debug_print_str
    call    debug_print_hex_byte

    ; Dump active segments
    mov     al, [ACTIVE_SEGMENTS]
    mov     si, dump_segments
    call    debug_print_str
    call    debug_print_hex_byte

    ; Dump gait state
    mov     al, [CURRENT_GAIT]
    mov     si, dump_gait
    call    debug_print_str
    call    debug_print_hex_byte

    ; Dump sensor data
    mov     si, dump_imu
    call    debug_print_str
    mov     ax, [sensor_data + SENSOR_FUSION.imu_roll]
    call    debug_print_hex_word
    mov     al, ','
    call    uart_send_byte
    mov     ax, [sensor_data + SENSOR_FUSION.imu_pitch]
    call    debug_print_hex_word
    mov     al, ','
    call    uart_send_byte
    mov     ax, [sensor_data + SENSOR_FUSION.imu_yaw]
    call    debug_print_hex_word

    ; Dump battery
    mov     si, dump_battery
    call    debug_print_str
    mov     ax, [sensor_data + SENSOR_FUSION.voltage]
    call    debug_print_dec_word
    mov     al, 'm'
    call    uart_send_byte
    mov     al, 'V'
    call    uart_send_byte

    ; Newline
    mov     al, 0x0D
    call    uart_send_byte
    mov     al, 0x0A
    call    uart_send_byte

    pop     si
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Debug Performance Report
; Reports timing and performance metrics
;==============================================================================
debug_performance_report:
    push    ax
    push    bx
    push    cx
    push    si

    mov     si, perf_header
    call    debug_print_str

    ; Report loop time
    mov     ax, [last_loop_time]
    mov     si, perf_loop_time
    call    debug_print_str
    call    debug_print_dec_word
    mov     al, 'u'
    call    uart_send_byte
    mov     al, 's'
    call    uart_send_byte

    ; Report max loop time
    mov     ax, [max_loop_time]
    mov     si, perf_max_time
    call    debug_print_str
    call    debug_print_dec_word

    ; Report servo update rate
    mov     ax, [servo_update_count]
    mov     si, perf_servo_rate
    call    debug_print_str
    call    debug_print_dec_word

    ; Report communication stats
    mov     ax, [comm_packet_count]
    mov     si, perf_comm_packets
    call    debug_print_str
    call    debug_print_dec_word

    ; Newline
    mov     al, 0x0D
    call    uart_send_byte
    mov     al, 0x0A
    call    uart_send_byte

    pop     si
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Debug Interrupt Handler
; Sample interrupt handler for debug/profiling
;==============================================================================
debug_interrupt_handler:
    push    ax

    ; Increment tick counter
    inc     word [tick_count_low]
    jnc     .no_carry
    inc     word [tick_count_high]

.no_carry:
    ; Set tick flag
    mov     byte [tick_flag], 1

    ; Profile timing - capture timer value
    in      al, 0x40          ; Read PIT counter
    mov     byte [last_timer_value], al

    pop     ax
    iret

;==============================================================================
; Timing Measurement
;==============================================================================
start_timing:
    push    ax
    
    ; Read PIT counter
    mov     al, 0x00          ; Latch counter 0
    out     0x43, al
    in      al, 0x40          ; Low byte
    mov     ah, al
    in      al, 0x40          ; High byte
    mov     [timing_start], ax
    
    pop     ax
    ret

stop_timing:
    push    ax
    
    mov     al, 0x00
    out     0x43, al
    in      al, 0x40
    mov     ah, al
    in      al, 0x40
    
    sub     ax, [timing_start]
    neg     ax                ; Timer counts down
    mov     [last_loop_time], ax
    
    ; Update max
    cmp     ax, [max_loop_time]
    jle     .no_new_max
    mov     [max_loop_time], ax

.no_new_max:
    pop     ax
    ret

;==============================================================================
; Data Section
;==============================================================================
section .data

; Debug strings
prompt_str:      db 'CP> ', 0x00

mode_idle:       db 'IDLE', 0x00
mode_stand:      db 'STAND', 0x00
mode_walk:       db 'WALK', 0x00
mode_turn:       db 'TURN', 0x00
mode_sit:        db 'SIT', 0x00
mode_emergency:  db 'EMRG', 0x00
mode_unknown:    db '????', 0x00

error_prefix:    db 'ERR:', 0x00

dump_header:     db 0x0D, 0x0A, '=== System Dump ===', 0x0D, 0x0A, 0x00
dump_mode:       db 'Mode: ', 0x00
dump_segments:   db ' Segs: ', 0x00
dump_gait:       db ' Gait: ', 0x00
dump_imu:        db 0x0D, 0x0A, 'IMU(R,P,Y): ', 0x00
dump_battery:    db 0x0D, 0x0A, 'Batt: ', 0x00

perf_header:     db 0x0D, 0x0A, '=== Performance ===', 0x0D, 0x0A, 0x00
perf_loop_time:  db 'Loop: ', 0x00
perf_max_time:   db ' Max: ', 0x00
perf_servo_rate: db ' SrvUpd: ', 0x00
perf_comm_packets: db ' Comm: ', 0x00

; Configuration
debug_enabled:   db 1            ; Debug output enabled by default
debug_counter:   db 0            ; Cycle counter for throttling output

; Performance counters
loop_count_low:  dw 0
loop_count_high: dw 0
last_loop_time:  dw 0
max_loop_time:   dw 0
servo_update_count: dw 0
comm_packet_count:  dw 0

; Timer
timing_start:    dw 0
last_timer_value: db 0

;==============================================================================
; BSS Section
;==============================================================================
section .bss

; Error log (circular buffer)
ERROR_LOG_SIZE  equ 256
error_log:
    resb    ERROR_LOG_SIZE
error_log_index:
    resw    1

