;==============================================================================
; centipede_robot/tests/test_gait.asm
; Gait Engine Unit Tests
;==============================================================================
;
; Tests gait pattern generation for correctness.
; Verifies leg phase offsets and timing.
;==============================================================================

%include "config/robot_config.inc"
%include "inc/macros.inc"
%include "inc/structs.inc"

section .text
global test_gait_suite
extern generate_gait_offsets
extern advance_gait_phase
extern gait_set_type
extern debug_print_str
extern debug_print_hex_byte
extern uart_send_byte

;==============================================================================
; Gait Test Suite
; Returns: AL = failure count
;==============================================================================
test_gait_suite:
    push    bx
    push    cx

    xor     bx, bx

    ; Test 1: Wave gait offsets
    call    test_wave_gait
    add     bx, ax

    ; Test 2: Ripple gait offsets
    call    test_ripple_gait
    add     bx, ax

    ; Test 3: Tripod gait offsets
    call    test_tripod_gait
    add     bx, ax

    ; Test 4: Phase advancement
    call    test_phase_advance
    add     bx, ax

    ; Print results
    mov     si, gait_test_msg
    call    debug_print_str
    mov     al, bl
    call    debug_print_hex_byte
    mov     al, 0x0D
    call    uart_send_byte
    mov     al, 0x0A
    call    uart_send_byte

    mov     al, bl
    pop     cx
    pop     bx
    ret

;==============================================================================
; Test Wave Gait
;==============================================================================
test_wave_gait:
    push    bx
    push    cx

    xor     bx, bx

    ; Set wave gait and generate offsets
    mov     al, GAIT_WAVE
    call    gait_set_type
    call    generate_gait_offsets

    ; Verify: First leg should have offset 0
    ; Last leg should have offset near 255 (for 8 segments, 16 legs)
    mov     byte [ACTIVE_SEGMENTS], 8

    call    generate_gait_offsets

    ; Check first leg offset = 0
    mov     al, [gait_offsets_test]
    test    al, al
    jz      .first_ok
    inc     bx
.first_ok:

    ; Check last leg offset = 240 (15/16 * 256)
    mov     al, [gait_offsets_test + 15]
    cmp     al, 240
    je      .last_ok
    inc     bx
.last_ok:

    mov     al, bl
    pop     cx
    pop     bx
    ret

;==============================================================================
; Test Ripple Gait
;==============================================================================
test_ripple_gait:
    push    bx
    push    cx

    xor     bx, bx

    mov     al, GAIT_RIPPLE
    call    gait_set_type
    call    generate_gait_offsets

    ; Verify ripple pattern: each leg should have increasing offset with 90deg shift
    ; Don't check exact values, just ensure they differ
    mov     al, [gait_offsets_test]
    mov     ah, [gait_offsets_test + 1]
    cmp     al, ah
    jne     .differ
    inc     bx
.differ:

    mov     al, bl
    pop     cx
    pop     bx
    ret

;==============================================================================
; Test Tripod Gait
;==============================================================================
test_tripod_gait:
    push    bx

    xor     bx, bx

    mov     al, GAIT_TRIPOD
    call    gait_set_type
    call    generate_gait_offsets

    ; Tripod: legs 0-3 at phase 0, legs 4-7 at phase 128
    mov     al, [gait_offsets_test]
    test    al, al
    jz      .group1_ok
    inc     bx
.group1_ok:

    mov     al, [gait_offsets_test + 4]
    cmp     al, 128
    je      .group2_ok
    inc     bx
.group2_ok:

    mov     al, bl
    pop     bx
    ret

;==============================================================================
; Test Phase Advancement
;==============================================================================
test_phase_advance:
    push    bx
    push    cx

    xor     bx, bx

    ; Set speed and advance multiple times
    mov     byte [gait_speed_test], 10
    call    advance_gait_phase

    ; Phase should now be 10
    mov     al, [gait_phase_test]
    cmp     al, 10
    je      .advance_ok
    inc     bx
.advance_ok:

    mov     al, bl
    pop     cx
    pop     bx
    ret

;==============================================================================
; Data
;==============================================================================
section .data

gait_test_msg: db 'Gait test failures: ', 0x00

; Test data section (overlay of gait_state for testing)
gait_offsets_test:
    times 32 db 0
gait_speed_test:
    db 0
gait_phase_test:
    db 0

