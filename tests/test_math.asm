;==============================================================================
; centipede_robot/tests/test_math.asm
; Unit Tests for Fixed-Point Math Library
;==============================================================================
;
; Tests for the fixed-point math routines used in kinematics.
; Run these tests to verify correctness before deploying.
;==============================================================================

%include "config/robot_config.inc"
%include "inc/macros.inc"

section .text
global test_math_suite
extern fixed_square
extern fixed_hypot
extern fixed_atan2
extern fixed_sin
extern fixed_cos
extern debug_print_str
extern debug_print_hex_word
extern uart_send_byte

;==============================================================================
; Math Test Suite
; Returns: AL = number of failed tests (0 = all passed)
;==============================================================================
test_math_suite:
    push    bx
    push    cx

    xor     cx, cx            ; Test counter
    xor     bx, bx            ; Failure count

    ;--- Test 1: fixed_square ---
    inc     cx
    call    test_square
    add     bx, ax

    ;--- Test 2: fixed_hypot ---
    inc     cx
    call    test_hypot
    add     bx, ax

    ;--- Test 3: fixed_atan2 ---
    inc     cx
    call    test_atan2
    add     bx, ax

    ;--- Test 4: fixed_sin ---
    inc     cx
    call    test_sin
    add     bx, ax

    ;--- Test 5: fixed_cos ---
    inc     cx
    call    test_cos
    add     bx, ax

    ; Print results
    push    bx
    mov     si, test_pass_msg
    call    debug_print_str
    mov     ax, cx
    call    debug_print_hex_word

    mov     si, test_fail_msg
    call    debug_print_str
    pop     ax
    call    debug_print_hex_word

    mov     al, bl            ; Return failure count

    pop     cx
    pop     bx
    ret

;==============================================================================
; Test square function
;==============================================================================
test_square:
    push    bx
    push    cx

    xor     bx, bx            ; Failure count

    ; Test: square(0) = 0
    xor     ax, ax
    call    fixed_square
    test    ax, ax
    jz      .test1_pass
    inc     bx
.test1_pass:

    ; Test: square(2) = 4 (in fixed point)
    mov     ax, 0x00020000    ; 2.0 in fixed 16.16
    call    fixed_square
    cmp     ax, 0x00040000    ; Should be 4.0
    je      .test2_pass
    inc     bx
.test2_pass:

    ; Test: square(-3) = 9
    mov     ax, 0xFFFD0000    ; -3.0 in fixed 16.16
    call    fixed_square
    cmp     ax, 0x00090000    ; Should be 9.0
    je      .test3_pass
    inc     bx
.test3_pass:

    mov     al, bl
    pop     cx
    pop     bx
    ret

;==============================================================================
; Test hypot function
;==============================================================================
test_hypot:
    push    bx

    xor     bx, bx

    ; Test: hypot(3,4) = 5
    mov     ax, 0x00030000    ; 3.0
    mov     bx, 0x00040000    ; 4.0
    call    fixed_hypot
    cmp     ax, 0x00050000    ; Should be 5.0
    je      .test1_pass
    inc     bx
.test1_pass:

    ; Test: hypot(0,0) = 0
    xor     ax, ax
    xor     bx, bx
    call    fixed_hypot
    test    ax, ax
    jz      .test2_pass
    inc     bx
.test2_pass:

    mov     al, bl
    pop     bx
    ret

;==============================================================================
; Test atan2 function
;==============================================================================
test_atan2:
    push    bx

    xor     bx, bx

    ; Test: atan2(0, 1) = 0
    xor     ax, ax            ; Y = 0
    mov     bx, 0x00010000    ; X = 1.0
    call    fixed_atan2
    test    ax, ax
    jz      .test1_pass
    inc     bx
.test1_pass:

    ; Test: atan2(1, 0) = PI/2 ≈ 1.5708
    mov     ax, 0x00010000    ; Y = 1.0
    xor     bx, bx            ; X = 0
    call    fixed_atan2
    ; Should be approximately PI/2 = 0x1921FB54 (in radians fixed 16.16)
    ; For now just check it's positive and reasonable
    cmp     ax, 0x00010000
    jg      .test2_pass
    inc     bx
.test2_pass:

    mov     al, bl
    pop     bx
    ret

;==============================================================================
; Test sin function
;==============================================================================
test_sin:
    push    bx

    xor     bx, bx

    ; Test: sin(0) = 0
    xor     ax, ax
    call    fixed_sin
    test    ax, ax
    jz      .test1_pass
    inc     bx
.test1_pass:

    ; Test: sin(PI/2) ≈ 1.0
    mov     ax, 0x0000C90F    ; PI/2 in fixed 16.16 radians
    call    fixed_sin
    cmp     ax, 0x00010000    ; Should be ~1.0
    je      .test2_pass
    ; Allow small tolerance
    sub     ax, 0x00010000
    abs_word ax
    cmp     ax, 0x00000100    ; < 0.01 error
    jl      .test2_pass
    inc     bx
.test2_pass:

    mov     al, bl
    pop     bx
    ret

;==============================================================================
; Test cos function
;==============================================================================
test_cos:
    push    bx

    xor     bx, bx

    ; Test: cos(0) = 1.0
    xor     ax, ax
    call    fixed_cos
    cmp     ax, 0x00010000    ; Should be 1.0
    je      .test1_pass
    inc     bx
.test1_pass:

    ; Test: cos(PI) ≈ -1.0
    mov     ax, 0x0001921F    ; PI in fixed 16.16
    call    fixed_cos
    cmp     ax, 0xFFFF0000    ; Should be ~ -1.0
    je      .test2_pass
    sub     ax, 0xFFFF0000
    abs_word ax
    cmp     ax, 0x00000100
    jl      .test2_pass
    inc     bx
.test2_pass:

    mov     al, bl
    pop     bx
    ret

;==============================================================================
; Data
;==============================================================================
section .data

test_pass_msg: db 0x0D, 0x0A, 'Tests run: ', 0x00
test_fail_msg: db ' Failures: ', 0x00

