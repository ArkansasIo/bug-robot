;==============================================================================
; centipede_robot/inc/macros.inc
; General-purpose assembly macros
;==============================================================================

;--- Register Save/Restore ---
%macro push_all 0
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di
    push    bp
    push    ds
    push    es
%endmacro

%macro pop_all 0
    pop     es
    pop     ds
    pop     bp
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax
%endmacro

;--- Segment Addressing ---
%macro seg_get_ptr 1         ; %1 = segment_number
    mov     bx, %1
    shl     bx, 4           ; Each segment = 16 bytes control block
    add     bx, SEGMENT_TABLE_BASE
%endmacro

%macro seg_get_count 0
    mov     al, [ACTIVE_SEGMENTS]
%endmacro

;--- Safe Memory Access ---
%macro mem_write_byte 2      ; %1 = address, %2 = value
    push    ax
    mov     al, %2
    mov     byte [%1], al
    pop     ax
%endmacro

%macro mem_write_word 2      ; %1 = address, %2 = value
    push    ax
    mov     ax, %2
    mov     word [%1], ax
    pop     ax
%endmacro

%macro mem_read_byte 2       ; %1 = dest register, %2 = address
    mov     %1, byte [%2]
%endmacro

%macro mem_read_word 2       ; %1 = dest register, %2 = address
    mov     %1, word [%2]
%endmacro

;--- Math Helpers ---
%macro clamp_word 3          ; %1 = value, %2 = min, %3 = max
    cmp     %1, %2
    jge     %%check_max
    mov     %1, %2
    jmp     %%done
%%check_max:
    cmp     %1, %3
    jle     %%done
    mov     %1, %3
%%done:
%endmacro

%macro abs_word 1            ; %1 = register to absolute
    cmp     %1, 0
    jge     %%done
    neg     %1
%%done:
%endmacro

%macro min_word 3            ; %1 = dest, %2 = a, %3 = b
    mov     %1, %2
    cmp     %1, %3
    jle     %%done
    mov     %1, %3
%%done:
%endmacro

%macro max_word 3            ; %1 = dest, %2 = a, %3 = b
    mov     %1, %2
    cmp     %1, %3
    jge     %%done
    mov     %1, %3
%%done:
%endmacro

;--- Fixed-Point Helpers ---
%macro fixed_to_int 2        ; %1 = dest (int), %2 = src (fixed 16.16)
    mov     %1, %2
    shr     %1, 16
%endmacro

%macro int_to_fixed 2        ; %1 = dest (fixed), %2 = src (int)
    mov     %1, %2
    shl     %1, 16
%endmacro

;--- Bit Manipulation ---
%macro bit_set 2             ; %1 = address, %2 = bit number
    push    ax
    mov     al, [%1]
    or      al, (1 << %2)
    mov     [%1], al
    pop     ax
%endmacro

%macro bit_clear 2           ; %1 = address, %2 = bit number
    push    ax
    mov     al, [%1]
    and     al, ~(1 << %2)
    mov     [%1], al
    pop     ax
%endmacro

%macro bit_test 3            ; %1 = address, %2 = bit number, %3 = jump label
    test    byte [%1], (1 << %2)
    jnz     %3
%endmacro

;--- Loop Helpers ---
%macro for_loop 4            ; %1 = counter reg, %2 = start, %3 = end, %4 = body label
    mov     %1, %2
%%loop_%4:
    cmp     %1, %3
    jge     %%end_%4
    ; body goes here
    inc     %1
    jmp     %%loop_%4
%%end_%4:
%endmacro

;--- Debug Printing ---
%macro debug_hex 1           ; %1 = register to print
    push_all
    mov     ax, %1
    call    debug_print_hex
    pop_all
%endmacro

%macro debug_str 1           ; %1 = string address
    push_all
    mov     si, %1
    call    debug_print_str
    pop_all
%endmacro

;--- Critical Section ---
%macro enter_critical 0
    cli
%endmacro

%macro exit_critical 0
    sti
%endmacro

;--- Timeout Loop ---
%macro wait_ticks 1          ; %1 = number of ticks
    push    cx
    mov     cx, %1
%%wait:
    call    delay_1ms
    loop    %%wait
    pop     cx
%endmacro

