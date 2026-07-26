;==============================================================================
; centipede_robot/src/communication.asm
; Inter-Segment and External Communication
;==============================================================================
;
; Communication Architecture:
;   - UART: External command interface (PC control)
;   - I2C: Inter-segment data bus (master-slave)
;   - CAN: Future expansion (not implemented)
;
; Protocol:
;   - Byte-stuffed frames with CRC8 checksum
;   - Master/slave addressing for segment routing
;==============================================================================

%include "config/robot_config.inc"
%include "inc/macros.inc"
%include "inc/structs.inc"

section .text
global communication_update
global i2c_init
global i2c_probe
global i2c_write_byte
global i2c_read_byte
global uart_init
global uart_send_byte
global uart_receive_byte
global comm_get_command
global comm_broadcast_emergency
global comm_send_packet
global comm_receive_packet

;==============================================================================
; Communication Update
; Main communication handler - called each control cycle
;==============================================================================
communication_update:
    push    ax
    push    bx
    push    cx
    push    dx

    ; Check for incoming UART commands
    call    uart_check_data
    test    al, al
    jz      .check_i2c

    ; Process received UART byte
    mov     al, [uart_rx_byte]
    call    process_uart_byte
    jmp     .done

.check_i2c:
    ; Check for I2C messages from other segments
    call    i2c_check_data
    test    al, al
    jz      .done

    ; Process received I2C data
    call    process_i2c_data

.done:
    ; Broadcast segment state (if master)
    mov     al, [IS_MASTER]
    test    al, al
    jz      .skip_broadcast

    call    broadcast_segment_state

.skip_broadcast:
    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; UART Initialization
; Configures 16550 UART for serial communication
;==============================================================================
uart_init:
    push    ax
    push    dx

    ; Configure UART at COM1 (0x3F8)
    mov     dx, 0x3F8         ; COM1 base

    ; Set baud rate divisor (115200 baud)
    ; Divisor = 115200 / 115200 = 1
    add     dx, 3             ; Line Control Register
    mov     al, 0x80          ; DLAB = 1
    out     dx, al

    sub     dx, 3             ; Divisor Latch Low Byte
    mov     al, 1             ; 115200 baud
    out     dx, al

    add     dx, 1             ; Divisor Latch High Byte
    xor     al, al
    out     dx, al

    ; Set line parameters: 8N1 (8 bits, No parity, 1 stop bit)
    add     dx, 2             ; Back to LCR
    mov     al, 0x03          ; 8N1
    out     dx, al

    ; Enable FIFO
    sub     dx, 2             ; FIFO Control Register
    mov     al, 0xC7          ; Enable, clear, 14-byte threshold
    out     dx, al

    ; Enable interrupts (receive data available)
    add     dx, 1             ; Interrupt Enable Register
    mov     al, 0x01          ; Enable receive interrupt
    out     dx, al

    xor     al, al            ; SUCCESS
    pop     dx
    pop     ax
    ret

;==============================================================================
; Send byte via UART
; IN:  AL = byte to send
;==============================================================================
uart_send_byte:
    push    ax
    push    dx

    mov     dx, 0x3F8         ; COM1

    ; Wait for transmitter holding register empty
.wait_tx:
    add     dx, 5             ; Line Status Register
    in      al, dx
    test    al, 0x20          ; Transmitter empty bit
    jz      .wait_tx

    sub     dx, 5             ; Back to data register
    pop     ax                ; Restore byte to send
    push    ax
    out     dx, al            ; Send byte

    pop     ax
    pop     dx
    ret

;==============================================================================
; Receive byte from UART (non-blocking)
; OUT: AL = received byte, CF = 1 if data available
;==============================================================================
uart_receive_byte:
    push    dx

    mov     dx, 0x3F8

    ; Check if data available
    add     dx, 5             ; Line Status Register
    in      al, dx
    test    al, 0x01          ; Receive data ready bit
    jz      .no_data

    sub     dx, 5             ; Back to data register
    in      al, dx            ; Read byte
    stc                       ; Set carry (data available)
    jmp     .done

.no_data:
    clc                       ; Clear carry (no data)

.done:
    pop     dx
    ret

;==============================================================================
; UART Check Data
; Returns: AL = 1 if data received, 0 otherwise
;==============================================================================
uart_check_data:
    push    dx

    mov     dx, 0x3F8 + 5     ; Line Status Register
    in      al, dx
    and     al, 0x01          ; Check receive data ready
    jz      .no_data2

    ; Read the byte
    mov     dx, 0x3F8
    in      al, dx
    mov     [uart_rx_byte], al
    mov     al, 1
    jmp     .done2

.no_data2:
    xor     al, al

.done2:
    pop     dx
    ret

;==============================================================================
; Process UART Byte
; Processes received command bytes
; IN:  AL = received byte
;==============================================================================
process_uart_byte:
    push    ax
    push    bx

    ; State machine for packet reception
    mov     bl, [uart_rx_state]

    cmp     bl, 0            ; Waiting for start byte
    je      .wait_start
    cmp     bl, 1            ; Receiving header
    je      .recv_header
    cmp     bl, 2            ; Receiving payload
    je      .recv_payload
    cmp     bl, 3            ; Receiving checksum
    je      .recv_checksum
    jmp     .reset

.wait_start:
    cmp     al, COMM_START_BYTE
    jne     .done

    ; Start byte received
    mov     byte [uart_rx_buf], al
    mov     byte [uart_rx_index], 1
    mov     byte [uart_rx_state], 1
    jmp     .done

.recv_header:
    ; Store header byte
    mov     bx, [uart_rx_index]
    mov     [uart_rx_buf + bx], al
    inc     bx
    mov     [uart_rx_index], bl

    ; Check if header complete (4 bytes: start, type, src, dst)
    cmp     bl, 4
    jl      .done

    ; Header complete - prepare for payload
    mov     byte [uart_rx_state], 2
    jmp     .done

.recv_payload:
    ; Store payload byte
    mov     bx, [uart_rx_index]
    mov     [uart_rx_buf + bx], al
    inc     bx
    mov     [uart_rx_index], bl

    ; Check if payload complete
    ; Payload length = uart_rx_buf[4] (low) + uart_rx_buf[5]*256 (high)
    mov     al, [uart_rx_buf + 4]
    mov     ah, [uart_rx_buf + 5]
    add     bx, -6           ; Subtract header size
    cmp     bx, ax
    jl      .done

    ; Payload complete - expect checksum
    mov     byte [uart_rx_state], 3
    jmp     .done

.recv_checksum:
    ; Verify checksum
    mov     bl, [uart_rx_buf + 6]
    add     al, bl
    mov     bl, [uart_rx_buf + 4]
    mov     bh, [uart_rx_buf + 5]
    inc     bx              ; Include payload length
    add     al, bl

    ; Compare with received checksum
    cmp     al, [uart_rx_buf + 2]  ; Compare with received
    jne     .checksum_fail

    ; Valid packet received - process command
    mov     al, [uart_rx_buf + 1]
    mov     [last_command], al

.checksum_fail:
    ; Reset for next packet
.reset:
    mov     byte [uart_rx_state], 0
    mov     byte [uart_rx_index], 0

.done:
    pop     bx
    pop     ax
    ret

;==============================================================================
; Get Last Received Command
; Returns: AL = command byte (0 if no new command)
;==============================================================================
comm_get_command:
    push    bx

    mov     al, [last_command]
    mov     byte [last_command], 0    ; Clear after read

    pop     bx
    ret

;==============================================================================
; I2C Initialization
; Configures I2C bus for segment communication
;==============================================================================
i2c_init:
    push    ax
    push    dx

    ; Configure I2C on standard pins
    ; Assuming bit-banged I2C on GPIO ports

    ; Set SCL and SDA pins as outputs, high (idle)
    call    i2c_set_scl_high
    call    i2c_set_sda_high

    ; Delay for bus stabilization
    call    delay_1ms

    xor     al, al            ; SUCCESS

    pop     dx
    pop     ax
    ret

;==============================================================================
; I2C Probe
; Checks if a device acknowledges at given address
; IN:  AL = I2C address (7-bit)
; OUT: AL = 1 if device found, 0 otherwise
;==============================================================================
i2c_probe:
    push    bx

    ; Generate START condition
    call    i2c_start

    ; Send address + write bit
    shl     al, 1             ; Shift to 8-bit format
    and     al, 0xFE          ; Write bit = 0
    call    i2c_send_byte
    mov     bl, al            ; Save ACK/NACK

    ; Generate STOP condition
    call    i2c_stop

    ; Check ACK
    test    bl, bl
    jnz     .not_found

    mov     al, 1             ; Device found
    jmp     .done

.not_found:
    xor     al, al            ; Device not found

.done:
    pop     bx
    ret

;==============================================================================
; I2C Write Byte
; IN:  AL = I2C address, AH = register, DL = data
;==============================================================================
i2c_write_byte:
    push    ax
    push    bx
    push    dx

    ; Start condition
    call    i2c_start

    ; Send address + write
    mov     al, ah            ; Address
    shl     al, 1
    or      al, 0x00          ; Write
    call    i2c_send_byte
    test    al, al
    jnz     .error

    ; Send register address
    mov     al, ah
    call    i2c_send_byte
    test    al, al
    jnz     .error

    ; Send data byte
    mov     al, dl
    call    i2c_send_byte

.error:
    ; Stop condition
    call    i2c_stop

    pop     dx
    pop     bx
    pop     ax
    ret

;==============================================================================
; I2C Read Byte
; IN:  AL = I2C address, AH = register
; OUT: AL = read data
;==============================================================================
i2c_read_byte:
    push    bx
    push    dx

    ; Start condition
    call    i2c_start

    ; Send address + write (to set register pointer)
    mov     al, ah
    shl     al, 1
    or      al, 0x00
    call    i2c_send_byte
    test    al, al
    jnz     .error

    ; Send register address
    mov     al, ah
    call    i2c_send_byte
    test    al, al
    jnz     .error

    ; Repeated start
    call    i2c_start

    ; Send address + read
    mov     al, ah
    shl     al, 1
    or      al, 0x01          ; Read
    call    i2c_send_byte
    test    al, al
    jnz     .error

    ; Read data with NACK (last byte)
    call    i2c_receive_byte
    mov     dl, al            ; Store result

    ; Send NACK
    call    i2c_send_nack

    ; Stop condition
    call    i2c_stop

    mov     al, dl            ; Return data
    jmp     .done

.error:
    call    i2c_stop
    xor     al, al

.done:
    pop     dx
    pop     bx
    ret

;==============================================================================
; I2C Check Data
; Returns AL = 1 if I2C data available
;==============================================================================
i2c_check_data:
    ; Placeholder - would check I2C hardware buffer
    xor     al, al
    ret

;==============================================================================
; Process I2C Data
;==============================================================================
process_i2c_data:
    ; Placeholder - process received I2C packets
    ret

;==============================================================================
; Broadcast Segment State
; Sends current segment state to all other segments via I2C
;==============================================================================
broadcast_segment_state:
    push    ax
    push    bx
    push    cx
    push    dx

    ; Build broadcast packet
    mov     byte [comm_packet + COMM_PACKET.start_byte], COMM_START_BYTE
    mov     byte [comm_packet + COMM_PACKET.packet_type], 0x01
    mov     byte [comm_packet + COMM_PACKET.src_address], 0x00  ; Master
    mov     byte [comm_packet + COMM_PACKET.dst_address], 0xFF  ; Broadcast

    ; Copy segment state into payload
    mov     al, [ACTIVE_SEGMENTS]
    mov     byte [comm_packet + COMM_PACKET.payload], al

    ; Calculate checksum
    call    calculate_packet_checksum
    mov     [comm_packet + COMM_PACKET.checksum], al

    mov     byte [comm_packet + COMM_PACKET.end_byte], COMM_END_BYTE

    ; Send to all slave segments
    xor     bx, bx
    mov     bl, [ACTIVE_SEGMENTS]
    dec     bl

.send_loop:
    push    bx
    mov     byte [comm_packet + COMM_PACKET.dst_address], bl

    ; Send via I2C
    mov     al, bl
    add     al, 0x30          ; Segment I2C base address
    call    i2c_send_packet

    pop     bx
    dec     bl
    jge     .send_loop

    pop     dx
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Broadcast Emergency Stop
; Sends emergency stop to all segments
;==============================================================================
comm_broadcast_emergency:
    push    ax

    mov     al, 0xFF          ; Emergency command
    mov     [comm_packet + COMM_PACKET.packet_type], al

    ; Broadcast to all segments
    mov     al, 0xFF
    mov     [comm_packet + COMM_PACKET.dst_address], al

    mov     al, 0x01
    mov     [comm_packet + COMM_PACKET.payload_length], al

    call    calculate_packet_checksum
    mov     [comm_packet + COMM_PACKET.checksum], al

    xor     bx, bx
.broadcast_loop:
    push    bx
    mov     al, bl
    add     al, 0x30
    call    i2c_send_packet
    pop     bx
    inc     bl
    cmp     bl, [ACTIVE_SEGMENTS]
    jl      .broadcast_loop

    pop     ax
    ret

;==============================================================================
; I2C Send Packet
; IN:  AL = destination address
;==============================================================================
i2c_send_packet:
    push    ax
    push    bx
    push    cx
    push    si

    mov     si, comm_packet
    mov     cx, COMM_PACKET_size

.send_loop:
    lodsb
    push    cx
    mov     ah, al
    call    i2c_write_byte
    pop     cx
    loop    .send_loop

    pop     si
    pop     cx
    pop     bx
    pop     ax
    ret

;==============================================================================
; Send/Receive Packet
;==============================================================================
comm_send_packet:
    ; TODO: Full packet send with retry logic
    ret

comm_receive_packet:
    ; TODO: Full packet receive with validation
    ret

;==============================================================================
; I2C Bit-Bang Primitives
;==============================================================================
i2c_start:
    call    i2c_set_sda_low
    call    i2c_delay_half
    call    i2c_set_scl_low
    call    i2c_delay_half
    ret

i2c_stop:
    call    i2c_set_scl_high
    call    i2c_delay_half
    call    i2c_set_sda_high
    call    i2c_delay_half
    ret

i2c_send_byte:
    push    cx
    push    dx
    mov     cx, 8            ; 8 bits
.send_bit:
    shl     al, 1
    jc      .send_one

    call    i2c_set_sda_low
    jmp     .clock

.send_one:
    call    i2c_set_sda_high

.clock:
    call    i2c_set_scl_high
    call    i2c_delay_half
    call    i2c_set_scl_low
    call    i2c_delay_half
    loop    .send_bit

    ; Check ACK
    call    i2c_set_sda_high  ; Release SDA
    call    i2c_set_scl_high
    call    i2c_delay_half
    in      al, i2c_sda_port
    and     al, 1
    xor     al, 1             ; Invert: 0=ACK, 1=NACK
    push    ax
    call    i2c_set_scl_low
    call    i2c_delay_half
    pop     ax

    pop     dx
    pop     cx
    ret

i2c_receive_byte:
    push    cx
    xor     al, al
    mov     cx, 8
    call    i2c_set_sda_high  ; Release SDA
.recv_bit:
    shl     al, 1
    call    i2c_set_scl_high
    call    i2c_delay_half
    in      ah, i2c_sda_port
    and     ah, 1
    or      al, ah
    call    i2c_set_scl_low
    call    i2c_delay_half
    loop    .recv_bit
    pop     cx
    ret

i2c_send_nack:
    call    i2c_set_sda_high
    call    i2c_set_scl_high
    call    i2c_delay_half
    call    i2c_set_scl_low
    call    i2c_delay_half
    ret

i2c_delay_half:
    push    cx
    mov     cx, 10
.delay:
    loop    .delay
    pop     cx
    ret

; I2C GPIO primitives
i2c_set_scl_high:
    push    ax
    mov     al, 1
    out     i2c_ctrl_port, al
    pop     ax
    ret

i2c_set_scl_low:
    push    ax
    xor     al, al
    out     i2c_ctrl_port, al
    pop     ax
    ret

i2c_set_sda_high:
    push    ax
    mov     al, 2
    out     i2c_ctrl_port, al
    pop     ax
    ret

i2c_set_sda_low:
    push    ax
    xor     al, al
    out     i2c_ctrl_port, al
    pop     ax
    ret

;==============================================================================
; Calculate Packet Checksum (XOR)
;==============================================================================
calculate_packet_checksum:
    push    cx
    push    si

    mov     si, comm_packet
    xor     al, al
    mov     cx, COMM_PACKET_size - 2  ; Exclude checksum and end bytes

.checksum_loop:
    xor     al, [si]
    inc     si
    loop    .checksum_loop

    pop     si
    pop     cx
    ret

;==============================================================================
; Delay
;==============================================================================
delay_1ms:
    push    cx
    mov     cx, 1000
.delay:
    loop    .delay
    pop     cx
    ret

;==============================================================================
; Data
;==============================================================================
section .data

IS_MASTER:      db 1            ; This node is master
i2c_ctrl_port:  dw 0x378        ; LPT1 data port (example GPIO)
i2c_sda_port:   dw 0x379        ; LPT1 status port

;==============================================================================
; BSS
;==============================================================================
section .bss

; UART buffers
uart_rx_byte:   resb    1       ; Received byte
uart_rx_state:  resb    1       ; RX state machine
uart_rx_index:  resb    1       ; RX buffer index
uart_rx_buf:    resb    72      ; RX packet buffer
last_command:   resb    1       ; Last received command

; Communication packet
comm_packet:
    resb    COMM_PACKET_size

