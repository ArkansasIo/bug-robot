;==============================================================================
; centipede_robot/inc/structs.inc
; Data Structure Definitions
;==============================================================================

;--- Segment Control Block (SCB) ---
; Each body segment has a control block
struc SEGMENT_CTRL
    .status:            resb    1   ; Segment status flags
    .segment_id:        resb    1   ; Segment ID (0-15)
    .reserved:          resb    2   ; Alignment padding
    
    .pos_x:             resd    1   ; X position (fixed 16.16)
    .pos_y:             resd    1   ; Y position (fixed 16.16)
    .pos_z:             resd    1   ; Z position (fixed 16.16)
    .rotation:          resd    1   ; Yaw rotation (fixed 16.16)
    
    .left_leg:          resb    16  ; Left leg control data
    .right_leg:         resb    16  ; Right leg control data
    
    .sensor_data:       resb    8   ; Local sensor readings
    .comm_buffer:       resb    16  ; Communication buffer
    .error_flags:       resb    1   ; Error flags
    .gait_phase:        resb    1   ; Current gait phase
    .pad:               resb    2   ; Padding
endstruc
; Total: 64 bytes per segment

;--- Leg Control Block (LCB) ---
struc LEG_CTRL
    .coxa_angle:        resd    1   ; Coxa joint angle (fixed 16.16)
    .femur_angle:       resd    1   ; Femur joint angle (fixed 16.16)
    .tibia_angle:       resd    1   ; Tibia joint angle (fixed 16.16)
    
    .coxa_pwm:          resw    1   ; Coxa PWM duty cycle
    .femur_pwm:         resw    1   ; Femur PWM duty cycle
    .tibia_pwm:         resw    1   ; Tibia PWM duty cycle
    
    .foot_pos_x:        resd    1   ; Foot X position (fixed 16.16)
    .foot_pos_y:        resd    1   ; Foot Y position (fixed 16.16)
    .foot_pos_z:        resd    1   ; Foot Z position (fixed 16.16)
    
    .force_sensor:      resw    1   ; Ground force sensor reading
    .state_flags:       resb    1   ; Leg state flags
    .phase:             resb    1   ; Current gait phase (0-255)
endstruc
; Total: 32 bytes per leg

;--- Gait State Machine ---
struc GAIT_STATE
    .current_gait:      resb    1   ; Current gait type
    .gait_speed:        resb    1   ; Speed multiplier (0-255)
    .gait_phase:        resb    1   ; Overall gait phase position
    .pad:               resb    1   ; Padding
    
    .step_height:       resd    1   ; Current step height
    .step_length:       resd    1   ; Current step length
    .body_height:       resd    1   ; Body height from ground
    
    .leg_offsets:       resb    32  ; Phase offsets for each leg
    .leg_states:        resb    32  ; Leg state flags (0=stance, 1=swing)
endstruc
; Total: 80 bytes

;--- Sensor Fusion Block ---
struc SENSOR_FUSION
    .imu_roll:          resd    1   ; Roll angle (fixed 16.16, degrees)
    .imu_pitch:         resd    1   ; Pitch angle (fixed 16.16, degrees)
    .imu_yaw:           resd    1   ; Yaw angle (fixed 16.16, degrees)
    
    .accel_x:           resd    1   ; X acceleration (fixed 16.16)
    .accel_y:           resd    1   ; Y acceleration (fixed 16.16)
    .accel_z:           resd    1   ; Z acceleration (fixed 16.16)
    
    .gyro_x:            resd    1   ; X gyro (fixed 16.16)
    .gyro_y:            resd    1   ; Y gyro (fixed 16.16)
    .gyro_z:            resd    1   ; Z gyro (fixed 16.16)
    
    .ground_distance:   resd    1   ; Front ground distance sensor
    .obstacle_front:    resd    1   ; Front obstacle detection
    .obstacle_rear:     resd    1   ; Rear obstacle detection
    
    .temperature:       resw    1   ; Temperature (milli-celsius)
    .voltage:           resw    1   ; Battery voltage (millivolts)
    .current:           resw    1   ; Current draw (milliamps)
endstruc
; Total: 52 bytes

;--- Communication Packet ---
struc COMM_PACKET
    .start_byte:        resb    1   ; Start of frame (0xAA)
    .packet_type:       resb    1   ; Packet type identifier
    .src_address:       resb    1   ; Source segment address
    .dst_address:       resb    1   ; Destination segment address
    
    .payload_length:    resw    1   ; Payload length (bytes)
    .payload:           resb    64  ; Payload data
    .checksum:          resb    1   ; XOR checksum
    .end_byte:          resb    1   ; End of frame (0x55)
endstruc
; Total: 72 bytes

;--- PID Controller State ---
struc PID_STATE
    .kp:                resd    1   ; Proportional gain
    .ki:                resd    1   ; Integral gain
    .kd:                resd    1   ; Derivative gain
    
    .setpoint:          resd    1   ; Target value
    .integral:          resd    1   ; Integral accumulator
    .prev_error:        resd    1   ; Previous error for derivative
    
    .output_min:        resd    1   ; Output limit min
    .output_max:        resd    1   ; Output limit max
    .output:            resd    1   ; Current output
endstruc
; Total: 36 bytes

