import time
from machine import Pin, I2C, ADC
import random
import sys  # Added for tracking initialization state

# PCA9685 constants
PCA9685_ADDRESS = 0x40
MODE1 = 0x00
MODE2 = 0x01
SUBADR1 = 0x02
SUBADR2 = 0x03
SUBADR3 = 0x04
PRESCALE = 0xFE
LED0_ON_L = 0x06
LED0_ON_H = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09
ALL_LED_ON_L = 0xFA
ALL_LED_ON_H = 0xFB
ALL_LED_OFF_L = 0xFC
ALL_LED_OFF_H = 0xFD

# Servo channel mapping on PCA9685
SERVO_CHANNELS = {
    "LR": 0,  # Left/Right movement
    "UD": 1,  # Up/Down movement
    "TL": 2,  # Top Left eyelid
    "BL": 3,  # Bottom Left eyelid
    "TR": 4,  # Top Right eyelid
    "BR": 5,  # Bottom Right eyelid
}

# Min, Max - 修改了眼皮舵机的极限值以反转方向
servo_limits = {
   "LR": (40, 140),   
   "UD": (40, 140),
   "TL": (10, 90),   # 修改：原来(90,10) -> 现在(10,90) 反转方向
   "BL": (90, 10),   # 修改：原来(10,90) -> 现在(90,10) 反转方向
   "TR": (90, 10),   # 修改：原来(10,90) -> 现在(90,10) 反转方向
   "BR": (10, 90),   # 修改：原来(90,10) -> 现在(10,90) 反转方向
} 
    
class PCA9685:
    def __init__(self, i2c, address=PCA9685_ADDRESS):
        self.i2c = i2c
        self.address = address
        self.reset()
        
    def reset(self):
        self.write_byte(MODE1, 0x00)
        
    def write_byte(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytes([value]))
        
    def read_byte(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]
    
    def set_pwm_freq(self, freq_hz):
        """Set PWM frequency in Hz"""
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq_hz)
        prescaleval -= 1.0
        prescale = int(prescaleval + 0.5)
        
        oldmode = self.read_byte(MODE1)
        newmode = (oldmode & 0x7F) | 0x10    # sleep
        self.write_byte(MODE1, newmode)       # go to sleep
        self.write_byte(PRESCALE, prescale)
        self.write_byte(MODE1, oldmode)
        time.sleep_ms(5)
        self.write_byte(MODE1, oldmode | 0x80)  # restart
    
    def set_pwm(self, channel, on, off):
        """Sets PWM values for a specific channel"""
        self.i2c.writeto_mem(self.address, LED0_ON_L + 4 * channel, bytes([on & 0xFF]))
        self.i2c.writeto_mem(self.address, LED0_ON_H + 4 * channel, bytes([on >> 8]))
        self.i2c.writeto_mem(self.address, LED0_OFF_L + 4 * channel, bytes([off & 0xFF]))
        self.i2c.writeto_mem(self.address, LED0_OFF_H + 4 * channel, bytes([off >> 8]))
        
    def set_servo_angle(self, channel, angle):
        """Convert angle (0-180) to PWM value and set"""
        # Map angle 0-180 to PWM duty cycle
        # Standard servo values: 150 (0°) to 600 (180°) for pulse width
        pulse_width = int(150 + (angle * 450 / 180))
        self.set_pwm(channel, 0, pulse_width)

# Set up ESP32 I2C and PCA9685
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
pca = PCA9685(i2c)
pca.set_pwm_freq(50)  # 50Hz is standard for servos

# Set up the switches and potentiometers on ESP32
enable = Pin(13, Pin.IN, Pin.PULL_UP)
mode = Pin(14, Pin.IN, Pin.PULL_UP)
blink_pin = Pin(15, Pin.IN, Pin.PULL_UP)

# ESP32 ADC setup for PS2 joystick
UD = ADC(Pin(34))  # PS2 Joystick VRy (Vertical/Up-Down)
UD.atten(ADC.ATTN_11DB)  # Full voltage range (0-3.3V)
trim = ADC(Pin(35))       # Trim potentiometer (optional)
trim.atten(ADC.ATTN_11DB)
LR = ADC(Pin(32))         # PS2 Joystick VRx (Horizontal/Left-Right)
LR.atten(ADC.ATTN_11DB)

# Variables to track eye position and limit movement speed
last_lr_angle = 90
last_ud_angle = 90
max_speed = 15  # Maximum degrees change per cycle

# Blink state variables
blink_timer = 0
is_blinking = False

# Set all servos to mechanical center position (90°) for assembly and calibration
def calibrate():
    # Move all servos to 90° (mechanical center)
    pca.set_servo_angle(SERVO_CHANNELS["LR"], 90)   # Center left/right
    pca.set_servo_angle(SERVO_CHANNELS["UD"], 90)   # Center up/down
    pca.set_servo_angle(SERVO_CHANNELS["TL"], 90)   # Center top left eyelid
    pca.set_servo_angle(SERVO_CHANNELS["BL"], 90)   # Center bottom left eyelid
    pca.set_servo_angle(SERVO_CHANNELS["TR"], 90)   # Center top right eyelid
    pca.set_servo_angle(SERVO_CHANNELS["BR"], 90)   # Center bottom right eyelid
    print("All servos moved to mechanical center (90°)")

# Initialize controller mode - eyes centered, eyelids open
def initialize_controller_mode():
    # Center the eye movement servos
    pca.set_servo_angle(SERVO_CHANNELS["LR"], 90)
    pca.set_servo_angle(SERVO_CHANNELS["UD"], 90)
    
    # OPEN the eyelids (使用修改后的极限值)
    pca.set_servo_angle(SERVO_CHANNELS["TL"], servo_limits["TL"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["BL"], servo_limits["BL"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["TR"], servo_limits["TR"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["BR"], servo_limits["BR"][1])  # OPEN position
    print("Controller mode initialized")

# Neutral pose - eyes centered, eyelids open
def neutral():
    # Center eyes
    pca.set_servo_angle(SERVO_CHANNELS["LR"], 90)
    pca.set_servo_angle(SERVO_CHANNELS["UD"], 90)
    
    # Open eyelids (使用修改后的极限值)
    pca.set_servo_angle(SERVO_CHANNELS["TL"], servo_limits["TL"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["BL"], servo_limits["BL"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["TR"], servo_limits["TR"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["BR"], servo_limits["BR"][1])  # OPEN position

# Corrected blink function - should CLOSE the eyes
def blink():
    # Close all eyelids (使用修改后的极限值)
    pca.set_servo_angle(SERVO_CHANNELS["TL"], servo_limits["TL"][0])  # CLOSED position
    pca.set_servo_angle(SERVO_CHANNELS["BL"], servo_limits["BL"][0])  # CLOSED position
    pca.set_servo_angle(SERVO_CHANNELS["TR"], servo_limits["TR"][0])  # CLOSED position
    pca.set_servo_angle(SERVO_CHANNELS["BR"], servo_limits["BR"][0])  # CLOSED position

# Open eyes function
def open_eyes():
    # Open all eyelids (使用修改后的极限值)
    pca.set_servo_angle(SERVO_CHANNELS["TL"], servo_limits["TL"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["BL"], servo_limits["BL"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["TR"], servo_limits["TR"][1])  # OPEN position
    pca.set_servo_angle(SERVO_CHANNELS["BR"], servo_limits["BR"][1])  # OPEN position

# Improved joystick mapping function with dynamic center point
def map_joystick(value, out_min, out_max):
    # ESP32 ADC range is 0-4095 (12-bit)
    in_min = 0
    in_max = 4095
    
    # Dynamic center point calculation
    center = (in_max - in_min) // 2  # Approximately 2048
    
    # Create a deadzone around center (±300)
    deadzone = 300
    if abs(value - center) < deadzone:
        return 90  # Return exact center position
    
    # Scale based on which half of the range we're in
    if value < center:
        # Lower half of range
        scaled_value = out_min + (value - in_min) * (90 - out_min) / (center - deadzone - in_min)
    else:
        # Upper half of range
        scaled_value = 90 + (value - (center + deadzone)) * (out_max - 90) / (in_max - (center + deadzone))
        
    # Clamp to valid range
    scaled_value = max(out_min, min(out_max, scaled_value))
        
    return int(scaled_value)

def control_ud_and_lids(ud_angle):
    """
    Improved function to move UD servo and make eyelids follow based on UD's position
    """
    # Get limits (使用修改后的极限值)
    ud_min, ud_max = servo_limits["UD"]
    tl_min, tl_max = servo_limits["TL"]  # min=closed, max=open for TL
    tr_min, tr_max = servo_limits["TR"]  # min=closed, max=open for TR
    bl_min, bl_max = servo_limits["BL"]  # min=closed, max=open for BL
    br_min, br_max = servo_limits["BR"]  # min=closed, max=open for BR

    # Normalize UD position to a 0-1 range (0 = looking down, 1 = looking up)
    ud_range = ud_max - ud_min
    ud_progress = (ud_angle - ud_min) / ud_range if ud_range > 0 else 0.5
    
    # Calculate how much to close the eyelids based on looking up/down
    # When looking up, close top lids slightly
    # When looking down, close bottom lids slightly
    top_close_factor = 0.6 * (1 - ud_progress)  # Higher when looking down
    bottom_close_factor = 0.6 * ud_progress     # Higher when looking up
    
    # Find target positions - interpolate between open and partially closed
    tl_target = tl_min + (tl_max - tl_min) * (1 - top_close_factor)
    tr_target = tr_min + (tr_max - tr_min) * (1 - top_close_factor)
    bl_target = bl_min + (bl_max - bl_min) * (1 - bottom_close_factor)
    br_target = br_min + (br_max - br_min) * (1 - bottom_close_factor)
   
    # Move all servos
    pca.set_servo_angle(SERVO_CHANNELS["UD"], ud_angle)
    pca.set_servo_angle(SERVO_CHANNELS["TL"], int(tl_target))
    pca.set_servo_angle(SERVO_CHANNELS["TR"], int(tr_target))
    pca.set_servo_angle(SERVO_CHANNELS["BL"], int(bl_target))
    pca.set_servo_angle(SERVO_CHANNELS["BR"], int(br_target))

def update_eyelid_limits(trim_value):
    """
    Update eyelid limits based on trim potentiometer
    Ensures consistent behavior with min=closed, max=open
    """
    # Define a wider trim value range
    trim_min = 0
    trim_max = 4095
    
    # Define significant ranges for eyelid openness
    # For TL and BR (修改后的设置):
    # - smaller values = more closed
    # - higher values = more open
    TL_range = (10, 80)  # From barely open (10) to very open (80)
    BR_range = (10, 80)  # From barely open (10) to very open (80)
    
    # For BL and TR (修改后的设置):
    # - higher values = more closed
    # - smaller values = more open
    BL_range = (20, 90)  # From very open (20) to barely open (90)
    TR_range = (20, 90)  # From very open (20) to barely open (90)
    
    # Scale trim_value to determine how open eyelids should be
    trim_progress = (trim_value - trim_min) / (trim_max - trim_min)  # 0 to 1
    trim_progress = max(0, min(1, trim_progress))  # Clamp to [0, 1] range
    
    # Adjust MAX values (openness) based on trim
    # Keep MIN values fixed (closed position)
    servo_limits["TL"] = (TL_range[0] + (TL_range[1] - TL_range[0]) * trim_progress, 90)
    servo_limits["BR"] = (BR_range[0] + (BR_range[1] - BR_range[0]) * trim_progress, 90)
    servo_limits["BL"] = (90, BL_range[0] + (BL_range[1] - BL_range[0]) * (1-trim_progress))
    servo_limits["TR"] = (90, TR_range[0] + (TR_range[1] - TR_range[0]) * (1-trim_progress))
    
    # For debugging
    # print(f"Trim: {trim_value}, Progress: {trim_progress}")
    # print(f"Limits: TL:{servo_limits['TL']}, TR:{servo_limits['TR']}, BL:{servo_limits['BL']}, BR:{servo_limits['BR']}")

# Initialize PCA9685
print("Initializing servos...")
calibrate()  # Move all servos to mechanical center for assembly
time.sleep_ms(1000)
print("System ready")

# Flag to track if controller mode has been initialized
controller_initialized = False

# Main loop
while True:
    mode_state = not mode.value()
    enable_state = not enable.value()
    
    if mode_state == 1:  # Calibration mode when switch is in hold position
        calibrate()  # Move all servos to mechanical center
        time.sleep_ms(500)
        # Reset controller initialization flag when exiting controller mode
        controller_initialized = False
    else:
        if enable_state == 0:  # Auto mode
            # Reset controller initialization flag when exiting controller mode
            controller_initialized = False
            
            command = random.randint(0, 2)
            if command == 0:
                blink()
                time.sleep_ms(100)
                neutral()  # Return to neutral after blinking
                time.sleep_ms(random.randint(1000, 3000))
            elif command == 1:
                blink()
                time.sleep_ms(100)
                control_ud_and_lids(random.randint(servo_limits["UD"][0], servo_limits["UD"][1]))
                pca.set_servo_angle(SERVO_CHANNELS["LR"], random.randint(servo_limits["LR"][0], servo_limits["LR"][1]))
                time.sleep_ms(random.randint(300, 1000))
            elif command == 2:
                control_ud_and_lids(random.randint(servo_limits["UD"][0], servo_limits["UD"][1]))
                pca.set_servo_angle(SERVO_CHANNELS["LR"], random.randint(servo_limits["LR"][0], servo_limits["LR"][1]))
                time.sleep_ms(random.randint(200, 400))
        elif enable_state == 1:  # Controller mode
            # Initialize controller mode first time
            if not controller_initialized:
                initialize_controller_mode()
                controller_initialized = True
                
            # Reading sensors - using improved joystick mapping
            UD_value = UD.read()  # ESP32 ADC read() returns 0-4095
            trim_value = trim.read()
            LR_value = LR.read()
            blink_state = not blink_pin.value()
            
            # Update eyelid limits based on trim (optional)
            # update_eyelid_limits(trim_value)
            
            # Handle blink button with state machine
            if blink_state and not is_blinking:
                blink()
                is_blinking = True
                blink_timer = time.ticks_ms()
            elif not blink_state and is_blinking and time.ticks_diff(time.ticks_ms(), blink_timer) > 200:
                open_eyes()
                is_blinking = False
            
            # Only process eye movement if not blinking
            if not is_blinking:
                # Use improved joystick mapping
                target_lr_angle = map_joystick(LR_value, servo_limits["LR"][0], servo_limits["LR"][1])
                target_ud_angle = map_joystick(UD_value, servo_limits["UD"][0], servo_limits["UD"][1])

                # Limit speed of movement
                if abs(target_lr_angle - last_lr_angle) > max_speed:
                    lr_angle = last_lr_angle + max_speed if target_lr_angle > last_lr_angle else last_lr_angle - max_speed
                else:
                    lr_angle = target_lr_angle

                if abs(target_ud_angle - last_ud_angle) > max_speed:
                    ud_angle = last_ud_angle + max_speed if target_ud_angle > last_ud_angle else last_ud_angle - max_speed
                else:
                    ud_angle = target_ud_angle

                # Update last positions
                last_lr_angle = lr_angle
                last_ud_angle = ud_angle

                # Apply movement
                pca.set_servo_angle(SERVO_CHANNELS["LR"], lr_angle)
                control_ud_and_lids(ud_angle)
                
            time.sleep_ms(20)
            # Debug output (optional)

            # print(f"LR: {LR_value}, UD: {UD_value}, Blink: {blink_state}")
