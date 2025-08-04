import RPi.GPIO as GPIO #For Raspberry Pi GPIO Controls
import gpiozero #Only used for LED controls
import time #For setting delays and other time functions.
from PCF8574 import PCF8574_GPIO #For LCD I2C GPIO Expansion
from Adafruit_LCD1602 import Adafruit_CharLCD #For LCD display controls
from ADCDevice import ADS7830 #To import ADS7830 adc converter library into the program.

# Inputs declarations
IN1 = 24    #GPIO pin for IN1 (physical pin 18).  
OnButt = 17    #GPIO pin for Start Button. Pin 17.
OffButt = 18    #GPIO pin for Stop Button. Pin 18.
red_led = gpiozero.LED(26) #GPIOzero for LED Red. It is used to indicate that the fan is not spinning.
green_led = gpiozero.LED(27) #GPIOzero for LED Green. It is used to indicate that the fan is spinning.
GPIO.setmode(GPIO.BCM)       #Use Broadcom pin numbering
GPIO.setwarnings(False)      #Stop the popups from interrupting the program
GPIO.setup(IN1, GPIO.OUT)    #Set IN1 as output
GPIO.setup(OnButt,GPIO.IN, pull_up_down = GPIO.PUD_UP) #Set OnButt as button input.
GPIO.setup(OffButt,GPIO.IN, pull_up_down = GPIO.PUD_UP) #Set OffButt as button input.
lm35_channel = 0 # Initialize ADC channel at channel 0 for the lm35 readings to be read.

# Temperature thresholds (in Celsius). 20C is set for minimum temperature reading, 25C is the mid-point and 30C would be labelled as high temperature.
LOW_TEMP = 20
MED_TEMP = 25
HIGH_TEMP = 30

# Manual mode settings
MANUAL_SPEEDS = [25, 60, 100] #These numbers are the % for the Duty Cycle percentages. 
HOLD_TIME = 3  # 3 seconds for button hold detection to switch between manual mode and auto mode.

# Setup for I2C Detections. LCD for LCD and ADS for adc converter.
mcp = None #Start as none until the address of the lcd is read. A placeholder for PCF8574.
LCD1 = None #It is initialized as "None" to ensure that the variable exists in the program firstly when "setup()" is run.

def read_temperature():
    #Read LM35 temperature in °C
    adc_value = adc.analogRead(lm35_channel)  # New way (using ADCDevice)
    voltage = (adc_value / 255.0) * 3.3 #3.3 being 3.3V reference and 255 being the ADC output values from 0-255 (2^8 = 256).
    temp_c = voltage * 100 #Since the lm35 reads 10mV/C, the equation for temp_c is arranged as such so the voltage * 100 will give the temperature value.
    return temp_c

def setup():
    global LCD1, mcp, adc, pwm_motor #Declaring these variables as global to avoid needing to declare them repeatedly.
    PCF8574_address = 0x27 #Address for LCD.
    PCF8574A_address = 0x3F #Alternative LCD address. Not fully needed but could have it here just in case.
    pwm_motor= GPIO.PWM(IN1,100)#Set pwm freq to 100Hz.
    pwm_motor.start(0) # Start pwm with 0 duty cycle.
    adc = ADS7830() # Initialize ADC device
    
    if adc.detectI2C(0x4b): #Check ADC is detected in I2C address 0x4B.
        print("ADS7830 is connected")
        
    else:
        print("Not connected.\n"
              "Use i2cdetect -y 1 to find it.\n"
              "Program Exit.")
        exit(-1)
    
    # Create PCF8574 GPIO adaptor. 
    try:
        mcp = PCF8574_GPIO(PCF8574_address) #Try and look for LCD address.
    except:
        try:
            mcp = PCF8574_GPIO(PCF8574A_address) #If first address not found, look for alternative address instead.
        except:
            print('I2C Address Error!')
            exit(1)
              
    LCD1 = Adafruit_CharLCD(pin_rs=0, pin_e=2, pins_db=[4,5,6,7], GPIO=mcp) #Inizialized LCD
    #pins_rs=0: Register Select via GPIO0. Controls HIGH or LOW commands to LCD.
    #pins_e=2: Enable pin connected to GPIO2. Clock signal reading LCD data pins.
    #Pins_db=[4,5,6,7]: Data bus pins GPIO 4-7.4 datalines for 4-bit mode communication (D4-D7).

def loop():
    mcp.output(3, 1) # Turn on LCD backlight
    LCD1.begin(16, 2) # Set number of LCD lines and columns
    LCD1.message("Press 'Start'")
    print ("Start")
    
    running = False  # State variable to track if fan is running
    manual_mode = False  # Track if in manual mode. Starts false.
    manual_speed_index = 0  # Current manual speed index
    last_update = time.time()  # Initialize last_update here
    
    while True:
        if not running:
            # Check for start button press
            if GPIO.input(OnButt) == 0:  # Button is pressed
                press_start = time.time() #time.time() in the current time. In this line, press_start is the time the moment it is pressed.
                                
                # Wait for button release or hold
                while GPIO.input(OnButt) == 0:
                    if time.time() - press_start >= HOLD_TIME: #HOLD_TIME value is 3 seconds. Holding the Start button past 3 seconds will start manual mode. Time.time() will continue while the press_start value stays at the moment of press.
                        #Manual Mode Start
                        manual_mode = True
                        running = True
                        red_led.off()
                        green_led.on()
                        LCD1.clear()
                        pwm_motor.start(0) #PWM 0 lines are in each auto/manual prompt to reset the PWM.
                        pwm_motor.ChangeDutyCycle(50) #Afterwards, have the duty cycle jump start at 50% so the fan can start properly. Without the jumpstart, the fans won't start properly.
                        LCD1.message("Manual Mode")
                        print("Manual mode started")
                        break
                    time.sleep(0.1)
                
                if not running:  # If the fan is not on, the start button will just turn on auto mode. Auto mode being the one where the fan changes speed based on temperature.
                    #Auto Mode Start
                    running = True
                    manual_mode = False
                    red_led.off()
                    green_led.on()
                    LCD1.clear()
                    LCD1.message("Auto Mode")
                    print("Fan started (auto mode)")
                    pwm_motor.start(0)
                    pwm_motor.ChangeDutyCycle(50)
                    last_update = time.time()  # Initialize when entering auto mode
                
                # Wait for button release if still pressed
                while GPIO.input(OnButt) == 0:
                    time.sleep(0.1)
            else:
                red_led.on()
                green_led.off()
                time.sleep(0.1)
                continue 
        
        # When running, check buttons
        while running:
            # Check for stop button press
            # Stop button sets parameters to their intended defaults.
            if GPIO.input(OffButt) == 0:
                running = False
                manual_mode = False
                green_led.off()
                red_led.on()
                pwm_motor.stop()
                LCD1.clear()
                LCD1.message("Press 'Start'")
                print("Fan stopped")
                # Wait for button release
                while GPIO.input(OffButt) == 0:
                    time.sleep(0.1)
                break
            
            # Handle button presses in both modes
            if GPIO.input(OnButt) == 0:
                press_start = time.time()
                button_handled = False
                
                # Wait for button release or hold
                while GPIO.input(OnButt) == 0:
                    if time.time() - press_start >= HOLD_TIME:
                        # Button held for 3 seconds to toggle modes
                        manual_mode = not manual_mode #When holding the Start button, it will toggle between the two. If the initial state is not manual_mode, it will switch to manual. Vice versa for manual to auto.
                        button_handled = True
                        
                        if manual_mode:
                            # Switching to manual mode
                            LCD1.clear()
                            LCD1.message("Manual Mode")
                            pwm_motor.start(0)
                            pwm_motor.ChangeDutyCycle(50)
                            print("Switched to manual mode")
                        else:
                            # Switching to auto mode
                            LCD1.clear()
                            LCD1.message("Auto Mode")
                            pwm_motor.start(0)
                            pwm_motor.ChangeDutyCycle(50)
                            print("Switched to auto mode")
                            last_update = time.time()  # Initialize when switching to auto mode
                        
                        # Wait for button release
                        while GPIO.input(OnButt) == 0:
                            time.sleep(0.1)
                        break
                    time.sleep(0.1)
                
                if not button_handled and manual_mode:
                    # Short press in manual mode - cycle speeds
                    manual_speed_index = (manual_speed_index + 1) % len(MANUAL_SPEEDS) #Starts at Low speed.
                    speed = MANUAL_SPEEDS[manual_speed_index]
                    pwm_motor.ChangeDutyCycle(speed)
                    LCD1.setCursor(0, 0)
                    LCD1.message("Manual Mode")
                    LCD1.setCursor(0, 1)
                    LCD1.message(f"Speed: {speed}%")
                    print(f"Manual speed set to {speed}%")
                    
                    # Wait for button release
                    while GPIO.input(OnButt) == 0:
                        time.sleep(0.1)
            
            # Update temperature and fan speed in auto mode
            #This is the program lines that run the temperature sensing mode aka auto mode.
            if not manual_mode:
                current_time = time.time()
                if current_time - last_update >= 1: #This is an alternative for 1 second sleep.
                    temp = read_temperature()
                    duty_cycle = max(25, min(100, (temp - 20) * 5)) #Paradoxal as the min acts as the upper limit while max acts as the lower limit. But the function works from in to out.
                    pwm_motor.ChangeDutyCycle(duty_cycle)
                    
                    if temp >= HIGH_TEMP:
                        speed_msg = "HIGH"
                    elif temp >= MED_TEMP:
                        speed_msg = "MED"
                    elif temp >= LOW_TEMP:
                        speed_msg = "LOW"
                    
                    # Display on LCD
                    LCD1.setCursor(0, 0)
                    LCD1.message(f"Temp: {temp:.1f}C")
                    LCD1.setCursor(0, 1)
                    LCD1.message(f"Fan: {speed_msg}")
                    
                    print(f"Motor PWM: {duty_cycle:.2f}%")
                    print(f"Temperature: {temp:.2f} °C")
                    print(f"Speed: {speed_msg}")
                    
                    last_update = current_time
            
            time.sleep(0.05)

def destroy():
    adc.close() #Disconnects ADC reading resources
    mcp.output(3, 0) # Turn off LCD backlight
    LCD1.clear() #Clear LCD
    red_led.off() #Turn red LED off
    green_led.off() #Turn green LED off
    GPIO.cleanup() #Rest GPIO pins
    pwm_motor.stop() #Stop PWM.
     
if __name__ == "__main__":
    setup()
    try:
        loop()
    except KeyboardInterrupt:
        destroy()
        print("\nExiting...")