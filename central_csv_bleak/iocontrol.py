import time
import RPi.GPIO as GPIO

class IO_Control:
    def __init__(self, led0, led1, led2, switch):
        self.led0 = led0
        self.led1 = led1
        self.led2 = led2
        self.switch = switch
        
        #
        # GPIO Setup
        #
        GPIO.setwarnings(False)  # Ignore warning for now
        GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
        
        # Inputs:
        GPIO.setup(switch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin "switch" to be an input pin and set initial value to be pulled low (off)
        
        # Outputs:
        GPIO.setup(led0, GPIO.OUT)
        GPIO.setup(led1, GPIO.OUT)
        GPIO.setup(led2, GPIO.OUT)
        
        # All LEDs off
        self.leds_off()
        
    def is_switch_on(self):
        if GPIO.input(self.switch) == GPIO.HIGH:
            return True
        else:
            return False
        
    def is_switch_off(self):
        if GPIO.input(self.switch) != GPIO.HIGH:
            return True
        else:
            return False
    
    def leds_on(self):
        GPIO.output(self.led0, 1)
        GPIO.output(self.led1, 1)
        GPIO.output(self.led2, 1)
        
    def leds_off(self):
        GPIO.output(self.led0, 0)
        GPIO.output(self.led1, 0)
        GPIO.output(self.led2, 0)
        
    def led0_on(self):
        GPIO.output(self.led0, 1)
    
    def led1_on(self):
        GPIO.output(self.led1, 1)
    
    def led2_on(self):
        GPIO.output(self.led2, 1)
        
    def led0_off(self):
        GPIO.output(self.led0, 0)
    
    def led1_off(self):
        GPIO.output(self.led1, 0)
    
    def led2_off(self):
        GPIO.output(self.led2, 0)         
    
    def wait_switch_on(self, delay):
        GPIO.output(self.led0, 1)
        time.sleep(delay)
        GPIO.output(self.led0, 0)
        GPIO.output(self.led1, 1)
        time.sleep(delay)
        GPIO.output(self.led1, 0)
        GPIO.output(self.led2, 1)
        time.sleep(delay)
        GPIO.output(self.led2, 0)
        
    def error(self, delay):
        for k in range(10):
            GPIO.output(self.led0, 1)
            GPIO.output(self.led1, 1)
            GPIO.output(self.led2, 1)
            time.sleep(delay)
            GPIO.output(self.led0, 0)
            GPIO.output(self.led1, 0)
            GPIO.output(self.led2, 0)
            time.sleep(delay/2)
        
        GPIO.output(self.led0, 1)
        GPIO.output(self.led1, 1)
        GPIO.output(self.led2, 1)
        time.sleep(delay*10)
        
    def successful_measurement(self, delay):
        GPIO.output(self.led2, 1)
        for k in range(10):
            GPIO.output(self.led0, 1)
            GPIO.output(self.led1, 1)
            time.sleep(delay)
            GPIO.output(self.led0, 0)
            GPIO.output(self.led1, 0)
            time.sleep(delay)
        GPIO.output(self.led2, 0)
        
    def sleep_mode_on(self):
        GPIO.output(self.led0, 1)
        GPIO.output(self.led1, 1)
        GPIO.output(self.led2, 0)
        
    def sleep_mode_off(self):        
        GPIO.output(self.led0, 0)
        GPIO.output(self.led1, 0)