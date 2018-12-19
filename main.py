# allow less secure apps to access your Gmail at: https://support.google.com/accounts/answer/6010255?hl=en
# guide for setting up PiCamera at: https://www.raspberrypi.org/learning/getting-started-with-picamera/worksheet/
# guide for connecting PIR sensor to Pi at: https://www.raspberrypi.org/learning/parent-detector/worksheet/
# requires your email password to run (line 56), obviously a security hazard so be careful.

from gpiozero import MotionSensor
from picamera import PiCamera
from datetime import datetime
from email.mime.multipart import MIMEMultipart
from email.mime.base import MIMEBase
from email.mime.text import MIMEText
import email.encoders
import smtplib
import os
import email
import sys
import time
import dht11
import RPi.GPIO as GPIO
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
from threading import Timer

ad_photores_mail=0
ad_photores_light=5

i_tilt_gpio_number = 5
i_humidity_gpio_number = 27
i_sec_button_gpio_number = 13
o_flag_gpio_number = 17
o_led_white_in_gpio_number = 21
o_led_red_alarm_gpio_number = 16
o_led_red_ir_gpio_number = 20



def isPhotoresistorInOn(threshold):
    #photo resistors
    CLK  = 26
    MISO = 19
    MOSI = 12
    CS   = 6
    mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

    if mcp.read_adc(ad_photores_mail) >= threshold:
        return True
    else:
        return False

def isPhotoresistorOutOn(threshold):
    #photo resistors
    CLK  = 26
    MISO = 19
    MOSI = 12
    CS   = 6
    mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)
    print (mcp.read_adc(ad_photores_light))
    if mcp.read_adc(ad_photores_light) >= threshold:
        return True
    else:
        return False

def tiltOn(ev=None):
    print ("In tiltOn func")

    isNight = False
    if isPhotoresistorOutOn(800):
        isNight = True
    
    if securityOn:
        alarm()
        alarm()
        alarm()
        print ("After alarm")

        if isNight:
            startFlash()
            
        takePhoto()
        print ("After takePhoto")
        sendEmail("Please check the attach photo", True)
        emailType = 0

        if isNight:
            stopFlash()

def startFlash():
    print ('startFlash')
    turnOnLed(o_led_red_ir_gpio_number)
    #time.sleep(1)

def stopFlash():
    print ('stopFlash')    
    turnOffLed(o_led_red_ir_gpio_number)
    #time.sleep(1)

def timeout():
    if(emailSent):
        email = False;

def initAll():
    global hum_sensor_value
    global footage
    global securityOn
    global isNight
    #global emailSent
    global emailType

    #emailSent = False

    footage = '/home/pi/smartbox/Intruder.jpg'
    securityOn = True
    emailType = 0

    #GPIO common settings
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    #tilt sensor
    tilt_sensor_value = GPIO.setup(i_tilt_gpio_number, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.add_event_detect(i_tilt_gpio_number, GPIO.FALLING, callback=tiltOn, bouncetime=100) 

    #humidity sensor
    hum_sensor_value = dht11.DHT11(pin=i_humidity_gpio_number)

    #security button
    GPIO.setup(i_sec_button_gpio_number, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    #LED
    GPIO.setup(o_led_white_in_gpio_number, GPIO.OUT)
    GPIO.setup(o_led_red_alarm_gpio_number, GPIO.OUT)
    GPIO.setup(o_led_red_ir_gpio_number, GPIO.OUT)

    
    #Flag motor
    GPIO.setup(o_flag_gpio_number, GPIO.OUT)
    
    #Turning off all LEDs
    turnOffLed(o_led_white_in_gpio_number)
    turnOffLed(o_led_red_alarm_gpio_number)
    turnOffLed(o_led_red_ir_gpio_number)

def moveFlag(upFlag):
    p=GPIO.PWM(o_flag_gpio_number, 50)
    p.start(0)
    p.ChangeDutyCycle(0)
    
    if upFlag:
    #Flag up
       p.ChangeDutyCycle(5)
       time.sleep(2)
    else:
       #Flag down
       p.ChangeDutyCycle(12)
       time.sleep(2)

    p.stop()

def isWaterInsideBox(threshold):
    result = hum_sensor_value.read()
    if result.is_valid():
        if result.humidity >= threshold:
            return True
        else:
            return False


def takePhoto():
    print("camera")

    camera = PiCamera()

    camera.rotation = 180
    camera.resolution=(2592,1944)
    camera.framerate=15
    #camera.annotate_text="Hello there!"
    camera.annotate_text_size=100
    #camera.brightness=60
    #camera.start_preview()
    #camera.image_effect = 'colorswap'

    time.sleep(1)
        
    camera.capture(footage)
    

    #camera.stop_preview()
    camera.close()

def sendEmail(textMessage,isAttach):
    print("sendEmail")
    # prepare the email
    f_time = datetime.now().strftime("%A %B %d %Y @ %H:%M:%S")
    msg = MIMEMultipart()
    msg["Subject"] = f_time
    msg["From"] = "your_address@gmail.com"
    msg["To"] = "to_address@gmail.com"
    text = MIMEText(textMessage)
    msg.attach(text)

    if isAttach:
        print ('attach mp4 video to email')
        # attach mp4 video to email
        part = MIMEBase("application", "octet-stream")
        part.set_payload(open(footage, "rb").read())
        email.encoders.encode_base64(part)
        part.add_header("Content-Disposition", "attachment; filename= %s" % os.path.basename(footage))
        msg.attach(part)

    print ('access Gmail account and send email')
    # access Gmail account and send email
    server = smtplib.SMTP("smtp.gmail.com:587")
    server.starttls()
    print ('login')
    server.login("qa069382@gmail.com","tsdtsdtsd")
    print ('sendmail')
    server.sendmail("qa069382@gmail.com", "qa069382@gmail.com", msg.as_string())
    server.quit()
    #emailSent = True;
    #t = Timer(600, timeout)
    #t.start()

    # delete mp4 from Pi after it has been emailed
    # os.system("rm " + footage)
    
def isPassword():
    button_state1 = GPIO.input(i_sec_button_gpio_number)
    if button_state1 == True:
        print("Wait 1 sec")
        time.sleep(1)
        print ("Now!")
        button_state2 = GPIO.input(i_sec_button_gpio_number)
        if button_state2 == True:
            print ("YES!")
            ret=True
        else:
            ret=False
    else:
        ret=False
    return ret

def alarm():
    turnOnLed(o_led_red_alarm_gpio_number)
    time.sleep(0.1)
    turnOffLed(o_led_red_alarm_gpio_number)
    time.sleep(0.1)

def turnOnLed(led_pin):
    GPIO.output(led_pin,GPIO.HIGH)

def turnOffLed(led_pin):
    GPIO.output(led_pin,GPIO.LOW)

initAll()

while True:
    
    if(isWaterInsideBox(70)):
        print ("THERE IS WATER INSIDE THE BOX!!!!")
        if (emailType != 1):
            sendEmail("Water is inside the box", False)
            emailType = 1
    if(isPhotoresistorInOn(800)):       
        if (emailType != 2):
            print ("Photoresistor")
            sendEmail("There is a mail in your box", False)
            emailType = 2
        moveFlag(True)
    else:
        moveFlag(False)
    if isPassword():
        print ("Password")
        if securityOn:
            print ("Security disabled")
            securityOn = False
        else:
            print ("Security enabled")
            securityOn = True
    #print ("Before sleep")    
    #time.sleep(1)
    #print ("After sleep, end of loop")



