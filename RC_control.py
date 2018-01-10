import time
import RPi.GPIO as GPIO
import pigpio
import serial
import cv2

# Setup for SERVO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(11, GPIO.OUT)

# Setup for ESC
ESC = 4

class RC_control:
    def __init__(self):
        self.ser = serial.Serial(port="/dev/ttyAMA0", baudrate=9600, parity=serial.PARITY_NONE, timeout=0.2)
        self.p = GPIO.PWM(11, 50)
        self.p.start(100.0/18+2)
        self.pi = pigpio.pi()
        self.speed = 1500
        self.pi.set_servo_pulsewidth(ESC, self.speed)
        self.forward_flag = False
        self.f = open('logs/manual_log.txt','a')
        self.f.truncate()

    def scrie(self, time, message):
        self.f.write(time + message)

    def inchide(self):
        self.f.close()

    def move(self, speed):
        self.pi.set_servo_pulsewidth(ESC, speed)
        if(speed == 1600):
            current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
            print current_time, "Masina accelereaza spre inainte"
            # f.write(current_time + "Masina accelereaza spre inainte\n")
            self.scrie(current_time, " Masina accelereaza spre inainte\n")
            self.forward_flag = True
        elif(speed == 1350):
            if(self.forward_flag):
                current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
                print current_time, "Masina trece pe retur"
##                f.write(current_time + "Masina trece pe retur\n")
                self.scrie(current_time, " Masina trece pe retur\n")
                self.forward_flag = False
            else:
                current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
                print current_time, "Masina accelereaza spre inapoi"
##                f.write(current_time + "Masina accelereaza spre inapoi\n")
                self.scrie(current_time, " Masina accelereaza spre inapoi\n")
        else:
            current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
            print current_time, "Masina franeaza treptat"
##            f.write(current_time + "Masina franeaza treptat\n")
            self.scrie(current_time, " Masina franeaza treptat\n")
            
        return self

    def servo(self, angle):
        self.duty = angle / 18 + 2
        
        if(angle < 100.0):
            current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
            print current_time, "Masina intoarce rotile spre stanga"
##            f.write(current_time + "Masina intoarce rotile spre stanga\n")
            self.scrie(current_time, " Masina intoarce rotile spre stanga\n")
        elif(angle > 100.0):
            current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
            print current_time, "Masina intoarce rotile spre dreapta"
##            f.write(current_time + "Masina intoarce rotile spre dreapta\n")
            self.scrie(current_time, " Masina intoarce rotile spre dreapta\n")
        else:
            current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
            print current_time, "Masina indreapta rotile"
##            f.write(current_time + "Masina indreapta rotile\n")
            self.scrie(current_time, " Masina indreapta rotile\n")
            
        GPIO.output(11, True)
        self.p.ChangeDutyCycle(self.duty)
        GPIO.output(11, False)

        return self

    def control(self):
        x = self.ser.read()
        if(len(x)>0):
            # print(x)
            if(x == "c"):
                self.servo(100.0)
                # time.sleep(.05)
                
            elif(x == "l"):
                self.servo(72.0)
                # time.sleep(.05)
                
            elif(x == "r"):
                self.servo(130.0)
                # time.sleep(.05)
                
            elif(x == "f"):
                self.move(1600)
                # speed = 1600
                # SetSpeed(speed)
                            
            elif(x == "b"):
                self.move(1350)
                # speed = 1350
                # SetSpeed(speed)

            elif(x == "s"):
                self.move(1500)
                # speed = 1500
                # SetSpeed(speed)
                
                  
class Automatic:
    def __init__(self):
        self.ser = serial.Serial(port="/dev/ttyAMA0", baudrate=9600, parity=serial.PARITY_NONE, timeout=0.2)
        self.p = GPIO.PWM(11, 50)
        self.p.start(100.0/18+2)
        self.pi = pigpio.pi()
        self.speed = 1500
        self.pi.set_servo_pulsewidth(ESC, self.speed)
        self.f = open('logs/automatic_log.txt','a')
        self.f.truncate()

    def scrie(self, time, message):
        self.f.write(time + message)

    def inchide(self):
        self.f.close()

    def move(self, speed):
        self.pi.set_servo_pulsewidth(ESC, speed)
        if(speed == 1600):
            current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
            print current_time, "Masina a pornit\n"
##            f.write(current_time + "Masina a pornit\n")
            self.scrie(current_time, " Masina a pornit\n\n")
        else:
            current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
            print current_time, "Masina a oprit\n"
##            f.write(current_time + "Masina a oprit\n")
            self.scrie(current_time, " Masina a oprit\n\n")

        return self

    def servo(self, angle):
        self.duty = angle / 18 + 2

        if(angle < 100.0):
            current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
            print current_time, "Masina intoarce rotile spre stanga"
##            f.write(current_time + "Masina intoarce rotile spre stanga\n")
            self.scrie(current_time, " Masina intoarce rotile spre stanga\n")
        elif(angle > 100.0):
            current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
            print current_time, "Masina intoarce rotile spre dreapta"
##            f.write(current_time + "Masina intoarce rotile spre dreapta\n")
            self.scrie(current_time, " Masina intoarce rotile spre dreapta\n")
        else:
            current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
            print current_time, "Masina indreapta rotile"
##            f.write(current_time + "Masina indreapta rotile\n")
            self.scrie(current_time, " Masina indreapta rotile\n")

            
        GPIO.output(11, True)
        self.p.ChangeDutyCycle(self.duty)
        GPIO.output(11, False)

        return self

    def control(self):
        x = self.ser.read()
        if(len(x)>0):
            # print(x)
                
            if(x == "f"):
                self.move(1600)
                # speed = 1600
                # SetSpeed(speed)

            elif(x == "b"):
                self.move(1500)
                # speed = 1350
                # SetSpeed(speed)
