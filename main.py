from flask import Flask, render_template, Response, request, send_from_directory
from camera import VideoCamera
import os
import serial
import time
import RPi.GPIO as GPIO
import threading
from picamera.array import PiRGBArray
from picamera import PiCamera
import kociemba
import random
import i2c_lcd
import time
import cv2
class cube:
    def __init__(self, alll):
        self.Ca = alll[0]
        self.Cb = alll[1]
        self.Cc = alll[2]
        self.Cd = alll[3]
        self.Ce = alll[4]
        self.Cf = alll[5]
        self.Cg = alll[6]
        self.Ch = alll[7]
        self.Ci = alll[8]
        self.Cj = alll[9]
        self.Ck = alll[10]
        self.Cm = alll[11]
        self.Cl = alll[12]
        self.Cn = alll[13]
        self.Co = alll[14]
        self.Cp = alll[15]
        self.Cq = alll[16]
        self.Cr = alll[17]
        self.Cs = alll[18]
        self.Ct = alll[19]
        self.Cu = alll[20]
        self.Cv = alll[21]
        self.Cw = alll[22]
        self.Cx = alll[23]

        self.Sa = alll[24]
        self.Sb = alll[25]
        self.Sc = alll[26]
        self.Sd = alll[27]
        self.Se = alll[28]
        self.Sf = alll[29]
        self.Sg = alll[30]
        self.Sh = alll[31]
        self.Si = alll[32]
        self.Sj = alll[33]
        self.Sk = alll[34]
        self.Sm = alll[35]
        self.Sl = alll[36]
        self.Sn = alll[37]
        self.So = alll[38]
        self.Sp = alll[39]
        self.Sq = alll[40]
        self.Sr = alll[41]
        self.Ss = alll[42]
        self.St = alll[43]
        self.Su = alll[44]
        self.Sv = alll[45]
        self.Sw = alll[46]
        self.Sx = alll[47]
    def translate(self):
        return (self.Ca + self.Sa + self.Cb + self.Sd + "U" + self.Sb + self.Cd + self.Sc +
                self.Cc + self.Ci + self.Si + self.Cj + self.Sm + "R" + self.Sj + self.Cm +
    self.Sk + self.Ck + self.Ce + self.Se + self.Cf + self.Sh + "F" + self.Sf + self.Ch +
                self.Sg + self.Cg + self.Cu + self.Su + self.Cv + self.Sx + "D" + self.Sv +
                self.Cx + self.Sw + self.Cw + self.Cq + self.Sq + self.Cr + self.St + "L" +
                self.Sr + self.Ct + self.Ss + self.Cs + self.Cl + self.Sl + self.Cn + self.Sp +
                "B" + self.Sn + self.Cp + self.So + self.Co)
    def reverse_translate(self):
        return self
    def U(self):
        return cube([self.Cd, self.Ca, self.Cb, self.Cc, self.Ci, self.Cj, self.Cg, self.Ch,
                     self.Cl, self.Cn, self.Ck, self.Cm, self.Cq, self.Cr, self.Co, self.Cp,
    self.Ce, self.Cf, self.Cs, self.Ct, self.Cu, self.Cv, self.Cw, self.Cx, self.Sd, self.Sa,
self.Sb, self.Sc, self.Si, self.Sf, self.Sg, self.Sh, self.Sl, self.Sj, self.Sk, self.Sm, self.Sq,
self.Sn, self.So, self.Sp, self.Se, self.Sr, self.Ss, self.St, self.Su, self.Sv, self.Sw, self.Sx])
    def UP(self):
        return self.U().U2()
    def U2(self):
        return self.U().U()
    def D(self):
        return cube([self.Ca, self.Cb, self.Cc, self.Cd, self.Ce, self.Cf, self.Cs, self.Ct, self.Ci,
                     self.Cj, self.Cg, self.Ch, self.Cl, self.Cn, self.Ck, self.Cm, self.Cq, self.Cr,
                     self.Co, self.Cp, self.Cx, self.Cu, self.Cv, self.Cw, self.Sa, self.Sb, self.Sc, self.Sd, self.Se, self.Sf, self.Ss,
                     self.Sh, self.Si, self.Sj, self.Sg, self.Sm, self.Sl, self.Sn, self.Sk, self.Sp, self.Sq, self.Sr, self.So, self.St, self.Sx, self.Su, self.Sv, self.Sw])
    def DP(self):
        return self.D().D2()
    def D2(self):
        return self.D().D()
    def R(self):
        return cube([self.Ca, self.Cf, self.Cg, self.Cd, self.Ce, self.Cv, self.Cw, self.Ch, self.Cm, self.Ci, self.Cj, self.Ck, self.Cc,
                     self.Cn, self.Co, self.Cb, self.Cq, self.Cr, self.Cs, self.Ct, self.Cu, self.Cp, self.Cl, self.Cx,self.Sa, self.Sf, self.Sc,
        self.Sd, self.Se, self.Sv, self.Sg, self.Sh, self.Sm, self.Si, self.Sj, self.Sk, self.Sl, self.Sn, self.So, self.Sb, self.Sq, self.Sr, self.Ss, self.St, self.Su, self.Sp, self.Sw, self.Sx])
    def RP(self):
        return self.R().R2()
    def R2(self):
        return self.R().R()
    def L(self):
        return cube([self.Co, self.Cb, self.Cc, self.Cn, self.Ca, self.Cf, self.Cg, self.Cd, self.Ci, self.Cj, self.Ck, self.Cm, self.Cl, self.Cx, self.Cu, self.Cp,
                     self.Ct, self.Cq, self.Cr, self.Cs, self.Ce, self.Cv, self.Cw, self.Ch,self.Sa, self.Sb, self.Sc, self.Sn, self.Se,
                     self.Sf, self.Sg, self.Sd, self.Si, self.Sj, self.Sk, self.Sm, self.Sl, self.Sx, self.So, self.Sp, self.St, self.Sq, self.Sr, self.Ss, self.Su, self.Sv, self.Sw, self.Sh])
    def LP(self):
        return self.L().L2()
    def L2(self):
        return self.L().L()
    def F(self):
        return cube([self.Ca, self.Cb, self.Cr, self.Cs, self.Ch, self.Ce, self.Cf, self.Cg, self.Cd, self.Cj, self.Ck, self.Cc, self.Cl, self.Cn, self.Co, self.Cp, self.Cq, self.Cu, self.Cv,
                     self.Ct, self.Cm, self.Ci, self.Cw, self.Cx, self.Sa, self.Sb, self.Sr, self.Sd, self.Sh, self.Se, self.Sf, self.Sg, self.Si, self.Sj, self.Sk,
                     self.Sc, self.Sl, self.Sn, self.So, self.Sp, self.Sq, self.Su, self.Ss, self.St, self.Sm, self.Sv, self.Sw, self.Sx])
    def FP(self):
        return self.F().F2()
    def F2(self):
        return self.F().F()
    def B(self):
        return cube([self.Cj, self.Ck, self.Cc, self.Cd, self.Ce, self.Cf, self.Cg, self.Ch,
                     self.Ci, self.Cw,
    self.Cx, self.Cm, self.Cp, self.Cl, self.Cn, self.Co, self.Cb, 
self.Cr, self.Cs, self.Ca, self.Cu, self.Cv, self.Ct, self.Cq,self.Sj, self.Sb, self.Sc, self.Sd, 
     self.Se, self.Sf, self.Sg, self.Sh, self.Si, self.Sw, self.Sk, self.Sm, self.Sp, self.Sl,
     self.Sn, self.So, self.Sq, self.Sr, self.Ss, self.Sa, self.Su, self.Sv, self.St, self.Sx])
    def BP(self):
        return self.B2().B()
    def B2(self):
        return self.B().B()
    def is_solved(self):
        return self.translate() == "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB"


class Nema:
    def __init__(self, dire_given, step_given, e_given, axis_enabled):
        self.dire = dire_given
        self.step = step_given
        self.e = e_given
        GPIO.setup(self.dire, GPIO.OUT)
        GPIO.setup(self.step, GPIO.OUT)
        if not axis_enabled:
            GPIO.setup(self.e, GPIO.OUT)
        GPIO.output(self.dire, GPIO.LOW)
        GPIO.output(self.step, GPIO.LOW)
        GPIO.output(self.e, GPIO.HIGH)    
    def step_turn(self, amount, direction):
        GPIO.output(self.e, GPIO.LOW)
        if direction:
            GPIO.output(self.dire, GPIO.HIGH)
        else:
            GPIO.output(self.dire, GPIO.LOW)
        for i in range(0, (50 * amount)+1):
            GPIO.output(self.step, GPIO.HIGH)
            time.sleep( 0.005)
            GPIO.output(self.step, GPIO.LOW)
            time.sleep(0.005)
        GPIO.output(self.e, GPIO.HIGH)
class robot:
    def __init__(self):
        robot.r = Nema(19, 26, 7, False)
        robot.l = Nema(27, 17, 7, True)#used to be 5
        robot.u = Nema(23, 18, 11, False)
        robot.d = Nema(25, 12, 11, True)#used to be 9
        robot.f = Nema(13, 6, 24, False)
        robot.b = Nema(22, 4, 24, True)#used to be 10
        robot.bt = False
        robot.s = serial.Serial("/dev/ttyS0", 9600)
        robot.connected = threading.Thread(target = robot.get_connection_signal)
        robot.pb = 8
        robot.pb2 = 5
        robot.pb3 = 10
        robot.pb4 = 9
        GPIO.setup(robot.pb, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(robot.pb2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(robot.pb3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(robot.pb4, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        robot.c = cube("UUUUFFFFRRRRBBBBLLLLDDDDUUUUFFFFRRRRBBBBLLLLDDDD")
        robot.lcd = i2c_lcd.lcd()
        robot.lcd.backlight_on(True)
    def get_read(s):
        received_data = s.read()              #read serial port
        time.sleep(0.03)
        data_left = s.inWaiting()             #check for remaining byte
        received_data += s.read(data_left)
        read = str(received_data.decode('utf-8'))
        return read
    def get_connection_signal():
        b = robot.get_read(robot.s)
        while not (b == "g"):
            b = robot.get_read(robot.s)
    def check_connection():
        start_time  = time.time()
        while robot.connected.is_alive():
            if (time.time() - start_time) > 60:
                return False
        return True
    def change_mode():
        if robot.bt:
            robot.bt = False
            robot.lcd_write("Controling via", "buttons")
            time.sleep(2)
        else:
            robot.bt = True
            robot.lcd_write("Controling via", "Bluetooth")
            time.sleep(2)
            robot.lcd_write("waiting for", "connection")
            robot.connected = threading.Thread(target = robot.get_connection_signal)
            robot.connected.start()
            if not robot.check_connection():
                robot.lcd_write("No connection", "")
                time.sleep(1)
                robot.change_mode()
            else:
                robot.lcd_write("connection made", "")
                time.sleep(1.5)
    def wait_for_button():
        if not robot.bt:
            while True:
                if  not GPIO.input(robot.pb):
                    return 1
                elif not GPIO.input(robot.pb2):
                    return 2
                elif not GPIO.input(robot.pb3):
                    return 3
                elif not GPIO.input(robot.pb4):
                    return 4
        else:
            b = robot.get_read(robot.s)
            if b == "a":
                return 1
            elif b == "b":
                return 2
            elif b == "c":
                return 3
            elif b == "d":
                return 4
            elif b == "h":
                return 5
        time.sleep(0.8)
    def is_solved():
        return robot.c.is_solved()        
    def lcd_write(s1, s2):
        robot.lcd.lcd_clear()
        robot.lcd.lcd_display_string(s1, 1)
        robot.lcd.lcd_display_string(s2, 2)        
    def move(moves):
        a = moves + " "
        for move in range(len(a)):
            if a[move] == ' ' or a[move] == '2' or a[move] == "'":
                 continue
            else:
                p = ""
                if a[move + 1] == "'":
                    p = "do " + a[move] + " prime"
                    p += " (" + a[move] + a[move + 1] + ")"
                elif a[move + 1] == '2':
                    p = "do " + a[move] + " twice"
                    p += " (" + a[move] + a[move + 1] + ")"
                else:
                    p = "do " + a[move]
                    p += " (" + a[move] + ")"
                print(p)
                robot.lcd_write(p, "")
                switcherm = {
                   "U ": robot.c.U(),
                   "U'": robot.c.UP(),
                   "U2": robot.c.U2() ,                      
                   "D ": robot.c.D(),
                   "D'": robot.c.DP(),
                   "D2": robot.c.D2() ,                      
                   "R ": robot.c.R(),
                   "R'": robot.c.RP(),
                   "R2": robot.c.R2() ,                      
                   "L ": robot.c.L(),
                   "L'": robot.c.LP(),
                   "L2": robot.c.L2() ,                      
                   "F ": robot.c.F(),
                   "F'": robot.c.FP(),
                   "F2": robot.c.F2() ,                      
                   "B ": robot.c.B(),
                   "B'": robot.c.BP(),
                   "B2": robot.c.B2()                                              
                    }
                switcher = {
                  "U": robot.u,
                  "D": robot.d,
                  "F": robot.f,
                  "B": robot.b,
                  "R": robot.r,
                  "L": robot.l
                   }
                s = 0
                d = False
                if a[move + 1] == " ":
                    s = 1
                    d = True
                elif a[move + 1] == "2":
                    s = 2
                    d = True
                else:
                    s = 1
                    d = False
                time.sleep(1.5)
                robot.c = switcherm.get(a[move] + a[move + 1])
                switcher.get(a[move]).step_turn(s, d)
                time.sleep(1.5)



pi_camera = VideoCamera(flip=False) 

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html') 

def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen(pi_camera),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/picture')
def take_picture():
    pi_camera.take_picture()
    return "None"
def app_start():
    app.run(host='0.0.0.0', debug=False)
def calibrate():
    robot.lcd_write("Calibrating", "")
    time.sleep(1)
    moves = ["R", "L", "U", "D", "F", "B"]
    back = [0, 0, 0, 0, 0, 0]
    ret = []
    index = 0
    bt = False
    while index < 6:
        robot.move(moves[index])
        back[index] += 1
        robot.lcd_write("Good?(" + moves[index] + ")", "y  n  stop  back")
        b = robot.wait_for_button()
        if b == 1:
            index += 1
        elif b == 2:
            continue
        elif b == 4:
            robot.lcd_write("reversing this", "move")
            time.sleep(1)
            robot.move(moves[index] + "'")
            back[index] -= 1
            if not index == 0:
                index -= 1
            else:
                robot.lcd_write("invalid", "")
                time.sleep(1)
        elif b == 5:
            bt = True
            break
        else:
            break
    robot.lcd_write("returning to", "first state")
    time.sleep(1)
    moves.reverse()
    back.reverse()
    for i in range(6):
        if back[i] % 4 == 1:
            ret.append(moves[i] + "' ")
        elif back[i] % 4 == 2:
            ret.append(moves[i] + "2 ")
        elif back[i] % 4 == 3:
            ret.append(moves[i] + " ")
    robot.move(make_string(ret)[0:-1])
    if bt:
        robot.change_mode()
def scramble():
    robot.lcd_write("mixing", "")
    time.sleep(1)
    moves = ["U", "U'", "U2", "D", "D'", "D2", "R", "R'","R2",
             "L", "L'", "L2", "F", "F'", "F2", "B", "B'", "B2"]
    scmbl = ""
    last = " "
    now = ""
    for i in range(20):
        now = random.choice(moves)
        while now[0] == last:
            now = random.choice(moves)            
        scmbl += now + " "
        last = now[0]
    return scmbl
def make_string(l):
    res = ""
    for i in l:
        res += i
    return res    
if __name__ == '__main__':

    t = threading.Thread(target = app_start)
    t.start()
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(True)
    robo = robot()
    robot.lcd_write("plug, and", "press any")
    robot.wait_for_button()
    robot.lcd_write("we recommend", "a calibration")
    time.sleep(2)
    while True:
        robot.lcd_write("do you want one?", "y  n")
        b = robot.wait_for_button()
        if b == 1:        
            calibrate()
            break
        elif b == 2:
            time.sleep(0.5)
            break
    while True:
        if robot.is_solved():
            robot.lcd_write("Menu:", "mx clbt stop  bt")
        else:
            robot.lcd_write("Menu:", "mx clbt solve bt")            
        b = robot.wait_for_button()
        if b == 1:
            s = scramble()
            print(s)
            robot.move(s)
        elif b == 2:
            calibrate()
        elif b == 3:
            if robot.is_solved():
                stop = False
                while True:
                    robot.lcd_write("are you sure?", "y  n")
                    b1 = robot.wait_for_button()
                    if b1 == 1:
                        stop = True
                        break
                    elif b1 == 2:
                        time.sleep(0.5)
                        break
                if stop:
                    break
            else:
                robot.lcd_write("solving", "")
                time.sleep(1)
                robot.move(kociemba.solve(robot.c.translate()))                    
        elif b == 4 or b == 5:
            robot.lcd_write("one sec", "")
            time.sleep(1)
            robot.change_mode()
            
            
    time.sleep(1)
    robot.lcd_write("unplug, and", "press any")
    robot.wait_for_button()
    robot.lcd.lcd_clear()
    robot.lcd.backlight_on(False)
    GPIO.cleanup()
    print("i cleaned up")
