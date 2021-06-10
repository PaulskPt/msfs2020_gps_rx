                                                                                                                                                                            
"""
    FOR CIRCUITPYTHON ONLY

    Copyright 2021 Paulus H.J. Schulinck (@paulsk on discord, CircuitPython, deepdiver member)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
    and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all copies
    or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
    INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
    DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
    ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

    End of license text

    (Using firmware: Adafruit CircuitPython 7.0.0-alpha.3-15-gbc014cecb on 2021-06-04; Raspberry Pi Pico with rp2040)

    This is the main python script for the project 'msfs2020_gps_rx',
    using a Raspberry Pi Pico dev board, mounted on a Seeed Grove Shield for Pi Pico
    Attached to it via I2C: a 4x20 character Hitachi 44780 LCD with piggy-back I2C expander.
    See the README.md for details about this project.
"""
from lcd.lcd import LCD
from lcd.i2c_pcf8574_interface import I2CPCF8574Interface
import digitalio
from busio import UART
from microcontroller import pin
import sys
from time import sleep, monotonic_ns

# -------------------+
# General debug flag |
my_debug = False #   |
# -------------------+

lcd = LCD(I2CPCF8574Interface(0x27), num_rows=4, num_cols=20)
sleep(1)
uart = UART(pin.GPIO0, pin.GPIO1, baudrate=9600)

BI_LED = pin.GPIO25
led = digitalio.DigitalInOut(BI_LED)
led.direction = digitalio.Direction.OUTPUT
biLdIsOn = False

if sys.version_info > (3,):
    long = int
    
class gps_msgs:
    def __init__(self):
        self.gps =    ["",  "",  "",  "",  "", "", "", ""]

    def write(self, s):
        tp = isinstance(s,list)
        if tp == True:
            self.gps[0] = s[0] # ID
            self.gps[1] = s[1] # Lat
            self.gps[2] = s[2] # LadID N/S
            self.gps[3] = s[3] # Lon
            self.gps[4] = s[4] # LonID E/W
            self.gps[5] = s[5] # GS
            self.gps[6] = s[6] # CRS
            self.gps[7] = s[7] # Alt

    def read(self, n):
        tp = isinstance(n, type(None))
        if tp == True:
            n = 0
        if n >= 0 and n <= 7:
            return self.gps[n]
        else:
            return self.gps

    def clean(self):
            self.gps[0] = ""
            self.gps[1] = ""
            self.gps[2] = ""
            self.gps[3] = ""
            self.gps[4] = ""
            self.gps[5] = ""
            self.gps[6] = ""
            self.gps[7] = ""

encoding = 'utf-8'
lcd_maxrows = 4
lcd_rowlen = 20
lp_cnt = 0
max_lp_cnt = 99
startup = -1
loop_time = 0
t_elapsed = 0
msg_nr = 0

# Buffers
rx_buffer_len = 160
rx_buffer = bytearray(rx_buffer_len * b'\x00')

# Classes
my_msgs = gps_msgs()

def loop():
    global startup, lp_cnt, biLdIsOn, lcd, msg_nr

    TAG = "loop(): "
    chrs_rcvd = 0
    lstop = False
    lcd.clear()
    lcd.set_cursor_pos(0, 0)
    lcd.print("MSFS 2020      ")
    lcd.set_cursor_pos(1, 0)
    lcd.print("GPRMC/GPGGA data RX")
    lcd.set_cursor_pos(2, 0)
    lcd.print("Platform "+sys.platform)
    sleep(1)
    print()
    print("MSFS2020 GPS GPRMC data reception decoder sketch by Paulsk (mailto: ct7agr@live.com.pt). ")
    print("\nNumber of loops in this run: {}".format(max_lp_cnt))
    chrs_rcvd = 0
    print("........................", end="\n")

    while True:
        lp_cnt += 1
        # lcd.clear()
        print("\nStart of loop {}".format(lp_cnt), end="\n")

        if startup == -1:
            uart.reset_input_buffer()
            lcd.set_cursor_pos(3, 0)
            lcd.print("About to receive...")
        wait_cnt = 0
        chrs_rcvd = ck_uart()

        if chrs_rcvd > 0:
            lResult = split_types()
            if lResult == True:
                msg_nr += 1
                lcd_pr_msgs()
                chrs_rcvd = 0
                #sleep(0.1)

                if msg_nr >= max_lp_cnt:
                    pass
                if lstop == True:
                    if biLdIsOn:
                        led.value = 0
                        biLdIsOn = False
                else:
                    if startup == -1:
                        print("Waiting for serial com line to become available...")
                        startup = 0
                print("End of loop {}".format(lp_cnt), end="\n")
                print("........................", end="\n")
                if msg_nr >= max_lp_cnt:
                    msg_nr = 0
                    lp_cnt = 0
        else:
            pass
        if lstop == True:
            break

    #sleep(0.5)
    return True

def ck_uart():
    global rx_buffer, loop_time
    nr_bytes = i = 0
    delay_ms = 0.2
    while True:
        nr_bytes = uart.readinto(rx_buffer)
        #print("ck_uart(): rcvd data: {}".format(rx_buffer),end="\n")
        loop_time = monotonic_ns()
        if not nr_bytes:
            sleep(delay_ms)
            continue
        if nr_bytes > 1:
            return nr_bytes
        elif nr_bytes == 1:
            if rx_buffer[0] == b'\x00':
                sleep(delay_ms)
                i += 1
                if i % 1000 == 0:
                    print("Waiting for uart line to become ready")
        else:
            empty_buffer()
            sleep(delay_ms)
            continue
    #return nr_bytes

def split_types():
    global rx_buffer, my_msgs
    TAG = "split_types(): "
    lResult = True
    sRMC = sGGA = None
    nr_RMC_msgs = nr_GGA_msgs = 0
    mRMC_ID = nRMC_EOT = nGGA_ID = nGGA_EOT = 0
    lGPRMC_go = lGPGGA_go = False
    msg_lst = []
    rmc_lst = []
    gga_lst = []
    rmc_msg_lst = []
    gga_msg_lst = []

    nRMC_ID = bfr_fnd(36) # find $
    if nRMC_ID >= 0:
        nRMC_EOT = bfr_fnd(13)
        if nRMC_EOT > 0:
            sRMC = rx_buffer.decode(encoding)[nRMC_ID:nRMC_EOT+1]
            rx_buffer = rx_buffer[nRMC_EOT+1:]
            nGGA_ID = bfr_fnd(36)
            if nGGA_ID >=0:
                nGGA_EOT = bfr_fnd(13)
                if nGGA_EOT > 0:
                    sGGA = rx_buffer.decode(encoding)[nGGA_ID:nGGA_EOT+1]
    lIsStr = isinstance(sRMC,str)
    if lIsStr:
        rmc_msg_lst = sRMC.split(",")
        nr_RMC_msgs = len(rmc_msg_lst)
        if nr_RMC_msgs == 12:
            lGPRMC_go = True

    lIsStr = isinstance(sGGA,str)
    if lIsStr:
        nGGA_ID = sGGA.find("$")
        if nGGA_ID > 0:
            pass
        else:
            gga_msg_lst = sGGA.split(",")
            nr_GGA_msgs = len(gga_msg_lst)
            if nr_GGA_msgs == 15:
                t_alt = float(gga_msg_lst[9])
                p = isinstance(t_alt,float)
                if p == True:
                    t_alt = round(int(float(gga_msg_lst[9])) * 3.2808)
                else:
                    t_alt = 0
                lGPGGA_go = True

        if lGPRMC_go == True:
            rmc_lst = [rmc_msg_lst[0], rmc_msg_lst[3], rmc_msg_lst[4], rmc_msg_lst[5], rmc_msg_lst[6], rmc_msg_lst[7], rmc_msg_lst[8]]
            if lGPGGA_go == True:
                rmc_lst.append(str(t_alt))
            my_msgs.write(rmc_lst)
            #print(TAG+"cross-check: my_msgs class data contents: {}".format(my_msgs.read(9)), end="\n")

    empty_buffer() # not emptying the buffer here causes an error in lcd_pr_msgs() !!!
    if lGPRMC_go == False and lGPGGA_go == False:
        lResult = False

    return lResult

def bfr_fnd(ck_val):
    global rx_buffer
    c = ""
    n = -1
    p = isinstance(ck_val, type(None))
    if p == True:
        return -1
    else:
        c = chr(ck_val)

        try:
            t_bfr = rx_buffer.decode(encoding)
            n = t_bfr.find(c)
        except UnicodeError:
            print("bfr_fnd(): Unicode Error. Check baudrate.", end="\n")

        return n

def led_toggle():
    global biLdIsOn

    if biLdIsOn:
        led.value = 0
        biLdIsOn = False
    else:
        led.value = 1
        biLdIsOn = True

def empty_buffer():
    global rx_buffer, rx_buffer_len
    rx_buffer = bytearray(rx_buffer_len * b'\x00')

def lcd_pr_msgs():
    global startup, loop_time, t_elapsed, msg_nr, my_msgs, lcd_maxrows
    TAG = "lcd_pr_msgs(): "
    dp = 0
    msg_itm = 0
    lcd_vpos = 0
    s = ""

    lat   =  1
    latID =  2
    lon   =  3
    lonID =  4
    gs    =  5
    crs   =  6
    alt   =  7

    if startup == -1:
        lcd.clear()
    lcd.set_cursor_pos(0,18)
    lcd.print("{:0>2d}".format(msg_nr))
    lcd_vpos = 0
    itms_lst = [lat, lon, gs, crs]
    led_toggle()
    for msg_itm in itms_lst:
        if msg_itm == lat:
            lat_v = my_msgs.read(lat)
            dp = lat_v.find(".")
            if dp >= 0:
                if dp == 4:
                    s = "{: >2s}\xDF{:0>2s}\'{:0>2s}.{:0>2s}\"".format(lat_v[:2], lat_v[2:4], lat_v[5:7],lat_v[7:])
                elif dp == 3:
                    s = "{: >2s}\xDF{:0>2s}\'{:0>2s}.{:0>2s}\"".format(lat_v[:1], lat_v[1:3], lat_v[4:6],lat_v[6:])
            s = my_msgs.read(latID) + "    " + s
        if msg_itm == lon:
            lon_v = my_msgs.read(lon)
            dp = lon_v.find(".")
            if dp >= 0:
                if dp == 5:
                    s = "{: >2s}\xDF{:0>2s}\'{:0>2s}.{:0>2s}\"".format(lon_v[:3], lon_v[3:5], lon_v[6:8], lon_v[8:])
                elif dp == 4:
                    s = "{: >2s}\xDF{:0>2s}\'{:0>2s}.{:0>2s}\" ".format(lon_v[:2], lon_v[2:4], lon_v[5:7], lon_v[7:])
            s = my_msgs.read(lonID) + "   " + s
        if msg_itm == gs:
            s = "GS  {: >3d} ALT {: >5d} FT".format(round(int(float(my_msgs.read(gs)))), \
                round(int(float(my_msgs.read(alt)))))
        if msg_itm == crs:
            s = "CRS {:0>3d} DEGS     ".format(round(int(float(my_msgs.read(crs)))))
        lcd.set_cursor_pos(lcd_vpos, 0)
        lcd.print(s)

        lcd_vpos += 1
    t_elapsed = (((monotonic_ns() - loop_time) + 500000)// 1000000)
    print(TAG+"Duration rx -> lcd: {} mSecs".format(t_elapsed), end="\n")

    my_msgs.clean()

    lcd.set_cursor_pos(3,19)

    lcd_vpos += 1
    if lcd_vpos >= lcd_maxrows:
        lcd_vpos = 0

    led_toggle()

def main():
    global my_debug
    lResult = True
    cnt = 0

    while True:
        lcd.clear()
        sleep(2)
        lcd.set_cursor_pos(0, 0)
        lcd.print("FSUIPC7 GPS RX ")
        lcd.set_cursor_pos(1, 0)
        lcd.print("for MSFS2020   ")
        sleep(2)
        lcd.set_cursor_pos(1, 0)
        lcd.print("via serial     ")
        sleep(2)
        lcd.clear()

        if cnt == 0:
            lResult = loop()
            cnt += 1
            if lResult == False:
                if my_debug == True:
                    print("loop(): setup returned with: \"{}\" in line nr: {}".format(lResult))
            break

    cnt = 0
    while True:
        cnt += 1
        if cnt >= 100:
            cnt = 0
            led_toggle()

if __name__ == '__main__':
    main()
