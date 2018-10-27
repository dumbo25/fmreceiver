#!/usr/bin/env python

#########################
#
# radio.py uses an Si4703 to tune to an FM radio station
#
# run using:
#    $ python3 radio.py
#
# radio.py was tested on a Raspberry Pi 3 model B+ running
# raspbian stretch
#
# currently, the implemnation is limited to using earbuds
#
# with raspi-config setup the following:
#    keyboard, timezone, locale
#    passwd, hostname
#    enable ssh, i2c
#    resize filesystem
#    reboot
#
# stretch comes with smbus, wiringPi and i2cdetect installed by default
#
#
# sudo nano /etc/modules
# i2c-dev
#
# and then reboot
#
#########################

#########################
#
# An Si4703 breakout board is connected to a Raspberry Pi 3
# as follows:
#
#    Si4703         Raspberry Pi 3
#    Pin Name       Pin Name
#    1   3.3v       1   3.3v
#    2   Ground     9   Ground
#    3   SDA/SDIO   3   I2C SDA (GPIO2)
#    4   SCLK       5   I2C SCL (GPIO3)
#    6   RST        36  GPIO16
#
# Note: there are multiple Si4703 boards and pin outs differ
#
#########################

#########################
#
# The original script is from:
#    Author: KansasCoder
#    Source: https://www.raspberrypi.org/forums/viewtopic.php?t=28920
#            Fri Dec 20, 2013 9:16 pm
#
#    PiFlyer found a way to flip back to alt0 mode
#
#########################

import RPi.GPIO as GPIO
import smbus
import time
import datetime
import subprocess


#########################
# Global Constants
#   BCM pin numbers
RST = 16
SDA = 2

#   Register Descriptions
DEVICEID = 0x00
CHIPID = 0x01
POWERCFG = 0x02
CHANNEL = 0x03
SYSCONFIG1 = 0x04
SYSCONFIG2 = 0x05
SYSCONFIG3 = 0x06
OSCILLATOR = 0x07
STATUSRSSI = 0x0A
READCHAN = 0x0B
RDSA = 0x0C
RDSB = 0x0D
RDSC = 0x0E
RDSD = 0x0F

#   Si4703 Address
#     Need to find the address of the Si4703
#     This is a bit complicated, because the output won't show
#     correctly until it works. The command to run is:
#
#       $ i2cdetect -y 1
SI4703_Address = 0x10

# FM stations are specified without the dot, so 94.7 is 947
DefaultStation = 947

#########################
# Global Variabless

fileLog = open('/home/pi/radio/radio.log', 'w+')

#   what is this used for ???
z = "000000000000000"

#   create #create 16 registers for SI4703
reg = [0] * 16

#   create list to write registers
#   only need to write registers 2-7 and since first byte is in the write
#   command then only need 11 bytes to write
writereg = [0] * 11

#   read 32 bytes
readreg = [0] * 32

# My favorite stations in Austin, TX
FavoriteStations=[937, 947, 955, 1023, 1035]

#########################
# Log messages should be time stamped
def timeStamp():
    t = time.time()
    s = datetime.datetime.fromtimestamp(t).strftime('%Y/%m/%d %H:%M:%S - ')
    return s

# Write messages in a standard format
def printMsg(s):
    fileLog.write(timeStamp() + s + "\n")

def write_registers():
    # starts writing at register 2
    # but first byte is in the i2c write command
    global writereg
    global reg
    global readreg
    cmd, writereg[0] = divmod(reg[2], 1<<8)
    writereg[1], writereg[2] = divmod(reg[3], 1<<8)
    writereg[3], writereg[4] = divmod(reg[4], 1<<8)
    writereg[5], writereg[6] = divmod(reg[5], 1<<8)
    writereg[7], writereg[8] = divmod(reg[6], 1<<8)
    writereg[9], writereg[10] = divmod(reg[7], 1<<8)
    w6 = i2c.write_i2c_block_data(SI4703_Address, cmd, writereg)
    readreg[16] = cmd #readreg
    read_registers()
    return

def read_registers():
    global readreg
    global reg
    readreg = i2c.read_i2c_block_data(SI4703_Address, readreg[16], 32)
    reg[10] = readreg[0] * 256 + readreg[1]
    reg[11] = readreg[2] * 256 + readreg[3]
    reg[12] = readreg[4] * 256 + readreg[5]
    reg[13] = readreg[6] * 256 + readreg[7]
    reg[14] = readreg[8] * 256 + readreg[9]
    reg[15] = readreg[10] * 256 + readreg[11]
    reg[0] = readreg[12] * 256 + readreg[13]
    reg[1] = readreg[14] * 256 + readreg[15]
    reg[2] = readreg[16] * 256 + readreg[17]
    reg[3] = readreg[18] * 256 + readreg[19]
    reg[4] = readreg[20] * 256 + readreg[21]
    reg[5] = readreg[22] * 256 + readreg[23]
    reg[6] = readreg[24] * 256 + readreg[25]
    reg[7] = readreg[26] * 256 + readreg[27]
    reg[8] = readreg[28] * 256 + readreg[29]
    reg[9] = readreg[30] * 256 + readreg[31]
    return

def getchannel():
    read_registers()
    channel = reg[READCHAN] & 0x03FF
    channel *= 2
    channel += 875
    return channel

def changechannel(newchannel):
    print("changechannel")
    print("newchannel = " + str(newchannel))
    c = str(float(newchannel) / 10.0)
    if newchannel < 878 or newchannel > 1080:
        print("  invalid channel " + c)
        return
    global reg
    newchannel *= 10
    newchannel -= 8750
    newchannel = int (newchannel / 20)
    print("newchannel - I think above needs to be an int")
    print(newchannel)
    read_registers()
    reg[CHANNEL] &= 0xFE00;     # Clear out the channel bits
    reg[CHANNEL] |= newchannel; # Mask in the new channel
    reg[CHANNEL] |= (1<<15);    # Set the TUNE bit to start
    write_registers()
    time.sleep(1)
    # Try ten times and then fail
    for i in range(0, 9):
        read_registers()
        time.sleep(1)
        if ((reg[STATUSRSSI] & (1<<14)) != 0):
            reg[CHANNEL] &= ~(1<<15)
            write_registers()
            return
    print("  no signal detected for " + c)
    return

def setVolume(volume):
    global reg
    if volume > 15:
        volume = 15
    if volume < 0:
        volume = 0
    read_registers()
    reg[SYSCONFIG2] &= 0xFFF0   # Clear volume bits
    reg[SYSCONFIG2] = volume # Set volume to lowest
    write_registers()
    return

def init():
    # Use BCM pin numbering
    GPIO.setmode(GPIO.BCM)
    # Disable warning messages
    GPIO.setwarnings(False)

    # Reset pin on Si4703, and BCM 23 on RPi
    GPIO.setup(RST, GPIO.OUT)
    # SDA or SDIO on Raspberry Pi 3 and same on Si4703
    GPIO.setup(SDA, GPIO.OUT)

    # Temporarily need SDA pin to put SI4703 into 2 wire mode (I2C)
    # The si4703 will not show up in i2cdetect until
    GPIO.output(SDA, GPIO.LOW)
    time.sleep(.1)

    # Transitioning the reset pin from low to high
    # completes putting the Si4703 in 2 wire mode
    GPIO.output(RST, GPIO.LOW)
    time.sleep(.1)
    GPIO.output(RST, GPIO.HIGH)
    time.sleep(.1)

    # Execute a gpio command to restore the SDA pin back to its 
    # original i2c SDA line
    #   '-g' causes pin numbers to be BCM
    #   'mode' is the option used to select the mode of the pin
    #   'alt0' is the alternate pin mode code for i2c
    subprocess.check_output(['gpio', '-g', 'mode', str(SDA), 'alt0'])

    read_registers()
    reg[OSCILLATOR] = 0x8100
    write_registers()
    time.sleep(1)

    read_registers()
    reg[POWERCFG] = 0x4001 #Enable the Radio IC and turn off muted
    write_registers()
    time.sleep(.1)

    read_registers()
    reg[SYSCONFIG1] |= (1<<12) # Enable RDS
    reg[SYSCONFIG2] &= 0xFFF0; # Clear volume bits
    reg[SYSCONFIG2] = 0x0000;  # Set volume to lowest
    reg[SYSCONFIG3] = 0x0100;  # Set extended volume range (too loud for me without this)
    write_registers()
    return

def seek(direction):
    print("in seek")
    read_registers()
    reg[POWERCFG] |= (1<<10 )
    if direction == 0:
        reg[POWERCFG] &= ~(1<<1)
    else:
        reg[POWERCFG] |= (1<<9)
    reg[POWERCFG] |= (1<<8)
    write_registers()
    # needs to try 10 times and quit
    while 1:
        read_registers()
        if ((reg[STATUSRSSI] & (1<<14)) != 0):
            break
    print("Trying Station ")
    print(str(float(float(getchannel())/float(10))))
    read_registers()
    valuesfbl = reg[STATUSRSSI] & (1<<13)
    reg[POWERCFG] &= ~(1<<8)
    write_registers()
    return

def printMenu():
    print ("\nCurrent FM station = ", str(float(float(getchannel())/float(10))))
    print ("  =    Set FM station (=101.1)")
    print ("  0-4  Favorite FM station buttons")
    print ("  d    Seek station down")
    print ("  u    Seek station up")
    print ("  +    Increase volume")
    print ("  -    Decrease volume")
    print ("  s    Display RDS status")
    print ("  m    Display RDS message")
    print ("       Press Enter to exit")


#########################
printMsg("Starting radio")

try:

    # The Raspberry Pi 3 has two I2C busses and this uses bus 1
    # Bus 1 uses SDA.1 (BCM pin 2) and SCL.1 (BCM pin 3)
    # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
    i2c = smbus.SMBus(1)

    print("init")
    init()

    print("changechannel")
    changechannel(DefaultStation)

    print("setvolume")
    volume = 7
    setVolume(volume)

    ans = True
    while ans:
        printMenu()

        # python2 uses raw_input
        ans = input(">")
        if ans >= "0" and ans <= "4":
            i = int(ans)
            j = (FavoriteStations[i:i+1] or [DefaultStation])[0]
            changechannel(j)
        if ans == "u":
            seek (1) #1=up
        if ans == "d":
            seek (0)
        if ans == "+":
            volume += 1
            setVolume (volume)
        if ans == "-":
            volume -= 1
            setVolume (volume)
        if ans != "" and ans[0] == "=":
            r = float(ans[1:])
            r *= 10
            changechannel(int(r))
        if ans == "s":
            read_registers()
            print ("\nRadio Status:")
            if reg[STATUSRSSI] & (1<<15):
                print("  RDS Available -", str(blockerr = reg[STATUSRSSI] & 0x0600 >> 9))
                if blockerr == 0:
                    print ("  No RDS Errors")
                if blockerr == 1:
                    print ("  1-2 RDS Errors")
                if blockerr == 2:
                    print ("  3-5 RDS Errors")
                if blockerr == 3:
                    print ("  6+ RDS Errors")
                r2 = z[:16 - len(bin(reg[RDSB])[2:])] + bin(reg[RDSB])[2:]
                r3 = z[:16 - len(bin(reg[RDSC])[2:])] + bin(reg[RDSC])[2:]
                r4 = z[:16 - len(bin(reg[RDSD])[2:])] + bin(reg[RDSD])[2:]
            else:
                print ("  RDS Not Available")
            if reg[STATUSRSSI] & (1<<8):
                print ("  Stereo")
            else:
                print ("  Mono")
        if ans == "m":
            msg = ""
            mi = 0
            h2=""
            h3=""
            h4=""
            wc = 0
            while 1:
                read_registers()
                if reg[STATUSRSSI] & (1<<15):
                    r2 = z[:16 - len(bin(reg[RDSB])[2:])] + bin(reg[RDSB])[2:]
                    r3 = z[:16 - len(bin(reg[RDSC])[2:])] + bin(reg[RDSC])[2:]
                    r4 = z[:16 - len(bin(reg[RDSD])[2:])] + bin(reg[RDSD])[2:]
                    if h2 != r2 or h3 != r3 or h4 != r4:
                        wc += 1
                        h2 = r2
                        h3 = r3
                        h4 = r4
                        value = int(r2[:4],2)
                        value2 = int(r2[5:-5],2)
                        if value2 == 0:
                            type = "A"
                        else:
                            type = "B"
                        code =  str(value) + type
                        if code == "2B":
                            chars = chr(int(r3[:8],2)) + chr(int(r3[9:],2)) + chr(int(r4[:8],2)) + chr(int(r4[9:],2))
                            index = int(r2[12:],2)
                            if index == 0 and mi != 0:
                                print("\nRDS MSG = " + msg + "\n")
                                break
                            if index == mi:
                                msg += chars
                                mi += 1
                        if wc == 500:
                            break

    GPIO.output(RST, GPIO.LOW)

except KeyboardInterrupt: # trap a CTRL+C keyboard interrupt
    printMsg("keyboard exception occurred")
    fileLog.close()
    GPIO.output(RST, GPIO.LOW)

except Exception as ex:
    printMsg("ERROR: an unhandled exception occurred " + str(ex))
    fileLog.close()
    GPIO.output(RST, GPIO.LOW)


