# http://www.toptechboy.com/tag/adafruit-ultimate-gps/
import serial
#import Adafruit_BBIO.UART as UART
from time import sleep
#UART.setup("UART1")
ser=serial.Serial('/dev/GPS_ultimate',9600)
class GPS:
    def __init__(self):
        #This sets up variables for useful commands.
        #This set is used to set the rate the GPS reports
        UPDATE_10_sec=  "$PMTK220,10000*2F\r\n" #Update Every 10 Seconds
        UPDATE_5_sec=  "$PMTK220,5000*1B\r\n"   #Update Every 5 Seconds  
        UPDATE_1_sec=  "$PMTK220,1000*1F\r\n"   #Update Every One Second
        UPDATE_200_msec=  "$PMTK220,200*2C\r\n" #Update Every 200 Milliseconds
        UPDATE_100_msec=  "$PMTK220,100*2F\r\n" #Update Every 100 Milliseconds
        #This set is used to set the rate the GPS takes measurements
        MEAS_10_sec = "$PMTK300,10000,0,0,0,0*2C\r\n" #Measure every 10 seconds
        MEAS_5_sec = "$PMTK300,5000,0,0,0,0*18\r\n"   #Measure every 5 seconds
        MEAS_1_sec = "$PMTK300,1000,0,0,0,0*1C\r\n"   #Measure once a second
        MEAS_200_msec= "$PMTK300,200,0,0,0,0*2F\r\n"  #Meaure 5 times a second
        MEAS_100_msec= "$PMTK300,100,0,0,0,0*2\r\n"  #Meaure 10 times a second
        #Set the Baud Rate of GPS
        BAUD_57600 = "$PMTK251,57600*2C\r\n"          #Set Baud Rate at 57600
        BAUD_9600 ="$PMTK251,9600*17\r\n"             #Set 9600 Baud Rate
        ANTENNA = "$PGCMD,33,1*6C\r\n" #Turn ON antenna info
        #Commands for which NMEA Sentences are sent
        ser.write(ANTENNA)
        sleep(1)
        ser.flushInput()
        ser.flushInput()
        print "GPS Antenna Info should be seen. PGTOP,11,x where x=1(problem), 2(internal), 3(external)"

myGPS=GPS()
sleep(1)
ser.close()
