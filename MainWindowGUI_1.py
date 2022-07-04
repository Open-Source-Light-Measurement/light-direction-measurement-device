from PyQt5.QtWidgets import QApplication, QMainWindow
import threading
from PyQt5 import uic

import sys

#Wichtig vor Start: sudo pigpiod


import RPi.GPIO as GPIO
import pigpio
import time
import random
import numpy as np
import smbus
import time
import csv
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as axes3d




class Hauptfenster(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("Mainwindow.ui", self)
        
        self.pushButton.pressed.connect(self.start_button)
        self.pushButton_2.clicked.connect(self.btn2_click)
    
    global flag
    flag = False
    
    def start_button(self):
        global flag
        flag = True
        t1 = threading.Thread(target=self.btn_click)
        t1.start()
    
    def btn2_click(self):
        global flag
        flag = False

    def btn_click(self):
        global flag
        

        ListeWinkelAchse1 = []
        ListeWinkelAchse2Sensor =[]
        Eoben = []
        Eunten = []
        WinkelAchse1=[]
        WinkelAchse2=[]
        Gesamtmatrix=[]

        servoPIN = 18 # der Servomotor wurde an den GPIO Pin 18 angeschlossen
        servoPIN2 = 12




        servoPositions1 = [500,700,930,1100,1300,1500,1700,1900,2100,2300,2500]#Drehung Stab
        #servoPositions1 = [500,700,900,1100]
        AnzWinkel1=len(servoPositions1)
        servoPositions = [500,700,900,1100,1300,1500,1700,1900,2100,2300,2500]#Drehung Photometer
        #servoPositions = [500,700,900,1100]
        #0-180 Grad=500-2500
        AnzWinkel=len(servoPositions1)


        DEVICE     = 0x23 # Default device I2C address

        POWER_DOWN = 0x00 # No active state
        POWER_ON   = 0x01 # Power on
        RESET      = 0x07 # Reset data register value

        # Start measurement at 4lx resolution. Time typically 16ms.
        CONTINUOUS_LOW_RES_MODE = 0x13
        # Start measurement at 1lx resolution. Time typically 120ms
        CONTINUOUS_HIGH_RES_MODE_1 = 0x10
        # Start measurement at 0.5lx resolution. Time typically 120ms
        CONTINUOUS_HIGH_RES_MODE_2 = 0x11
        # Start measurement at 1lx resolution. Time typically 120ms
        # Device is automatically set to Power Down after measurement.
        ONE_TIME_HIGH_RES_MODE_1 = 0x20
        # Start measurement at 0.5lx resolution. Time typically 120ms
        # Device is automatically set to Power Down after measurement.
        ONE_TIME_HIGH_RES_MODE_2 = 0x21
        # Start measurement at 1lx resolution. Time typically 120ms
        # Device is automatically set to Power Down after measurement.
        ONE_TIME_LOW_RES_MODE = 0x23

        bus = smbus.SMBus(1)  # Rev 2 Pi uses 1
        bus2 = smbus.SMBus(4)  # Neuen Bus hinzugefügt


        def convertToNumber(data):
          # Simple function to convert 2 bytes of data
          # into a decimal number. Optional parameter 'decimals'
          # will round to specified number of decimal places.
          result=(data[1] + (256 * data[0])) / 1.2
          return (result)

        def convertToNumber(data2):
          # Simple function to convert 2 bytes of data
          # into a decimal number. Optional parameter 'decimals'
          # will round to specified number of decimal places.
          result2=(data2[1] + (256 * data2[0])) / 1.2
          return (result2)

        def readLight(addr=DEVICE):
          # Read data from I2C interface
          data = bus.read_i2c_block_data(addr,ONE_TIME_HIGH_RES_MODE_1)
          return convertToNumber(data)

        def readLight2(addr=DEVICE):
          # Read data from I2C interface
          data2 = bus2.read_i2c_block_data(addr,ONE_TIME_HIGH_RES_MODE_1)
          return convertToNumber(data2)

        def setServoCycle(p, position):
          pwm.set_servo_pulsewidth( servoPIN, position ) ;
          # eine Pause von 0,5 Sekunden
          time.sleep(1)
          
        def setServoCycle2(p, position):
          pwm.set_servo_pulsewidth( servoPIN2, position ) ;
          # eine Pause von 0,5 Sekunden
          time.sleep(1)
              
        # versuche

        try:
          
          pwm = pigpio.pi() 
          pwm.set_mode(servoPIN, pigpio.OUTPUT)
          pwm.set_mode(servoPIN2, pigpio.OUTPUT)
          
          
          
          GPIO.setmode(GPIO.BCM)
          # setzen des GPIO Pins als Ausgang
          #GPIO.setup(servoPIN, GPIO.OUT)
          #GPIO.setup(servoPIN2, GPIO.OUT)
          p = pwm.set_PWM_frequency( servoPIN, 50 )
          q = pwm.set_PWM_frequency( servoPIN2, 50 )
          #p.start(servoPositions[0]) # Initialisierung mit dem ersten Wert aus unserer Liste
          #q.start(servoPositions[0]) 
          # eine Endlos Schleife
          i=0
          while True:
            
            # fuer jeden Wert in der Liste, mache...
            for pos in servoPositions:
                if flag == False:
                    break
                setServoCycle(p, pos)
                for x in servoPositions1:    # setzen der Servopostion
                    if flag == False:
                        break
                    setServoCycle2(q, x)
                    lightLevel=readLight()
                    lightLevel2=readLight2()
                    WinkelAchse1=(pos-500)*(0.09)
                    WinkelAchse2=(x-500)*(0.09)
                    print("Light Level : " + format(lightLevel,'.2f') + " lx bei Winkel Achse 1: " + format(WinkelAchse1,'.2f') + " Grad und Winkel Achse 2: " + format(WinkelAchse2,'.2f') + " Grad")
                    print("Light Level Rückseite : " + format(lightLevel2,'.2f') + " lx bei Winkel Achse 1: " + format(WinkelAchse1,'.2f') + " Grad und Winkel Achse 2: " + format(WinkelAchse2,'.2f') + " Grad")
                    ListeWinkelAchse1=np.append(ListeWinkelAchse1,WinkelAchse1)
                    ListeWinkelAchse2Sensor=np.append(ListeWinkelAchse2Sensor,WinkelAchse2)
                    Eoben=np.append(Eoben,lightLevel)
                    Eunten=np.append(Eunten,lightLevel2)
                    Gesamtmatrix=[[Eoben],[Eunten],[ListeWinkelAchse1],[ListeWinkelAchse2Sensor]]
                # durchlaufen der Liste  in umgekehrter Reihenfolge
            print (Gesamtmatrix)
            rows=zip(Eoben,Eunten,ListeWinkelAchse1,ListeWinkelAchse2Sensor)
            with open('Sensordaten.csv', 'w', encoding='UTF8') as f:
                writer = csv.writer(f)
                writer.writerow(["Eoben in lx", "Eunten in lx", "Achse1 in Grad", "Achse2 in Grad"])
                for row in rows:
                    writer.writerow(row)

            array1 = np.array(Eoben)
            array2 = np.array(Eunten)
            subtracted_array = np.subtract(Eoben, Eunten)
            #subtracted_array = np.abs(subtracted_array)
            subtracted = list(subtracted_array)
            print("Subtrahiertes Array: ")
            print(subtracted)

            max_value = max(subtracted)
            max_index = subtracted.index(max_value)
            max_index = max_index + 1
            print("Max Index: ")
            print(max_index)
            print("Maximale Differenz: ")
            print(max_value)
            i+=1
            if i == 1:
                break

        #Anfahren der dominanten Lichtrichtung
            #Winkel1 - Achse 1
          Winkel1= max_index / AnzWinkel1
          Winkel1=int(Winkel1+1)
          #Winkel1=(servoPositions1[Winkel1])
          print("Winkel1: ")
          print(Winkel1)
          pwm.set_servo_pulsewidth( servoPIN, servoPositions[Winkel1-1] ) 
            
            #Winkel2
          Winkel2= max_index / AnzWinkel
          Winkel2=Winkel2-int(Winkel2)
          Faktor=1/AnzWinkel
          Winkel2=Winkel2/Faktor
          Winkel2=int(Winkel2+1)
          #Winkel2=(servoPositions[Winkel2])
          print("Winkel2: ")
          print(Winkel2)
          pwm.set_servo_pulsewidth( servoPIN2, servoPositions[Winkel2-1] )

          AnzWinkel = 11
          Eoben = []
          Eunten = []
          ListeWinkelAchse1 = []
          ListeWinkelAchse2Sensor = []

          with open('Sensordaten.csv', 'r') as csvfile:
              points = csv.reader(csvfile, delimiter=',')
              for row in points:
                  Eoben.append(row[0])
                  Eunten.append(row[1])
                  ListeWinkelAchse1.append(row[2])
                  ListeWinkelAchse2Sensor.append(row[3])

          Eoben = np.array(Eoben)
          # bei allen Variablen entferne ich die Kopfzeile
          Eoben = np.delete(Eoben, 0)
          # bei allen Variablen entferne ich jeden 11ten Wert, weil die doppelt gemessen werden
          Eoben = np.delete(Eoben, np.arange(0, Eoben.size, 11))

          Eunten = np.array(Eunten)
          Eunten = np.delete(Eunten, 0)
          Eunten = np.delete(Eunten, np.arange(0, Eunten.size, 11))

          ListeWinkelAchse1 = np.array(ListeWinkelAchse1)
          ListeWinkelAchse1 = np.delete(ListeWinkelAchse1, 0)
          ListeWinkelAchse1 = np.delete(ListeWinkelAchse1, np.arange(0, ListeWinkelAchse1.size, 11))

          ListeWinkelAchse2Sensor = np.array(ListeWinkelAchse2Sensor)
          ListeWinkelAchse2Sensor = np.delete(ListeWinkelAchse2Sensor, 0)
          ListeWinkelAchse2Sensor = np.delete(ListeWinkelAchse2Sensor, np.arange(0, ListeWinkelAchse2Sensor.size, 11))

          # Datentyp float zuweisen
          Eoben = np.asarray(Eoben, dtype=np.float64, order='C')
          Eunten = np.asarray(Eunten, dtype=np.float64, order='C')
          ListeWinkelAchse1 = np.asarray(ListeWinkelAchse1, dtype=np.float64, order='C')
          ListeWinkelAchse2Sensor = np.asarray(ListeWinkelAchse2Sensor, dtype=np.float64, order='C')
          # ------------------------

          B = []
          C = []
          Zeilen = (Eoben.size)
          print(Zeilen)

          array1 = np.array(Eoben)
          array2 = np.array(Eunten)
          subtracted_array = np.subtract(Eoben, Eunten)
          # subtracted_array = np.abs(subtracted_array)
          subtracted = list(subtracted_array)
          print("Subtrahiertes Array: ")
          print(subtracted)

          # roty
          for i in range(Zeilen):
              x = [0, 0, subtracted[i]]
              print(x)
              # Gewünschter Rotationswinkel
              Winkel = np.radians(ListeWinkelAchse1[i])
              print(Winkel)
              r = np.array(((np.cos(Winkel), 0, np.sin(Winkel)),
                            (0, 1, 0),
                            (-np.sin(Winkel), 0, np.cos(Winkel))))
              print(r)
              v = r.dot(x)
              B = np.append(B, v)
          print(B)

          B1 = B[0::3]
          B2 = B[1::3]
          B3 = B[2::3]

          # rotx
          for i in range(Zeilen):
              x = [B1[i], B2[i], B3[i]]
              # Gewünschter Rotationswinkel
              Winkel = np.radians(ListeWinkelAchse2Sensor[i])
              print(Winkel)
              r = np.array(((1, 0, 0),
                            (0, np.cos(Winkel), -np.sin(Winkel)),
                            (0, np.sin(Winkel), np.cos(Winkel))))
              print(r)
              v = r.dot(x)
              C = np.append(C, v)
          print(C)
          X = C[0::3]
          Y = C[1::3]
          Z = C[2::3]
          print(x)

          ax = plt.figure().add_subplot(projection='3d')
          plot = ax.quiver(0, 0, 0, X, Y, Z, length=0.1)
          ax.set_xlim([-5, 5])
          ax.set_ylim([-5, 5])
          ax.set_zlim([-5, 5])
          plt.show()


        # wenn das Script auf dem Terminal / der Konsole abgebrochen wird, dann...
        except KeyboardInterrupt:
          p.stop()
          # alle Pins zuruecksetzen
          GPIO.cleanup()



app= QApplication(sys.argv)
window=Hauptfenster()
window.show()
sys.exit(app.exec_())
