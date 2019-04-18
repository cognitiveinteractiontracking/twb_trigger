# TWB Trigger Projekt

Das Ziel dieses Projektes ist es, die Kameras in der TWB mithilfe eines Raspberry Pi zu synchronisieren. 
Dazu wird ein periodischer Timer für das Triggern der Kameras genutzt, um an den GPIOs des Raspberry Pi ein Signal zu erzeugen.
Da die GPIOs des Raspberry Pi einen Output von 3,3V generieren, werden diese mit einem Pegelwandler auf 5V gewandelt.
Außerdem läuft auf dem Raspberry Pi ROS, welches es dem Pi erlaubt mit dem bestehendem System zu kommunizieren.<br />
![TWB_Projekt](https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/TWB_Trigger_Projekt.png)

## Vorbereitung
Dieser Teil befasst sich mit der Einrichtung der Hard- und Software des Raspberry Pi.

### Hardwarekomponenten
Folgende Hardwarekomponenten werden verwendet:
* Raspberry Pi 3B
* Adafruit TXB0108 Level Shifter
* DS3231 Real-Time-Clock

### Hardware Einrichtung/Schematic
1. RTC <-> Rapsberry Pi 3B
     1. Vcc <-> Pin01: 3,3V
     2. Data <-> Pin03: GPIO02/SDA1
     3. Clock <-> Pin05: GPIO03/SCL1
     4. NC <-> Pin07: GPIO04
     5. GND <-> Pin09: GND
2. Pegelwandler
     1. VCCA <-> Pin17: 3,3V
     2. VCCB <-> Pin02: 5V
     3. GND <-> Pin39: GND
     4. A1 <-> Pin31: GPIO06
     5. A2 <-> Pin33: GPIO13
     6. A3 <-> Pin35: GPIO19
     7. A4 <-> Pin37: GPIO26
     8. B1-B4: Trigger Outputs
     
TODO: Image

### Raspberry Pi OS und ROS Installation
Verwendet wird das [ubiquityrobotics Image](https://ubiquity-pi-image.sfo2.cdn.digitaloceanspaces.com/2019-02-19-ubiquity-xenial-lxde-raspberry-pi.img.xz),
welches auf Ubuntu 16.04 basiert und ROS vorinstalliert hat. Verwendet wird die Version *2019-02-19-ubiquity-xenial-lxde*<br />
Das Image wird auf eine SD-Card geschrieben und im Anschluss mit dem Raspberry Pi eingerichtet.

### Einrichtung der RTC
Folgende Pakete müssen nachträglich installiert werden:
```
sudo apt-get install i2c-tools
sudo apt-get install ntp
```
Für die Aktivierung der I2C Schnittstelle rufe die Konfiguration auf:
```
sudo raspi-config
```
Unter Interfacing *Options --> I2C --> Enable* wird die Schnittstelle freigeschaltet.<br />
Für das Überprüfen, ob der RTC erkannt wurde folgenden Befehl ausführen:
```
sudo i2cdetect -y 1
```
Hier sollte die Adresse 0x68 vom RTC zu sehen sein.<br />
Um die RTC zu setzen, muss der Raspberry Pi mit dem Internet verbunden sein bzw. Zugriff auf einem ntp Server besitzen.
Die RTC wird dann mit folgendem Befehl mit der aktuellen Systemzeit beschrieben:
```
sudo hwclock -w
```
Die aktuelle RTC Zeit sollte nun der Systemzeit entsprechen. Zu Prüfen ist dies mit:
```
sudo hwclock -r
```
Im Anschluss kann die Systemzeit mit der RTC gesetzt werden:
```
sudo hwclock -s
```
Um die Systemzeit beim Booten auf die RTC zu setzen, werden folgende Zeilen an das Ende von */etc/rc.local* gehängt:
```
echo ds1307 0x68 > /sys/class/i2c-adapter/i2c-1/new_device
hwclock -s
```
Weitere nützliche Kommandos:
* Systemzeit vom NTP Server holen:
```
sudo nptd -g -q 
```
* Systemzeit und RTC Zeit vergleichen:
```
sudo timedatectl 
```
* Systemzeit anzeigen:
```
date 
```

### PIGPIO Installation
[PIGPIO](http://abyz.me.uk/rpi/pigpio/pdif2.html) ist eine C Library, welche Funktionen zum parallelen/synchronen 
Setzen von GPIOs beinhaltet. Die Anleitung für den Downlaod und die Library sind auf deren Website hinterlegt.
Verwendet wurde das pigpiod Interface.<br />
Nach der Installation kann das PIGPIO Deamon mit folgendem Kommando gestartet werden:
```
sudo pigpiod
```
Um eine automatische Aktivierung zu erhalten kann der Kommando auch in die .bashrc im Homeverzeichnis geschrieben werden.


## Testen und Evaluation der PIGPIO Library und des periodischen Timers
Für das Testen der Trigger Funktion wurde eine C-Programm [*InterruptTrigger*](https://github.com/kevinp1993/TWB_Trigger/blob/master/InterruptTrigger/InterruptTrigger.c) geschrieben. Um die Frequenz in Hz und die Weite des Impulses in % anzupassen, können folgende Variablen im Quellcode geändert werden:
```c
int freq = 30;
double width = 5;
```
Das Setzen bzw. Bereinigen von mehreren GPIOs gleichzeitig efolgt über die Funktionen:
```c
int set_bank_1(int pi, uint32_t bitmaske);
int clear_bank_1(int pi, uint32_t bitmaske);
```
Der Interrupt Timer aus der *signal.h* Library benötigt eine Interrupt Routine Funktionshandler *alarmWakeup*	und eine Periodendauer in us, wann der Timer einen Interrupt auslösen soll:
```c
signal(SIGALRM, alarmWakeup);
ualarm(timer_periode,timer_periode);
```
<br />
Die Ausgänge der GPIOs wurden mithilfe eines Oszilloskop beobachtet und die Signale auf Amplitude, Frequenzabweichung, Impulsweite und Synchronität zwischen den Signalen hin geprüft.<br />
Zunächst einmal die vier Ausgänge bei unterschiedlichem großem Zeitfenster:<br />

<p float="left">
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_1/01_Signal_10ms.png" width="400" />
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_1/01_Signal_20ms.png" width="400" /> 
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_1/01_Signal_100ms.png" width="400" />
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_1/01_Signal_400ms.png" width="400" />
</p> <br /> <br />

Die Pulsweite lässt sich ebenfalls anpassen (10%, 30%, 50% und 70% als Beispiel aufgeführt):<br />

<p float="left">
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_1/02_Signal_10P.png" width="400" />
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_1/02_Signal_30P.png" width="400" /> 
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_1/02_Signal_50P.png" width="400" />
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_1/02_Signal_70P.png" width="400" />
</p> <br /> <br />

Bei gegebener Freuqeunz von 30Hz (entspricht einer Periodendauer von 33,33ms) wird diese auch an den Ausgängen gemessen: <br />

![Frequenz](https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_1/03_Periode.png) <br /> <br />

Der Jitter beträgt ungefähr 150us bei der abfallenden Flanke:
![Jitter](https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_1/04_Jitter.png) <br /> <br />

## Realisierung in der ROS Umgebung
Im [catkin workspace](https://github.com/kevinp1993/TWB_Trigger/tree/master/catkin_ws) wurde das Paket [cam_trigger](https://github.com/kevinp1993/TWB_Trigger/tree/master/catkin_ws/src/cam_trigger) erstellt. Hier befindet sich zum einen die [cfg-File](https://github.com/kevinp1993/TWB_Trigger/blob/master/catkin_ws/src/cam_trigger/cfg/cam_trigger.cfg) für die [Dynamic Reconfigure](http://wiki.ros.org/dynamic_reconfigure/Tutorials) GUI, als auch der [Trigger Node](https://github.com/kevinp1993/TWB_Trigger/blob/master/catkin_ws/src/cam_trigger/src/cam_trigger_publisher_node.cpp). <br />
Über die GUI sollen zur Laufzeit des Triggers Frequenz und Pulslänge einstellbar sein. Ebenso kann das sowohl der gesamte Triggerprozess über System, als auch einzelne Triggers für die Kamera ein- und ausgeschaltet werden. <br />
Im Node werden periodische Clock-Message bei den Triggerzeitpunkten gepublished. Da das Publishen der Message auch Zeit benötigt, wird diese Zeit beim sleep berücksichtigt, um abweichende Periodendauern zu verhindern. <br />

### Starten des Triggers und der GUI
Zunächst muss das Projekt im catkin workspace gesourcet und gebaut werden:
```
source catkin_ws/devel/setup.bash
catkin_make
```
Da das ubiquityrobotics Image beim booten automatisch den roscore startet, muss dies nicht mehr manuell gemacht werden. Um den roscore dennoch zu beenden und neuzustarten, müssen folgende Befehle eingegeben werden:
```
killall -9 roscore
roscore
```
Der Node wird folgendermaßen gestartet:
```
rosrun cam_trigger cam_trigger_publisher
```
Die Dynamic Reconfigure GUI wird im Anschluss gestartet:
```
rosrun rqt_reconfigure rqt_reconfigure
```

### Test und Evaluation
Auch die Realisierung in ROS wurde mithilfe eines Oszilloskops getestet. Hier die Ergebnisse: <br />
1. Das System wird mit der Auswahl von System gestartet. Der Defaultwert der Frequenz liegt bei 30Hz, für die Pulsweite bei 20% und alle Kameras sind ausgewählt:<br />
<p float="left">
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_2/01_Signal_GUI.png" width="400" />
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_2/01_Signal.png" width="400" /> 
</p> <br /> <br />

2. Die Kameras lassen sich ein- und ausschalten:
<p float="left">
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_2/02_Signal_cam_GUI.png" width="400" />
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_2/02_Signal_cam.png" width="400" /> 
</p> <br /> <br />

3. Die Frequenz und die Pulsweite der Signal lassen sich einstellen
<p float="left">
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_2/03_Signal_30_40_GUI.png" width="400" />
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_2/03_Signal_30_40.png" width="400" />
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_2/04_Signal_50_50_GUI.png" width="400" /> 
  <img src="https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/Oscilloscope_Images_2/04_Signal_50_50.png" width="400" /> 
</p> <br /> <br />

## Entwickler/Autoren/Verantwortliche

* [kevinp1993](https://github.com/kevinp1993)
* [tik0](https://github.com/tik0)
