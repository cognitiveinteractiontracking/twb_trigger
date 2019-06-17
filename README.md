# TWB Trigger Projekt

Das Ziel dieses Projektes ist es, die Kameras in der TWB mithilfe eines Raspberry Pi zu synchronisieren. 
Dazu wird ein periodischer Timer für das Triggern der Kameras genutzt, um an den GPIOs des Raspberry Pi ein Signal zu erzeugen.
Außerdem läuft auf dem Raspberry Pi ROS, welches es dem Pi erlaubt mit dem bestehendem System zu kommunizieren.<br />
Hinweis: Der Pegelwandler von 3,3V (GPIO Raspberry) auf 5V ist nicht nötig, da die Kameras auch mit einer Eingangsspannung von 3,3V getriggert werden können!
![TWB_Projekt](https://github.com/kevinp1993/TWB_Trigger/blob/master/Images/TWB_Trigger_Projekt.png)

## Vorbereitung
Dieser Teil befasst sich mit der Einrichtung der Hard- und Software des Raspberry Pi.

### Hardwarekomponenten
Folgende Hardwarekomponenten werden verwendet:
* Raspberry Pi 3B
* DS3231 Real-Time-Clock

### Hardware Einrichtung/Schematic
1. RTC <-> Rapsberry Pi 3B
     1. Vcc <-> Pin01: 3,3V
     2. Data <-> Pin03: GPIO02/SDA1
     3. Clock <-> Pin05: GPIO03/SCL1
     4. NC <-> Pin07: GPIO04
     5. GND <-> Pin09: GND
2. Triggerausgänge 
     1. Pin 37 Raspberry: GPIO 26
     2. Pin 35 Raspberry: GPIO 19
     3. Pin 33 Raspberry: GPIO 13
     4. Pin 31 Raspberry: GPIO 6
     

### Raspberry Pi OS und ROS Installation
Verwendet wird das [Raspbian Strech with desktop Image](https://www.raspberrypi.org/downloads/) mit der Kernel version 4.14.
Das Image wird auf eine SD-Card geschrieben und im Anschluss in den Raspberry Pi gesteckt. <br />
Nachdem das Filesystem expandiert wurde, kann sich mit folgenden Zugangsdaten eingeloggt werden:
```
username: pi
pw: raspberry
```
Verbinde dich nun mit einem internetfähigen Netzwerk<br />

Installiere Updates:
```
sudo apt-get update
sudo apt-get upgrade
```
Nun kann ROS eingerichtet werden. Hierzu findet sich im ROS Wiki eine [Anleitung](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi) zum Einrichten von ROS Kinetic.
 
Für einen Betrieb des Raspberry Pi ohne Bildschrim ist eine SSH Verbindung zu einem PC sinnvoll.
Hierzu muss zunächst über raspi-config die SSH-Verbindung aktiviert werden am Pi.
Nach der Verbindung des Pis mit einem Ethernet-Kabel kann die IP-Adresse des Pi mit folgendem Kommando eingesehen werden:
```
ifconfig
```
Hiernach ist eine lokale Verbindung am PC einzurichten. Mit folgendem Befehl kann nun eine SSH-Verbindung aufgebaut werden:
```
ssh -X -Y pi@IP-des-Raspberry
```
Ebenso ist die Nutzung eines Terminals hilfreich. Nach dem Start der SSH-Verbindung kann dies mit:
```
x-terminal-emulator &
```
gestartet werden.


### Einrichtung der RTC
Folgende Treiber müssen nachträglich in die Datei /etc/modules eingetragen werden:
```
i2c-bcm2708
i2c_dev
```
Dannach sollten folgende Pakete nachträglich installiert/aktuallisiert werden:
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
Um das Modul zu aktivieren, muss folgender Befehl ausgeführt werden:
```
echo ds3231 0x68 | sudo tee /sys/class/i2c-adapter/i2c-1/new_device
```

Um die hw-clock beim Booten einzubinden, folgende Zeile vor exit 0 an die Datei */etc/rc.local* hängen:
```
echo ds3231 0x68 > /sys/class/i2c-adapter/i2c-1/new_device
```
Danach sollte das System rebootet werden.<br />

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
Um die Systemzeit beim Booten auf die RTC zu setzen, werden folgende Zeilen an das Ende von */etc/rc.local* gehängt (vor exit 0):
```
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
Um eine automatische Aktivierung beim Booten zu erhalten sind folgende Kommandos hilfreich:
```
sudo systemctl enable pigpiod
sudo systemctl start pigpiod 
```

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
Zunächst muss das Projekt im catkin workspace gebaut und gesourcet werden:
```
catkin_make
source catkin_ws/devel/setup.bash
```
Nun wird eine roscore gestartet
```
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


### Langzeittest an Kamera
Zu Testzwecken wurde der Trigger an den realen Kameras getestet und dabei die Zeitstempel der Kameras aufgezeichnet. Aus den Zeitstempeln wurden die Frequenzen extrahiert. Diese wurden mittels Mittelwert, Varianz, Histogramm und Ausreißer hin ausgewertet. Folgendes Histogramm zeigt alle relevanten Statistiken:

![TWB_Projekt](https://github.com/kevinp1993/TWB_Trigger/blob/master/analyseData/Versuch2/Histogram_cam1_25Hz_Freq_header.png)


### Hinweise
Bei der Analyse des Jitters ist aufgefallen, dass trotz minimaler Abweichung der Abtastperiode (siehe Plot 2), der Jitter linear über der Zeit immer größer wird (siehe Plot 1). Die Berechnung des Jitters erfolgt über die Differenz zwischem gemessenen und erwarteten Zeitstempel. Dies kommt dadurch zustande, dass die interne clock des Raspberrys für die Zeitmessung einen Drift besitzt. Dieser muss für ein optimales Triggern noch entfernt werden.

![TWB_Projekt](https://github.com/kevinp1993/TWB_Trigger/blob/master/analyseData/Versuch2/Jitter_and_dt_header.png)



## Entwickler/Autoren/Verantwortliche

* [kevinp1993](https://github.com/kevinp1993)
* [tik0](https://github.com/tik0)
