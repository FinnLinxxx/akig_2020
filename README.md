# akig_2020
AKIG 2020 TU Wien

![img](documents/folien/senHusky.png)


In diesem Jahr soll im Rahmen von AKIG der selbst fahrende Roboter [Huksy](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/) um Funktionen im Zusammenhang mit dem Tachymeter erweitert werden. Neben der 2D- und 3D-Aufnahme des Raumes ist in der Ingenieurgeodäsie vor allem die Bewertung der Genauigkeit und Zuverlässigkeit der erzeugten Daten von Bedeutung. Zu entwickeln ist ein maximal autonom agierendes System, welches in der Lage ist sich selbstständig in einem zu schaffenden Industriemessnetz zu orientieren und zu navigieren. Als Methode kommen für den Husky bereits implementierte [ROS-Packete](http://wiki.ros.org/Robots/Husky) wie etwa [gmapping](http://wiki.ros.org/gmapping) zum [Einsatz](https://www.youtube.com/watch?v=XQg-1DZRqQo). 


Das Tachymeter erzeugt ground-truth Daten wie sie in der Robotik allgemein nicht zur Verfügung stehen, Daher sind neue Ansätze zu entwickeln die mit Blick auf die bekannten geodätischen Methoden auch in der Informatik eingang finden können.

Folgende Meilensteine wurden bisher definiert:

- Aufbau eines Messnetzes
- Anschluss des Messnetz ans Landesnetz
- Tachymetersteuerung mit Hilfe der verfügbaren Matlab-Toolbox
- Automatisches Tracking der Robotertrajektorie mittels Tachymeter (Tachymeter steht extern)
- Automatische Aufnahme von Zielzeichen und Bestimmung der Pose (Tachymeter fest auf Roboterplattform)
- Erzeugen einer 2D Karte der Umgebung, Transformation dieser Karte ins übergeordnete Netz
- Aufnahme einer 3D Punktwolke, mittels 2D Laserscanner und bekannter dynamischer Pose

## 0. Installation - Husky

In Anlehnung an das [ROS-Tutorial](http://wiki.ros.org/ROS/Tutorials), soll ein eigener Workspace auf dem Raspberry Pi, der virtuellen Linux Umgebung oder sonst irgendeinem Linux-System auf dem Ubuntu 16.04, 18.04 oder 20.04 läuft, erzeugt werden (siehe Kapitel [1.3.](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)). Statt `catkin_ws` ist ein anderer Name zu wählen, etwa `workspace_husky`. Im dazu gehörigen `workspace_husky/src` Ordner können beliebig viele [ROS-Packages](http://wiki.ros.org/Packages) abgelegt werden, `catkin` übernimmt einen großteil der Verwaltung zur Erzeugung lauffähiger Programme bzw. Nodes. 

Folgende Packete sind mit folgenden Befehlen in den Workspace zu laden :
```bash
$ cd ~/workspace_husky/src
$ git clone https://github.com/husky/husky.git
$ git clone https://github.com/clearpathrobotics/LMS1xx
```
Außerdem ist mittels `git` der für das eigene System benötigte Branch auszuwählen (in diesem Beispiel im Bezug auf die ROS Version `melodic`). Ist kein passender Branch für das eigene System verfügbar kann das nächstbeste ausgewählt werden, eventuell ist dieses trotzdem über catkin kompilierbar:
```bash
$ cd ~/workspace_husky/src/husky
$ git checkout melodic-devel
$ cd ..
$ cd ~/workspace_husky/src/MS1xx
$ git checkout melodic-devel
```
Anschließend ist der Workspace mit `$ catkin_make` zu kompilieren, eventuell auftretende Fehlermeldungen sind zu behandeln.
```bash
$ cd ~/workspace_husky
$ catkin_make
```

Die Anmeldung des Workspaces ins System erfolgt wie gewohnt über den `$ source .` Befehl, etwa:
```bash
$ source ~/workspace_husky/devel/setup.bash
```
Praktischerweise sollte dieser Befehl nach eigenem Ermessen in die `$ nano ~/.bashrc` geladen werden, um bei jedem Terminal verfügbar zu sein.

Anschließend muss die Node 
```bash
$ roslaunch husky_viz view_robot.launch
```
aufrufbar sein. Es startet das bereits bekannte rviz Visualisierungstool mit getroffenen Voreinstellungen die im Bezug auf den Husky optimiert wurden. Ist das rviz Fenster geöffnet kann aber sogleich in der Linken Spalte der Haken bei `Sensing/Odometrie` ⬜️ entfernt werden, da dies für die nun folgende Visualiserung hinderlich ist. Dafür sollte aber der Haken bei `Navigation` ✅ gesetzt werden, um eine sich aufbauende Karte beobachten zu können. Zusätzlich sind über `>> Add >> TF >> Ok` die TF-Frames zu laden. 


## 0. Installation - Sick&xsens 3D PW-Transformation

adsf


## 1. Publish PoseStamped
`Victoria`



`Ziel - Die Aufgabe ist dann gelöst, wenn `

## 2. Publish Hz, V, D
`Matthias - `

`Ziel - Die Aufgabe ist dann gelöst, wenn `

## 3. Provide Fixed-point field
`Andreas S. -`

`Ziel - Die Aufgabe ist dann gelöst, wenn `

## 4. Publish 2D probability Map
`Max - `

`Ziel - Die Aufgabe ist dann gelöst, wenn `

## 5. Publish transformed Pointcloud
`Andreas B. -`

`Ziel - Die Aufgabe ist dann gelöst, wenn `
