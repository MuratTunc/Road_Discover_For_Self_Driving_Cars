# Road_Discover_For_Self_Driving_Cars

1-INSTALL OPENCV 4.2.0 TO UBUNTU 18.04

-Run script install_opencv.sh (I get this script from https://github.com/AmitThakur/opencvp)
-cd /etc/
-sudo nano ld.so.conf
-add #include /usr/local/lib
-save file.
-sudo ldconfig


2-QT ADJUSTMENTS
.pro file:
INCLUDEPATH += /usr/local/include/opencv4/

LIBS += $(shell pkg-config opencv4 --libs)





Best Regards. 
---------------
Murat Tunç
