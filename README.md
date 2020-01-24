# Road_Discover_For_Self_Driving_Cars

1-INSTALL OPENCV 4.2.0 TO UBUNTU 18.04

-Run script install_opencv.sh (I get this script from https://github.com/milq/milq/blob/master/scripts/bash/install-opencv.sh)
-cd /etc/
-sudo nano ld.so.conf
-add #include /usr/local/lib
-save file.
-sudo ldconfig
run this command at shell:
pkg-config --libs opencv4

You will see like this;
-L/usr/local/lib -lopencv_gapi -lopencv_stitching -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_cvv -lopencv_dnn_objdetect -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hdf -lopencv_hfs -lopencv_img_hash -lopencv_line_descriptor -lopencv_quality -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_sfm -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow -lopencv_surface_matching -lopencv_tracking -lopencv_datasets -lopencv_text -lopencv_dnn -lopencv_plot -lopencv_videostab -lopencv_video -lopencv_xfeatures2d -lopencv_shape -lopencv_ml -lopencv_ximgproc -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core




2-QT ADJUSTMENTS
.pro file:
INCLUDEPATH += /usr/local/include/opencv4/

LIBS += $(shell pkg-config opencv4 --libs)





Best Regards. 
---------------
Murat Tunç
