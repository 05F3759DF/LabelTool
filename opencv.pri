CONFIG += OPENCV3
OPENCV2 {
message("opencv2")
#opencv2
LIBS += -L/usr/lib/x86-64-linux-gnu -lopencv_core  -lopencv_highgui -lopencv_imgproc
}
else{
message("opencv3")
#opencv3
INCLUDEPATH += /usr/local/include/
LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_videoio
}
