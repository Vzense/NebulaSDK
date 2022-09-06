TARGET := DS77_OpenCVSample

SOURCE := FrameViewer.cpp

FLAGS := -I../../../Include -I../Thirdparty/opencv-3.4.1/include -Wl,-rpath,./:../Lib/:../../../Lib/:../Samples/OpenCV/Thirdparty/opencv-3.4.1/lib:../Thirdparty/opencv-3.4.1/lib -Wall -Wconversion -O3 -L../../../Lib -lNebula_api -L../Thirdparty/opencv-3.4.1/lib -lopencv_core -lopencv_highgui -lopencv_imgproc 

$(TARGET):$(SOURCE)
	g++ -std=c++11 -o $(TARGET) $(SOURCE) $(FLAGS) 

clean:
	rm -rf *.o $(TARGET)
