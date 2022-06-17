TARGET := FrameViewer_DS77

SOURCE := FrameViewer.cpp

FLAGS := -I../../Include/DS77 -I../../Thirdparty/opencv-3.4.1/include -Wl,-rpath,./:../Lib/:../../Lib/:../Thirdparty/opencv-3.4.1/lib:../../Thirdparty/opencv-3.4.1/lib -Wall -Wconversion -O3 -L../../Lib -lvzenseDS_api -L../../Thirdparty/opencv-3.4.1/lib -lopencv_core -lopencv_highgui -lopencv_imgproc 

$(TARGET):$(SOURCE)
	g++ -std=c++11 -o $(TARGET) $(SOURCE) $(FLAGS) 

clean:
	rm -rf *.o $(TARGET)
