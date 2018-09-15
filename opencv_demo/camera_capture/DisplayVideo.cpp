/*************************************************************************
    > File Name: DisplayVideo.cpp
    > Author: jones
    > Mail: kangjunderensheng@163.com 
    > Created Time: Thu 03 May 2018 11:39:18 PM CST
 ************************************************************************/
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;


void show_camera_properties(cv::VideoCapture camera)
{
	int code;
	printf("\r\rwidth      = %.2f\n",camera.get(CV_CAP_PROP_FRAME_WIDTH));
	printf("\r\rheight     = %.2f\n",camera.get(CV_CAP_PROP_FRAME_HEIGHT));
	printf("\r\rfbs        = %.2f\n",camera.get(CV_CAP_PROP_FPS));
	printf("\r\rbrightness = %.2f\n",camera.get(CV_CAP_PROP_BRIGHTNESS));
	printf("\r\rcontrast   = %.2f\n",camera.get(CV_CAP_PROP_CONTRAST));
	printf("\r\rsaturation = %.2f\n",camera.get(CV_CAP_PROP_SATURATION));
	printf("\r\rhue        = %.2f\n",camera.get(CV_CAP_PROP_HUE));
	printf("\r\rexposure   = %.2f\n",camera.get(CV_CAP_PROP_EXPOSURE));
	printf("\r\rcodec     = %.2f\n", camera.get(CV_CAP_PROP_FOURCC));

	printf("\t\t MPEG-1 codec: %d\n", CV_FOURCC('P', 'I', 'M', '1')); // MPEG-1 codec
	printf("\t\t MPEG-1 codec: %d\n", CV_FOURCC('M', 'J', 'P', 'G')); // motion-jpeg codec
	printf("\t\t MPEG-1 codec: %d\n", CV_FOURCC('M', 'P', '4', '2')); // MPEG-4.2 codec 
	printf("\t\t MPEG-1 codec: %d\n", CV_FOURCC('D', 'I', 'V', '3')); // MPEG-4.3 codec
	printf("\t\t MPEG-1 codec: %d\n", CV_FOURCC('D', 'I', 'V', 'X')); // MPEG-4 codec 
	printf("\t\t MPEG-1 codec: %d\n", CV_FOURCC('U', '2', '6', '3')); // H263 codec 
	printf("\t\t MPEG-1 codec: %d\n", CV_FOURCC('I', '2', '6', '3')); // H263I codec 
	printf("\t\t MPEG-1 codec: %d\n", CV_FOURCC('F', 'L', 'V', '1')); // FLV1 codec

}

int main()
{   
	//initialize and allocate memory to load the video stream from camera     
	// left 
	cv::VideoCapture camera0(2);  
	// camera0.set(CV_CAP_PROP_FRAME_WIDTH,320);  
	// camera0.set(CV_CAP_PROP_FRAME_HEIGHT,240);  	
	// camera0.set(CV_CAP_PROP_FPS,30);  
	printf("Camera 0: \r\n");
	show_camera_properties(camera0);	
	// right 
	cv::VideoCapture camera1(1);  
	// camera1.set(CV_CAP_PROP_FPS,30);  	
	// camera1.set(CV_CAP_PROP_FRAME_WIDTH,320);
	// camera1.set(CV_CAP_PROP_FRAME_HEIGHT,240);   
	printf("Camera 1: \r\n");
	show_camera_properties(camera1);	

	int i = 1;
	if( !camera0.isOpened() )
		return 1;  
	if( !camera1.isOpened() )
		return 1;
	while(true)
	{   
		//grab and retrieve each frames of the video sequentially      
		// left 
		cv::Mat frame0;     
		camera0 >> frame0;   

		// right
		cv::Mat frame1;      
		camera1 >> frame1;      

		cv::imshow("left", frame0);      
		cv::imshow("right", frame1);  
		//  std::cout << frame1.rows() << std::endl;      
		//wait for 40 milliseconds      
		char c = cvWaitKey(40);   
		string filename_left = format("./pic/left%d.jpg", i);
		string filename_right = format("./pic/right%d.jpg", i);
		//exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)      
		if(27 == c)
			break;
		else if('p' == c)
		{
			i++;
			imwrite(filename_left, frame0);
			imwrite(filename_right, frame1);
			imshow("photo0", frame0);
			imshow("photo1", frame1);
			waitKey(500);
			destroyWindow("photo0");
			destroyWindow("photo1");
		}
    }  
	return 0;
}
