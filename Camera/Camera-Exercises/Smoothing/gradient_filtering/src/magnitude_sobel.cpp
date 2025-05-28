#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void magnitudeSobel()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // apply smoothing using the GaussianBlur() function from the OpenCV
    // ToDo : Add your code here
	// Apply Gaussian Blur
    cv::Mat blurredImage = imgGray.clone();
    int kernel_size = 5;
  	float stdDev = 2.0;
    cv::GaussianBlur(imgGray, blurredImage, cv::Size(kernel_size, kernel_size), stdDev);

    // create filter kernels using the cv::Mat datatype both for x and y
    // ToDo : Add your code here
	 float sobel_x[9] = {-1, 0, +1,
                        -2, 0, +2, 
                        -1, 0, +1};
  
  	 float sobel_y[9] = {-1, -2, -1,
                        0, 0, 0, 
                        +1, +2, +1};
    cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);
    cv::Mat kernel_y = cv::Mat(3, 3, CV_32F, sobel_y);

    // apply filter using the OpenCv function filter2D()
    // ToDo : Add your code here
    cv::Mat result_x, result_y;
    cv::filter2D(blurredImage, result_x, -1, kernel_x, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    cv::filter2D(blurredImage, result_y, -1, kernel_y, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);


    // compute magnitude image based on the equation presented in the lesson 
    // ToDo : Add your code here
     // compute magnitude image
  
  	cv::Mat magnitudeImage = imgGray.clone();
  	for(int r = 0; r < magnitudeImage.rows; r++){
  		for(int c = 0; c < magnitudeImage.cols; r++){
    		magnitudeImage.at<unsigned char>(r,c) = sqrt(pow(result_x.at<unsigned char>(r, c), 2) + pow(result_y.at<unsigned char>(r, c),2));
    	}
    }
  

    // show result
    string windowName = "Gaussian Blurring";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, magnitudeImage);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    magnitudeSobel();
}