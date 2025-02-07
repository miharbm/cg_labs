#include <iostream>
#include "opencv4/opencv2/opencv.hpp"

using namespace std;
using namespace cv;


int main(int argc, char** argv) {
    string imgToGrayscale = "../images/ALX09491.jpg";


    Mat image = imread(imgToGrayscale, IMREAD_COLOR);

    if (image.empty()){
        cout << "Could not open or find the image for first part of the lab" << endl;
        return -1;
    }

    Mat grayImage(image.rows, image.cols, CV_8U);

    for (int i = 0; i < image.rows; i++) {
        for (int j= 0; j < image.cols; j++) {
            Vec3b pixel = image.at<Vec3b>(i, j);
            grayImage.at<uchar>(i, j) = static_cast<uchar>(0.299 * pixel[2] + 0.587 * pixel[1] + 0.114 * pixel[0]);
        }
    }
    
    Mat mask = Mat::zeros(grayImage.size(), CV_8U);
    int radius = min(grayImage.rows, grayImage.cols) / 2;
    Point center(grayImage.cols / 2, grayImage.rows / 2);
    for (int i = 0; i < grayImage.rows; i++) {
        for (int j = 0; j < grayImage.cols; j++) {
            if (sqrt(pow(j - center.x, 2) + pow(i - center.y, 2)) <= radius) {
                mask.at<uchar>(i, j) = 255;
            }
        }
    }

    Mat maskedImage = Mat::zeros(grayImage.size(), grayImage.type());
    for (int i = 0; i < grayImage.rows; i++) {
        for (int j = 0; j < grayImage.cols; j++) {
            maskedImage.at<uchar>(i, j) = grayImage.at<uchar>(i, j) * mask.at<uchar>(i, j) / 255;
        }
    }

    // Вторая часть
    string imgToBlend1 = "../images/ALX09491.jpg";
    string imgToBlend2 = "../images/ALX09742.jpg";
    string imgToBlendAlpha = "../images/ALX09580.jpg";


    Mat image1 = imread(imgToBlend1, IMREAD_COLOR);
    Mat image2 = imread(imgToBlend2, IMREAD_COLOR);
    Mat alphaImage = imread(imgToBlendAlpha, IMREAD_COLOR);

    if (image1.empty() || image2.empty() || alphaImage.empty()) {
        cout << "Could not open or find the image for second part of the lab" << endl;
        return -1;
    }

    Mat image1Grayscale= Mat::zeros(image1.rows, image1.cols, CV_8U);
    Mat image2Grayscale= Mat::zeros(image2.rows, image2.cols, CV_8U);
    Mat alphaImageGrayscale= Mat::zeros(alphaImage.rows, alphaImage.cols, CV_8U);
    for (int i = 0; i < image.rows; i++) {
        for (int j= 0; j < image.cols; j++) {
            Vec3b pixel1 = image1.at<Vec3b>(i, j);
            Vec3b pixel2 = image2.at<Vec3b>(i, j);
            Vec3b alphaPixel = alphaImage.at<Vec3b>(i, j);
            image1Grayscale.at<uchar>(i, j) = static_cast<uchar>(0.299 * pixel1[2] + 0.587 * pixel1[1] + 0.114 * pixel1[0]);
            image2Grayscale.at<uchar>(i, j) = static_cast<uchar>(0.299 * pixel2[2] + 0.587 * pixel2[1] + 0.114 * pixel2[0]);
            alphaImageGrayscale.at<uchar>(i, j) = static_cast<uchar>(0.299 * alphaPixel[2] + 0.587 * alphaPixel[1] + 0.114 * alphaPixel[0]);

        }
    }

    Mat blendedImage = Mat::zeros(image1.size(), CV_8U);
    for (int i = 0; i < image1.rows; i++) {
        for (int j = 0; j < image1.cols; j++) {
            float alpha = alphaImageGrayscale.at<uchar>(i, j) / 255.0f;
            blendedImage.at<uchar>(i, j) = static_cast<uchar>(image1Grayscale.at<uchar>(i, j) * alpha + image2Grayscale.at<uchar>(i, j) * (1.0 - alpha));
        }
    }

    // namedWindow("Image1", WINDOW_GUI_EXPANDED);
    // imshow("Image1", maskedImage);
    imwrite("../out/maskedImage.jpg", maskedImage);

    // namedWindow("Blended Image", WINDOW_GUI_EXPANDED);
    // imshow("Blended Image", blendedImage);
    imwrite("../out/blendedImage.jpg", blendedImage);
    
    // waitKey(0);
 
    return 0;
}