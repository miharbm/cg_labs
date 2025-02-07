#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;
using namespace cv;


int getClosestLevel(int value, int levels) {
    int step = 255 / (levels - 1);
    return round((float)value / step) * step;
}

int checkIntOverflow(int value) {
    return clamp(value, 0, 255);
}

Mat floydSteinberg(Mat &image, int levels) {
    Mat outputImage = image.clone();

    for (int i = 0; i < image.rows; ++i) {
        if (i % 2 == 0) {
            for (int j = 0; j < image.cols; ++j) {
                int oldPixel = outputImage.at<uchar>(i, j);
                int newPixel = getClosestLevel(oldPixel, levels);  
                outputImage.at<uchar>(i, j) = newPixel;

                int difference = oldPixel - newPixel;

                if (j + 1 < image.cols) {
                    outputImage.at<uchar>(i, j + 1) = checkIntOverflow(outputImage.at<uchar>(i, j + 1) + difference * 7 / 16);
                }
                if (j + 1 < image.cols && i + 1 < image.rows) {
                    outputImage.at<uchar>(i + 1, j + 1) = checkIntOverflow(outputImage.at<uchar>(i + 1, j + 1)  + difference * 1 / 16);
                }
                if (i + 1 < image.rows) {
                    outputImage.at<uchar>(i + 1, j) = checkIntOverflow(outputImage.at<uchar>(i + 1, j) + difference * 5 / 16);
                }
                if (j - 1 >= 0 && i + 1 < image.rows) {
                    outputImage.at<uchar>(i + 1, j - 1) = checkIntOverflow(outputImage.at<uchar>(i + 1, j - 1) + difference * 3 / 16);
                }
            }
        }
        else {
            for (int j = image.cols - 1; j >= 0; --j) {
                int oldPixel = outputImage.at<uchar>(i, j);
                int newPixel = getClosestLevel(oldPixel, levels);  
                outputImage.at<uchar>(i, j) = newPixel;

                int difference = oldPixel - newPixel;

                if (j - 1 >= 0) {
                    outputImage.at<uchar>(i, j - 1) = checkIntOverflow(outputImage.at<uchar>(i, j - 1) + difference * 7 / 16);
                }
                if (j - 1 >= 0 && i + 1 < image.rows) {
                    outputImage.at<uchar>(i + 1, j - 1) = checkIntOverflow(outputImage.at<uchar>(i + 1, j - 1)  + difference * 1 / 16);
                }
                if (i + 1 < image.rows) {
                    outputImage.at<uchar>(i + 1, j) = checkIntOverflow(outputImage.at<uchar>(i + 1, j) + difference * 5 / 16);
                }
                if (j + 1 < image.cols && i + 1 < image.rows) {
                    outputImage.at<uchar>(i + 1, j + 1) = checkIntOverflow(outputImage.at<uchar>(i + 1, j + 1) + difference * 3 / 16);
                }
            }
        }
    }

    return outputImage;
}


int main(int argc, char** argv) {
    string srcImg = "../images/ALX09502.jpg";

    Mat image = imread(srcImg, IMREAD_GRAYSCALE);

    int n;
    cin >> n;


    if (n < 1 || n > 8) {
        cout << "Invalid value" << endl;
        return -1;
    }

    if (n == 8) {
        imwrite("../out/result.png", image);
        return 0;
    }

    Mat outputImage = floydSteinberg(image, static_cast<int>(pow(2, n)));

    imwrite("../out/result.png", outputImage);
    
    return 0;
}