#include <iostream>
#include "opencv4/opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int getClosestLevel(int value, int levels) {
    int step = 255 / (levels - 1);
    return int(round((float)value / step) * step);
}

int checkIntOverflow(int value) {
    return clamp(value, 0, 255);
}

vector<vector<int>> getD(int R) {
    if (R == 2) {
        return vector<vector<int>> {
            {0, 2},
            {3, 1}
        };
    }
    else {
        vector<vector<int>> result(R, vector<int>(R));
        vector<vector<int>> DR2 = getD(int(round(R / 2)));
        for (int i = 0; i < DR2.size(); ++i) {
            for (int j = 0; j < DR2[i].size(); ++j) {
                result[i][j] = 4 * DR2[i][j];
                result[i + DR2.size()][j] = 4 * DR2[i][j] + 3;
                result[i][j + DR2[i].size()] = 4 * DR2[i][j] + 2;
                result[i + DR2.size()][j + DR2[i].size()] = 4 * DR2[i][j] + 1;
            }
        }
        return result;
    }
}

Mat getNewImage(Mat img, int R, int numColorsOut) {
    Mat result = Mat::zeros(img.size(), CV_8U);
    vector<vector<int>> D = getD(R);
    for (int i = 0; i < R; ++i) {
        for (int j = 0; j < R; ++j) {
            cout << D[i][j] << " ";
        }
        cout << endl;
    }
    for (int i = 0; i < result.rows; ++i) {
        for (int j = 0; j < result.cols; ++j) {
            double first = img.at<uchar>(i, j) * double(numColorsOut) / 256.0;
            double second = double(D[i % R][j % R]) / (R * R - 1);
            int pixel = getClosestLevel(int(round((first + second) * 255)), numColorsOut);
            result.at<uchar>(i, j) = checkIntOverflow(pixel);
        }
    }
    return result;
}

int main(int argc, char** argv) {
    Mat img = imread("../images/img1.jpg", IMREAD_GRAYSCALE);
    imwrite("../out/originalImage.jpg", img);
    Mat result = getNewImage(img, 8, 2);
    imwrite("../out/result.png", result);
    return 0;
}