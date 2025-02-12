#include <iostream>
#include "opencv4/opencv2/opencv.hpp"

using namespace std;
using namespace cv;

void drawLine(Mat& img, Point p1, Point p2, Vec3b color = Vec3b(0, 0, 0)) {
    int x2 = p2.x, y2 = p2.y;
    int x = p1.x, y = p1.y;
    int dx = (x < x2) ? x2 - x : x - x2;
    int dy = (y < y2) ? y2 - y : y - y2;
    int ix = (x < x2) ? 1 : -1;
    int iy = (y < y2) ? 1 : -1;
    int error;
    if (dx >= dy) {
        error = 2 * dy - dx;
        if (iy >= 0) {
            for (int i = 0; i < dx; ++i) {
                img.at<Vec3b>(y, x) = color;
                if (error >= 0) {
                    y += iy;
                    error -= 2 * dx;
                }
                x += ix;
                error += 2 * dy;
            }
        }
        else {
            for (int i = 0; i < dx; ++i) {
                img.at<Vec3b>(y, x) = color;
                if (error > 0) {
                    y += iy;
                    error -= 2 * dx;
                }
                x += ix;
                error += 2 * dy;
            }
        }
    }
    else {
        error = 2 * dx - dy;
        if (iy >= 0) {
            for (int i = 0; i < dy; ++i) {
                img.at<Vec3b>(y, x) = color;
                if (error >= 0) {
                    x += ix;
                    error -= 2 * dy;
                }
                y += iy;
                error += 2 * dx;
            }
        }
        else {
            for (int i = 0; i < dy; ++i) {
                img.at<Vec3b>(y, x) = color;
                if (error > 0) {
                    x += ix;
                    error -= 2 * dy;
                }
                y += iy;
                error += 2 * dx;
            }
        }
    }
}

Point getErmitCurvePoint(vector<Point> points, vector<Point> vectors, double t) {
    return Point((1 - 3 * t * t + 2 * t * t * t) * points[0] + t * t * (3 - 2 * t) * points[1] + 
                 t * (1 - 2 * t + t * t) * vectors[0] - t * t * (1 - t) * vectors[1]);
}

void drawErmitCurve(Mat& img, vector<Point> points, vector<Point> vectors, int numPoints) {
    double tStep = 1.0 / (numPoints + 1);
    for (int i = 0; i < numPoints; ++i) {
        Point p1 = getErmitCurvePoint(points, vectors, i * tStep);
        Point p2 = getErmitCurvePoint(points, vectors, (i + 1) * tStep);
        drawLine(img, p1, p2);
    }
}

void drawCompositeErmitCurve(Mat& img, vector<Point> points, vector<Point> vectors, int numPoints) {
    if (points.size() != vectors.size() || points.size() < 2) {
        cerr << "Wrong input values!" << endl;
        return;
    }

    int numCurves = points.size() - 1;
    int pointsPerCurve = numPoints / numCurves;
    
    for (int i = 0; i < points.size() - 1; ++i) {
        vector<Point> newPoints = {points[i], points[i + 1]};
        vector<Point> newVectors = {vectors[i], vectors[i + 1]};
        drawErmitCurve(img, newPoints, newVectors, pointsPerCurve);
    }
}

int main(int argc, char** argv) {
    Mat img(500, 500, CV_8UC3, Scalar(255, 255, 255));
    vector<Point> points = {{0, 0}, {100, 150}, {250, 200}, {400, 400}};
    vector<Point> vectors = {{25, 75}, {-50, 100}, {40, -10}, {100, 100}};
    drawCompositeErmitCurve(img, points, vectors, 300);

    imwrite("../out/ermit_curve.png", img);
    return 0;
}