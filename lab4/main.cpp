#include <opencv4/opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

enum CLPointType {LEFT, RIGHT, BEYOND, BEHIND, BETWEEN, ORIGIN, DESTINATION};

CLPointType classify(Point p1, Point p2, Point p) {
    double ax = p2.x - p1.x;
    double ay = p2.y - p1.y;
    double bx = p.x - p1.x;
    double by = p.y - p1.y;
    double s = ax * by - bx * ay;
    
    if (s > 0) return LEFT;
    if (s < 0) return RIGHT;
    if ((ax * bx < 0) || (ay * by < 0)) return BEHIND;
    if ((ax * ax + ay * ay) < (bx * bx + by * by)) return BEYOND;
    if (p1.x == p.x && p1.y == p.y) return ORIGIN;
    if (p2.x == p.x && p2.y == p.y) return DESTINATION;
    return BETWEEN;
}

void drawLine(Mat &img, Point p1, Point p2) {
    int x = p1.x, y = p1.y;
    int dx = (x < p2.x) ? p2.x - x : x - p2.x;
    int dy = (y < p2.y) ? p2.y - y : y - p2.y;
    int ix = (x < p2.x) ? 1 : -1;
    int iy = (y < p2.y) ? 1 : -1;
    int error;
    if (dx >= dy) {
        error = 2 * dy - dx;
        if (iy >= 0) {
            for (int i = 0; i < dx; ++i) {
                img.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
                if (error >= 0)
                {
                    y += iy;
                    error -= 2 * dx;
                }
                x += ix;
                error += 2 * dy;
            }
        }
        else {
            for (int i = 0; i < dx; ++i){
                img.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
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
                img.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
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
                img.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
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

Point getCubicBezierCurvePoint(Point p0, Point p1, Point p2, Point p3, double t)
{
    double B0 = (1 - t) * (1 - t) * (1 - t);
    double B1 = 3 * t * (1 - t) * (1 - t);
    double B2 = 3 * t * t * (1 - t);
    double B3 = t * t * t;
    int x = int(round(B0 * p0.x + B1 * p1.x + B2 * p2.x + B3 * p3.x));
    int y = int(round(B0 * p0.y + B1 * p1.y + B2 * p2.y + B3 * p3.y));
    return Point(x, y);
}

void drawCubicBezierCurve(Mat &img, Point p0, Point p1, Point p2, Point p3, int N)
{
    double step = 1.0 / (N - 1);
    for (int i = 0; i < N; ++i)
    {
        Point p = getCubicBezierCurvePoint(p0, p1, p2, p3, i * step);
        Point q = getCubicBezierCurvePoint(p0, p1, p2, p3, (i + 1) * step);
        drawLine(img, p, q);
    }
}

int CyrusBeckClipLine(Point& p1, Point& p2, std::vector<Point> points) {
    int n = points.size();
    Point new_p1, new_p2;
    double t1 = 0, t2 = 1, t;
    double sx = p2.x - p1.x, sy = p2.y - p1.y;
    double nx, ny, denom, num;
    for (int i = 0; i < n; ++i) {
        nx = points[(i + 1) % n].y - points[i].y;
        ny = points[i].x - points[(i + 1) % n].x;
        denom = nx * sx + ny * sy;
        num = nx * (p1.x - points[i].x) + ny * (p1.y - points[i].y);
        if (denom != 0) {
            t = -num / denom;
            if (denom > 0) {
                if (t > t1)
                    t1 = t;
            }
            else {
                if (t < t2) {
                    t2 = t;
                }
            }
        }
        else {
            if (classify(points[i], points[(i + 1) % n], p1) == LEFT) {
                return 0;
            }
        }
    }
    if (t1 <= t2) {
        new_p1.x = p1.x + t1 * (p2.x - p1.x);
        new_p1.y = p1.y + t1 * (p2.y - p1.y);
        new_p2.x = p1.x + t2 * (p2.x - p1.x);
        new_p2.y = p1.y + t2 * (p2.y - p1.y);
        p1.x = new_p1.x;
        p1.y = new_p1.y;
        p2.x = new_p2.x;
        p2.y = new_p2.y;
        return 1;
    }
    return 0;
}

void drawPolygon(Mat& img, std::vector<Point>& points) {
    for (int i = 0; i < points.size(); ++i) {
        drawLine(img, points[i], points[(i + 1) % points.size()]);
    }
}

void clipLine(Mat &img, Point p1, Point p2, std::vector<Point> points) {
    Point point1 = p1, point2 = p2;
    if (CyrusBeckClipLine(point1, point2, points)) {
        drawLine(img, point1, point2);
    }
    else {
        std::cout << "Line can't be clipped";
    }
}

int main() {
    Mat img(500, 500, CV_8UC3, Scalar(255, 255, 255));
    int n;
    std::cin >> n;
    std::vector<Point> points;
    for (int i = 0; i < n; ++i) {
        int x, y;
        std::cin >> x >> y;
        points.push_back(Point(x, y));
    }
    // drawCubicBezierCurve(img, Point(0, 0), Point(400, 20), Point(0, 100), Point(400, 400), 1000);
    drawPolygon(img, points);
    clipLine(img, Point(220, 320), Point(320, 220), points);
    imwrite("../out/clipped_line.jpg", img);
    return 0;
}