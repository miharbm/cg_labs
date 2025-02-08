#include <iostream>
#include "opencv4/opencv2/opencv.hpp"

using namespace std;
using namespace cv;

enum CLPointType {LEFT, RIGHT, BEYOND, BEHIND, BETWEEN, ORIGIN, DESTINATION};
enum IntersectType {SAME, PARALLEL, SKEW, SKEW_CROSS, SKEW_NO_CROSS};
enum EType {TOUCHING, CROSS_LEFT, CROSS_RIGHT, INESSENTIAL};
enum PType {INSIDE, OUTSIDE};
enum fillType {EO, NZW};

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

EType getEdgeType(Point o, Point d, Point a) {
    switch(classify(o, d, a)) {
        case LEFT:
            if (a.y > o.y && a.y <= d.y) {
                return CROSS_LEFT;
            }
            else {
                return INESSENTIAL;
            }
        case RIGHT:
            if (a.y > d.y && a.y <= o.y) {
                return CROSS_RIGHT;
            }
            else {
                return INESSENTIAL;
            }
        case BETWEEN:
        case ORIGIN:
        case DESTINATION:
            return TOUCHING;
        default:
            return INESSENTIAL;
    }
}

PType PInPolygonEOMode(Point p, const vector<Point>& points) {
    int n = points.size();
    int param = 0;
    for (int i = 0; i < n; ++i) {
        switch(getEdgeType(points[i], points[(i + 1) % n], p)) {
            case TOUCHING:
                return INSIDE;
            case CROSS_LEFT:
            case CROSS_RIGHT:
                param = 1 - param;
        }
    }
    if (param == 1) {
        return INSIDE;
    } 
    else {
        return OUTSIDE;
    }
}

PType PInPolygonNZWMode(Point p, const vector<Point>& points) {
    int n = points.size();
    int param = 0;
    for (int i = 0; i < n; ++i) {
        switch(getEdgeType(points[i], points[(i + 1) % n], p)) {
            case TOUCHING:
                return INSIDE;
            case CROSS_LEFT:
                param++;
                break;
            case CROSS_RIGHT:
                param--;
                break;
        }
    }
    
    if (param == 0) {
        return OUTSIDE;
    } 
    else {
        return INSIDE;
    }
}

void fillPolygon(Mat& img, vector<Point>& points, fillType type) {
    int n = img.rows;
    int m = img.cols;
    if (type == EO) {
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                if (PInPolygonEOMode(Point(j, i), points) == INSIDE) {
                    img.at<Vec3b>(i, j) = Vec3b(127, 127, 0);
                }
            }
        }
    }
    else {
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                if (PInPolygonNZWMode(Point(j, i), points) == INSIDE) {
                    img.at<Vec3b>(i, j) = Vec3b(127, 127, 0);
                }
            }
        }
    }
}

IntersectType intersect(Point a, Point b, Point c, Point d, double* t) {
    double nx = d.y - c.y;
    double ny = c.x - d.x;
    CLPointType type;
    double denom = nx * (b.x - a.x) + ny * (b.y - a.y);
    if (denom == 0) {
        type = classify(c, d, a);
        if (type == LEFT || type == RIGHT) {
            return PARALLEL;
        }
        else {
            return SAME;
        }
    }
    double num = nx * (a.x - c.x) + ny * (a.y - c.y);
    *t = -num/denom;
    return SKEW;
}

IntersectType cross(Point a, Point b, Point c, Point d, double* tab, double* tcd) {
    IntersectType type = intersect(a, b, c, d, tab);
    if (type == SAME || type == PARALLEL) {
        return type;
    }
    if ((*tab < 0) || (*tab > 1)) {
        return SKEW_NO_CROSS;
    }
    intersect(c, d, a, b, tcd);
    if ((*tcd < 0) || (*tcd > 1)) {
        return SKEW_NO_CROSS;
    }
     return SKEW_CROSS;
}

bool isComplex(const vector<Point>& points) {
    int n = points.size();
    if (n < 3) return true;
    for (int i = 0; i < n; ++i) {
        Point a = points[i];
        Point b = points[(i + 1) % n];

        for (int j = i + 2; j < n; ++j) {
            if (j == (i + n - 1) % n) {
                continue;
            }

            Point c = points[j];
            Point d = points[(j + 1) % n];


            double tab, tcd;
            IntersectType type = cross(a, b, c, d, &tab, &tcd);
            if (type == SKEW_CROSS) {
                return true;
            }
        }
    }
    return false;
}

bool isConvex(const vector<Point>& points) {
    int n = points.size();
    if (n < 3) return false;

    CLPointType initialSide = LEFT;
    bool sideSet = false;

    for (int i = 0; i < n; ++i) {
        cv::Point p1 = points[i];
        cv::Point p2 = points[(i + 1) % n];

        
        for (int j = 0; j < n; ++j) {
            if (j == i || j == (i + 1) % n) continue;

            CLPointType position = classify(p1, p2, points[j]);
            
            if (position == LEFT || position == RIGHT) {
                if (!sideSet) {
                    initialSide = position;
                    sideSet = true;
                } 
                else if (position != initialSide) {
                    return false;
                }
            }
        }
    }

    return true;
}

void drawLine(Mat& img, Point p1, Point p2) {
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
                img.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
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

void drawPolygon(Mat& img, vector<Point>& points) {
    for (int i = 0; i < points.size(); ++i) {
        drawLine(img, points[i], points[(i + 1) % points.size()]);
    }
}

int main(int argc, char** argv) {
    Mat img1(1000, 1000, CV_8UC3, Scalar(255, 255, 255));
    Mat img2(1000, 1000, CV_8UC3, Scalar(255, 255, 255));

    int n;
    cout << "Enter number of points: ";
    cin >> n;

    if (n <= 0) {
        cout << "Invalid number of points" << endl;
        return -1;
    }

    vector<Point> points;
    for (int i = 0; i < n; ++i) {
        int x, y;
        cout << "Enter point " << i + 1 << ": ";
        cin >> x >> y;
        points.push_back(Point(x, y));
    }

    drawPolygon(img1, points);
    drawPolygon(img2, points);

    if (isConvex(points)) {
        cout << "Многоугольник выпуклый" << endl;
    }
    else {
        cout << "Многоугольник вогнутый" << endl;
    }

    if (isComplex(points)) {
        cout << "Многоугольник сложный" << endl;
    }
    else {
        cout << "Многоугольник простой" << endl;
    }

    imwrite("../out/initial.png", img1);

    fillPolygon(img1, points, EO);
    fillPolygon(img2, points, NZW);

    imwrite("../out/filled_eo.png", img1);
    imwrite("../out/filled_nzw.png", img2);
    
    // waitKey(0);
 
    return 0;
}