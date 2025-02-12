#include <iostream>
#include "opencv4/opencv2/opencv.hpp"

using namespace std;
using namespace cv;

enum CLPointType {LEFT, RIGHT, BEYOND, BEHIND, BETWEEN, ORIGIN, DESTINATION};
enum IntersectType {SAME, PARALLEL, SKEW, SKEW_CROSS, SKEW_NO_CROSS};
enum EType {TOUCHING, CROSS_LEFT, CROSS_RIGHT, INESSENTIAL};
enum PType {INSIDE, OUTSIDE};
enum polygonOrientation {CW, CCW};

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


polygonOrientation getPolygonOrientation(vector<Point> points) {
    int n = points.size();
    int sum = 0;

    for (int i = 0; i < n; ++i) {
        sum += (points[(i + 1) % n].x - points[i].x) * (points[(i + 1) % n].y + points[i].y);
    }

    return (sum > 0) ? CW : CCW;
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

vector<Point> reorientPolygon(vector<Point> points) {
    vector<Point> newPolygon;
    newPolygon.push_back(points[0]);
    for (int i = points.size() - 1; i >= 1; --i) {
        newPolygon.push_back(points[i]);
    }
    return newPolygon;
}

vector<Point> getConvexContour(vector<Point>& points) {
    if (points.size() < 3) return {};
    
    if (getPolygonOrientation(points) == CCW) {
        points = reorientPolygon(points);
    }
    
    int leftmost = 0;
    for (int i = 1; i < points.size(); i++) {
        if (points[i].x < points[leftmost].x) {
            leftmost = i;
        }
    }
    
    vector<Point> contour;
    int p = leftmost, q;
    do {
        contour.push_back(points[p]);
        q = (p + 1) % points.size();
        
        for (int i = 0; i < points.size(); i++) {
            if (classify(points[p], points[i], points[q]) == LEFT) {
                q = i;
            }
        }
        p = q;
    } while (p != leftmost);
    
    return contour;
}

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

void drawPolygon(Mat& img, vector<Point>& points, Vec3b color = Vec3b(0, 0, 0)) {
    for (int i = 0; i < points.size(); ++i) {
        drawLine(img, points[i], points[(i + 1) % points.size()], color);
    }
}

int main(int argc, char** argv) {
    Mat img(500, 500, CV_8UC3, Scalar(255, 255, 255));

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
    drawPolygon(img, points);

    vector<Point> contour = getConvexContour(points);
    drawPolygon(img, contour, Vec3b(0, 0, 255));

    imwrite("../out/polygon_and_convex_contour.jpg", img);
    return 0;
}