#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>

using namespace std;
using namespace cv;

std::vector<cv::Point3d> makeRotation(std::vector<cv::Point3d> points, cv::Point3d rotationNormalLine, int angle) {
    double phi = angle * CV_PI / 180;
    std::vector<cv::Point3d> rotatedPoints;
    for (int i = 0; i < points.size(); i++) {
        cv::Point3d newPoint;
        newPoint.x = points[i].x * (rotationNormalLine.x * rotationNormalLine.x * (1 - std::cos(phi)) + std::cos(phi)) + points[i].y * (rotationNormalLine.x * rotationNormalLine.y * (1 - std::cos(phi)) - rotationNormalLine.z * std::sin(phi)) + points[i].z * (rotationNormalLine.x * rotationNormalLine.z * (1 - std::cos(phi)) + rotationNormalLine.y * std::sin(phi));
        newPoint.y = points[i].x * (rotationNormalLine.x * rotationNormalLine.y * (1 - std::cos(phi)) + rotationNormalLine.z * std::sin(phi)) + points[i].y * (rotationNormalLine.y * rotationNormalLine.y * (1 - std::cos(phi)) + std::cos(phi)) + points[i].z * (rotationNormalLine.y * rotationNormalLine.z * (1 - std::cos(phi)) - rotationNormalLine.x * std::sin(phi));
        newPoint.z = points[i].x * (rotationNormalLine.x * rotationNormalLine.z * (1 - std::cos(phi)) - rotationNormalLine.y * std::sin(phi)) + points[i].y * (rotationNormalLine.y * rotationNormalLine.z * (1 - std::cos(phi)) + rotationNormalLine.x * std::sin(phi)) + points[i].z * (rotationNormalLine.z * rotationNormalLine.z * (1 - std::cos(phi)) + std::cos(phi));
        rotatedPoints.push_back(cv::Point3d(newPoint.x, newPoint.y, newPoint.z));
    }
    return rotatedPoints;
}

void drawLine(Mat& img, Point2d p1, Point2d p2, Point2d figureCenter) {
    Point2d imgCenter(int(round(img.cols / 2)), int(round(img.rows / 2)));
    p1 = p1 + imgCenter - figureCenter;
    p2 = p2 + imgCenter - figureCenter;
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
                img.at<Vec3b>(img.rows - y, x) = Vec3b(0, 0, 0);
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
                img.at<Vec3b>(img.rows - y, x) = Vec3b(0, 0, 0);
                if (error > 0) {
                    y += iy;
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
                img.at<Vec3b>(img.rows - y, x) = Vec3b(0, 0, 0);
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
                img.at<Vec3b>(img.rows - y, x) = Vec3b(0, 0, 0);
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

enum CLPointType {LEFT, RIGHT, BEYOND, BEHIND, BETWEEN, ORIGIN, DESTINATION};
enum IntersectType {SAME, PARALLEL, SKEW, SKEW_CROSS, SKEW_NO_CROSS};
enum EType {TOUCHING, CROSS_LEFT, CROSS_RIGHT, INESSENTIAL};
enum PType {INSIDE, OUTSIDE};
enum fillType {EO, NZW};
enum polygonOrientation {CW, CCW};

CLPointType classify(Point2d p1, Point2d p2, Point2d p) {
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

EType getEdgeType(Point2d o, Point2d d, Point2d a) {
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


PType PInPolygonEOMode(Point2d p, const vector<Point2d>& points) {
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

Point3d transformPerspective(Point3d p, double r) {
    Point3d transformedPoint(
        double(p.x) / (r * p.z + 1.0),
        double(p.y) / (r * p.z + 1.0), 
        double(p.z) / (r * p.z + 1.0)
    );
    return transformedPoint;
}

Point2d transformParallel(Point3d p) {
    return Point2d(p.x, p.y);
}

double dot(Point3d p1, Point3d p2) {
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

Point3d getNormal(Point3d p1, Point3d p2, Point3d p3) {
    Point3d normal;
    normal.x = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
    normal.y = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
    normal.z = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
    return normal;
}

bool isFrontFace(Point3d normal, Point3d viewDirection) {
    double dotProduct = normal.x * viewDirection.x + normal.y * viewDirection.y + normal.z * viewDirection.z;
    return dotProduct > 0;
}

// void drawCube(Mat& img, vector<Point2d> points) {
//     for (int i = 0; i < 4; ++i) {
//         drawLine(img, points[i], points[(i + 1) % 4]);
//     }

//     for (int i = 4; i < 8; ++i) {
//         drawLine(img, points[i], points[(i - 3) % 4 + 4]);
//     }

//     for (int i = 0; i < 4; ++i) {
//         drawLine(img, points[i], points[i + 4]);
//     }
// }

void drawCubePerspective(Mat& img, vector<Point3d> transoformedPoints, Point2d center) {
    img.setTo(Scalar(255, 255, 255));
    vector<vector<int>> faces = {
        {0, 1, 2, 3}, // Нижняя грань
        {7, 6, 5, 4}, // Верхняя грань
        {3, 7, 4, 0}, // Передняя грань
        {7, 3, 2, 6}, // Правая грань
        {5, 6, 2, 1}, // Задняя грань
        {0, 4, 5, 1}  // Левая грань
    };

    Point3d viewDir(0, 0, -1);

    for (const auto& face : faces) {
        Point3d normal = getNormal(
            transoformedPoints[face[0]],
            transoformedPoints[face[1]],
            transoformedPoints[face[2]]
        );

        if (!isFrontFace(normal, viewDir)) {
            continue;
        }

        for (int i = 0; i < face.size(); ++i) {
            Point2d p1(transoformedPoints[face[i]].x, transoformedPoints[face[i]].y);
            Point2d p2(transoformedPoints[face[(i + 1) % face.size()]].x, transoformedPoints[face[(i + 1) % face.size()]].y);
            drawLine(img, p1, p2, center);
        }
    }
}

void drawCubeParallel(Mat& img, vector<Point2d> newPoints, vector<Point3d> originalPoints, Point2d center) { 
    img.setTo(Scalar(255, 255, 255));
    vector<vector<int>> faces = {
        {0, 1, 2, 3}, // Нижняя грань
        {7, 6, 5, 4}, // Верхняя грань
        {4, 5, 1, 0}, // Передняя грань
        {5, 6, 2, 1}, // Правая грань
        {6, 7, 3, 2}, // Задняя грань
        {7, 4, 0, 3}  // Левая грань
    };

    Point3d viewDir(0, 0, -1);

    for (const auto& face : faces) {
        Point3d normal = getNormal(
            originalPoints[face[0]],
            originalPoints[face[1]],
            originalPoints[face[2]]
        );


        if (!isFrontFace(normal, viewDir)) {
            continue;
        }

        for (int i = 0; i < face.size(); ++i) {
            drawLine(img, newPoints[face[i]], newPoints[face[(i + 1) % face.size()]], center);
        }
    }
}

Point3d normalize(Point3d vector){
    double length = 0;
    Point3d result;
    length = vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
    length = sqrt(length);
    result.x = vector.x / length;
    result.y = vector.y / length;
    result.z = vector.z / length;
    return result;
}

Point3d calculateNormalForVertex(Point3d vertex, vector<Point3d> points, vector<vector<int>> faces) {
    Point3d sumNormals = {0, 0, 0};
    int count = 0;

    for (auto face : faces) {
        for (int i = 0; i < face.size(); ++i) {
            if (points[face[i]] == vertex) {
                Point3d normal = getNormal(points[face[0]], points[face[1]], points[face[2]]);
                sumNormals.x += normal.x;
                sumNormals.y += normal.y;
                sumNormals.z += normal.z;
                count++;
                break;
            }
        }
    }

    sumNormals.x /= count;
    sumNormals.y /= count;
    sumNormals.z /= count;
    double length = sqrt(sumNormals.x * sumNormals.x + sumNormals.y * sumNormals.y + sumNormals.z * sumNormals.z);
    sumNormals.x /= -length; // тут заодно умножим на минус 1, чтобы получить внешнюю нормаль
    sumNormals.y /= -length;
    sumNormals.z /= -length;
    return sumNormals;
}

double getVertexFongIntensity(double Ia, double Ip, double kd, double ks, double n, 
                                    Point3d lightSource, Point3d viewDir, Point3d vertex, vector<Point3d> points, vector<vector<int>> faces){
    Point3d N = calculateNormalForVertex(vertex, points, faces);
    Point3d light = {lightSource.x - vertex.x, lightSource.y - vertex.y, lightSource.z - vertex.z};
    light = normalize(light);
    Point3d R = {2 * dot(N, light) * N.x - light.x, 2 * dot(N, light) * N.y - light.y, 2 * dot(N, light) * N.y - light.y};
    R = normalize(R);
    return Ia + Ip * kd * dot(N, light) + ks * pow(dot(normalize(viewDir), R), n);
}

double interpolate(double x, double left, double right, double leftIntensity, double rightIntensity) {
    return leftIntensity + (rightIntensity - leftIntensity) * (x - left) / (right - left);
}

pair<vector<double>, vector<double>> getIntersectionsShading(int y, vector<Point2d> points, vector<double> I) {
    int n = points.size();
    vector<double> intersections;
    vector<double> intensities;
    for (int i = 0; i < n; ++i) {
        double x1 = points[i].x;
        double y1 = points[i].y;
        double x2 = points[(i + 1) % n].x;
        double y2 = points[(i + 1) % n].y;
        if (y1 != y2) {
            double x = x1 + (y - y1) * (x2 - x1) / (y2 - y1);
            double intensity = I[i] + (I[(i + 1) % n] - I[i]) * (y - y1) / (y2 - y1);
            intersections.push_back(x);
            intensities.push_back(intensity); 
        }
    }
    return {intersections, intensities};
}

double getIntensity(int x, int y, vector<Point2d> points, vector<double> I) {
    pair<vector<double>, vector<double>> intersectResult = getIntersectionsShading(y, points, I);
    vector<double> intersections = intersectResult.first;
    vector<double> intensities = intersectResult.second;

    vector<pair<int, size_t>> indexed_vector;
    for (size_t i = 0; i < intersections.size(); ++i) {
        indexed_vector.emplace_back(intersections[i], i);
    }

    sort(indexed_vector.begin(), indexed_vector.end());

    vector<double> sorted_intersections;
    vector<double> sorted_intensities;
    vector<char> sorted_v2;
    for (const auto& [value, index] : indexed_vector) {
        sorted_intersections.push_back(value);
        sorted_intensities.push_back(intensities[index]);
    }

    for (int i = 0; i < sorted_intersections.size() - 1; ++i) {
        if (x >= sorted_intersections[i] && x <= sorted_intersections[i + 1]) {
            return interpolate(x, sorted_intersections[i], sorted_intersections[i + 1], 
                sorted_intensities[i], sorted_intensities[i + 1]);
        }
    }
    return 0;
}

void fillProjectedFace(Mat& img, vector<Point2d> points, vector<double> I, Point2d figureCenter) {
    Point2d imgCenter(int(round(img.cols / 2)), int(round(img.rows / 2)));
    int xMin = 1000, yMin = 1000, xMax = -1, yMax = -1;
    for (int i = 0; i < points.size(); ++i) {
        if (points[i].x < xMin) xMin = points[i].x;
        if (points[i].y < yMin) yMin = points[i].y;
        if (points[i].x > xMax) xMax = points[i].x;
        if (points[i].y > yMax) yMax = points[i].y;
    }
    for (int y = yMin; y <= yMax; ++y) {
        for (int x = xMin; x <= xMax; ++x) {
            if (PInPolygonEOMode(Point(x, y), points) == INSIDE) {
                double intensity = getIntensity(x, y, points, I);
                int clampedIntensity = max(0, min(255, int(round(intensity * 255))));
                img.at<uchar>(img.rows - (y + imgCenter.y - figureCenter.y), x + imgCenter.x - figureCenter.x) = static_cast<uchar>(clampedIntensity);
            }
        }
    }
}

void drawCubeParallelWithShading(Mat& img, vector<Point2d> newPoints, vector<Point3d> originalPoints, 
                                 double Ia, double Ip, double kd, double ks, int n, 
                                 Point3d lightSource, Point2d center) { 
    img.setTo(Scalar(0, 0, 0));
    vector<vector<int>> faces = {
        {0, 1, 2, 3}, // Нижняя грань
        {7, 6, 5, 4}, // Верхняя грань
        {4, 5, 1, 0}, // Передняя грань
        {5, 6, 2, 1}, // Правая грань
        {6, 7, 3, 2}, // Задняя грань
        {7, 4, 0, 3}  // Левая грань
    };

    Point3d viewDir(0, 0, -1);

    for (const auto& face : faces) {
        Point3d normal = getNormal(
            originalPoints[face[0]],
            originalPoints[face[1]],
            originalPoints[face[2]]
        );


        if (!isFrontFace(normal, viewDir)) {
            continue;
        }

        vector<double> I(face.size()); 
        for (int i = 0; i < face.size(); ++i) {
            I[i] = getVertexFongIntensity(Ia, Ip, kd, ks, n, lightSource, viewDir, originalPoints[face[i]], originalPoints, faces);
        }
        vector<Point2d> facePoints = {newPoints[face[1]], newPoints[face[2]], newPoints[face[3]], newPoints[face[4]]};
        for (int i = 0; i < facePoints.size(); ++i) {
            fillProjectedFace(img, facePoints, I, center);
        }
        // for (int i = 0; i < face.size(); ++i) {
        //     drawLine(img, newPoints[face[i]], newPoints[face[(i + 1) % face.size()]], center);
        // }
    }
}

void getPerspectiveProjection(Mat& img, vector<Point3d> points, double k) {
    vector<Point3d> transformedPoints;
    int n = points.size();
    for (int i = 0; i < n; ++i) {
        Point3d transformedPoint = transformPerspective(points[i], 1.0 / k);
        transformedPoints.push_back(transformedPoint);
    }
    Point2d center(0, 0);
    for (int i = 0; i < transformedPoints.size(); ++i) {
        center.x += transformedPoints[i].x;
        center.y += transformedPoints[i].y;
    }
    center = Point2d(int(round(center.x / transformedPoints.size())), int(round(center.y / transformedPoints.size())));
    drawCubePerspective(img, transformedPoints, center);
    // return transformedPoints;
}

void getParallelProjection(Mat& img, vector<Point3d> points) {
    vector<Point2d> newPoints;
    int n = points.size();
    for (int i = 0; i < n; ++i) {
        Point2d newPoint = transformParallel(points[i]);
        newPoints.push_back(newPoint);
    }
    Point2d center(0, 0);
    for (int i = 0; i < newPoints.size(); ++i) {
        center.x += newPoints[i].x;
        center.y += newPoints[i].y;
    }
    center = Point2d(int(round(center.x / newPoints.size())), int(round(center.y / newPoints.size())));
    drawCubeParallelWithShading(img, newPoints, points, 0.2, 0.8, 0.3, 0.5, 50, Point3d(0, 1000, 1000), center);
}

void animatePerspective(Mat& img, vector<Point3d> points, Point3d normal, int numFrames, int k) {
    double norm = sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    normal.x = int(round(double(normal.x) / norm));
    normal.y = int(round(double(normal.y) / norm));
    normal.z = int(round(double(normal.z) / norm));
    for (int i = 0; i < numFrames; ++i) {
        getPerspectiveProjection(img, points, k);
        string fileName = "../frames/frame" + to_string(i) + ".jpg";
        imwrite(fileName, img);
        points = makeRotation(points, normal, 6);
    }
}

void animateParallel(Mat& img, vector<Point3d> points, Point3d normal, int numFrames) {
    double norm = sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    normal.x = int(round(double(normal.x) / norm));
    normal.y = int(round(double(normal.y) / norm));
    normal.z = int(round(double(normal.z) / norm));
    for (int i = 0; i < numFrames; ++i) {
        getParallelProjection(img, points);
        string fileName = "../frames/frame" + to_string(i) + ".jpg";
        imwrite(fileName, img);
        points = makeRotation(points, normal, 6);
    }
}

int main() {
    Mat img(1000, 1000, CV_8UC3, Scalar(0, 0, 0));

    int n;
    cin >> n;

    vector<Point3d> points;
    for (int i = 0; i < n; ++i) {
        int x, y, z;
        cin >> x >> y >> z;
        points.push_back(Point3d(x, y, z));
    }
    
    Point3d normal(1, 0, 0);
    // animatePerspective(img, points, normal, 120, 400);
    //animateParallel(img, points, normal, 120);

    animateParallel(img, points, Point3d(1, 1, 1), 100);
    return 0;
}
