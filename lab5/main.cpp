#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>

using namespace std;
using namespace cv;

vector<Point3d> makeRotation(vector<Point3d> points, Point3d normal, int angle) {
    vector<Point3d> newPoints;
    for (int i = 0; i < points.size(); ++i) {
        double cosphi = cos(double(angle) * M_PI / 180);
        double sinphi = sin(double(angle) * M_PI / 180);
        Point3d newPoint;
        newPoint.x = points[i].x * (normal.x * normal.x * (1 - cosphi) + cosphi) + points[i].y * (normal.x * normal.y * (1 - cosphi) - normal.z * sinphi) + points[i].z * (normal.x * normal.z * (1 - cosphi) + normal.y * sinphi);
        newPoint.y = points[i].x * (normal.x * normal.y * (1 - cosphi) + normal.z * sinphi) + points[i].y * (normal.y * normal.y * (1 - cosphi) + cosphi) + points[i].z * (normal.y * normal.z * (1 - cosphi) - normal.x * sinphi);
        newPoint.z = points[i].x * (normal.x * normal.z * (1 - cosphi) - normal.y * sinphi) + points[i].y * (normal.y * normal.z * (1 - cosphi) + normal.x * sinphi) + points[i].z * (normal.z * normal.z * (1 - cosphi) + cosphi);
        newPoints.push_back(newPoint);
    }
    return newPoints;
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

Point3f transformPerspective(Point3d p, double r) {
    Point3f transformedPoint(
        double(p.x) / (r * p.z + 1.0),
        double(p.y) / (r * p.z + 1.0), 
        double(p.z) / (r * p.z + 1.0)
    );
    return transformedPoint;
}

Point2d transformParallel(Point3d p) {
    return Point2d(p.x, p.y);
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

Vec3b getVertexColor(Point3d normal, Point3d lightDir, Vec3b baseColor) {
    double intensity = max(0.0, (normal.x * lightDir.x + normal.y * lightDir.y + normal.z * lightDir.z));
    Vec3b color;
    for (int i = 0; i < 3; ++i) {
        color[i] = uchar(baseColor[i] * intensity);
    }
    return color;
}

void drawTriangleGouraud(Mat& img, vector<Point3d> points, vector<Vec3b> colors, Point2d center) {
    Point2d imgCenter(int(round(img.cols / 2)), int(round(img.rows / 2)));
    for (auto& p : points) {
        p.x += imgCenter.x - center.x;
        p.y += imgCenter.y - center.y;
    }

    // Сортировка по y
    if (points[1].y < points[0].y) swap(points[0], points[1]);
    if (points[2].y < points[1].y) swap(points[1], points[2]);
    if (points[1].y < points[0].y) swap(points[0], points[1]);

    // Интерполяция по высоте для получения цвета по каждому пикселю
    for (int y = points[0].y; y <= points[2].y; ++y) {
        if (y < 0 || y >= img.rows) continue;
        
        // Вычисление границ строки
        double t = (y - points[0].y) / (points[2].y - points[0].y + 1e-6);
        Point2d left = Point2d((points[0] + t * (points[2] - points[0])).x, (points[0] + t * (points[2] - points[0])).y);
        Vec3b leftColor = colors[0] + t * (colors[2] - colors[0]);

        Point2d right;
        Vec3b rightColor;
        if (y < points[1].y) {
            double t1 = (y - points[0].y) / (points[1].y - points[0].y + 1e-6);
            right = Point2d((points[0] + t1 * (points[1] - points[0])).x, (points[0] + t1 * (points[1] - points[0])).y);
            rightColor = colors[0] + t1 * (colors[1] - colors[0]);
        } else {
            double t2 = (y - points[1].y) / (points[2].y - points[1].y + 1e-6);
            right = Point2d((points[1] + t2 * (points[2] - points[1])).x, (points[1] + t2 * (points[2] - points[1])).y);
            rightColor = colors[1] + t2 * (colors[2] - colors[1]);
        }

        if (right.x < left.x) {
            swap(left, right);
            swap(leftColor, rightColor);
        }

        // Интерполяция цвета по строке
        for (int x = left.x; x <= right.x; ++x) {
            if (x < 0 || x >= img.cols) continue;
            double tx = (x - left.x) / (right.x - left.x + 1e-6);
            Vec3b color = leftColor + tx * (rightColor - leftColor);
            img.at<Vec3b>(img.rows - y, x) = color;
        }
    }
}


void drawCubePerspective(Mat& img, vector<Point3d> transoformedPoints, Point2d center) {
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

void drawCubeGouraud(Mat& img, vector<Point3d> points, Point3d lightDir, Point2d center) {
    vector<vector<int>> faces = {
        {0, 1, 2, 3}, // Нижняя грань
        {7, 6, 5, 4}, // Верхняя грань
        {4, 5, 1, 0}, // Передняя грань
        {5, 6, 2, 1}, // Правая грань
        {6, 7, 3, 2}, // Задняя грань
        {7, 4, 0, 3}  // Левая грань
    };

    Point3d viewDir(0, 0, -1);
    Vec3b baseColor(100, 100, 255);

    for (const auto& face : faces) {
        Point3d normal = getNormal(
            points[face[0]],
            points[face[1]],
            points[face[2]]
        );

        if (!isFrontFace(normal, viewDir)) {
            continue;
        }

        // Вычисление цвета для каждой вершины
        vector<Vec3b> colors;
        for (int i = 0; i < 4; ++i) {
            colors.push_back(getVertexColor(normal, lightDir, baseColor));
        }

        vector<Point3d> facePoints;
        for (int i = 0; i < 4; ++i) {
            facePoints.push_back(points[face[i]]);
        }

        // Закрашиваем грань методом Гуро
        drawTriangleGouraud(img, facePoints, colors, center);
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
    drawCubeParallel(img, newPoints, points, center);
    // return newPoints;
}

void getParallelProjectionGouraud(Mat& img, vector<Point3d> points) {
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
    drawCubeGouraud(img, points, Point3d(0, 0, -1), center);
    // return newPoints;
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

void animateGouraud(Mat& img, vector<Point3d> points, Point3d normal, int numFrames) {
    double norm = sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    normal.x = int(round(double(normal.x) / norm));
    normal.y = int(round(double(normal.y) / norm));
    normal.z = int(round(double(normal.z) / norm));
    for (int i = 0; i < numFrames; ++i) {
        getParallelProjectionGouraud(img, points);
        string fileName = "../frames/frame" + to_string(i) + ".jpg";
        imwrite(fileName, img);
        points = makeRotation(points, normal, 6);
    }
}

int main() {
    Mat img(1440, 2560, CV_8UC3, Scalar(255, 255, 255));

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
    // animateParallel(img, points, normal, 120);
    animateGouraud(img, points, normal, 120);
    
    return 0;
}
