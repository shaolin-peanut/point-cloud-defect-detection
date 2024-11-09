#include <iostream>
#include <vector>
#include <GL/glut.h>
#include <liblas/liblas.hpp>
#include <fstream>

float camX = 0.0f, camY = 0.0f, camZ = 20.0f;
float camAngleX = 0.0f, camAngleY = 0.0f;

struct Point {
    float x, y, z;
    uint8_t classification;  // Classification code for color mapping
};

std::vector<Point> pointCloud;

void loadPointCloud(const std::string& filename) {
    std::ifstream ifs(filename, std::ios::in | std::ios::binary);
    liblas::ReaderFactory readerFactory;
    liblas::Reader reader = readerFactory.CreateWithStream(ifs);
    pointCloud.clear();
    float scale = 0.0001f;
    // float scale = 1.0f;

    while (reader.ReadNextPoint()) {
        const liblas::Point& p = reader.GetPoint();
        float x = (p.GetX() - 2600000) * scale;
        float y = (p.GetY() - 1200000) * scale;
        float z = p.GetZ() * 0.001f;
        uint8_t classification = p.GetClassification().GetClass();
        pointCloud.push_back({x, y, z, classification});
    }
    std::cout << "Loaded " << pointCloud.size() << " points from " << filename << "." << std::endl;
}

void setColorForClassification(uint8_t classification) {
    switch (classification) {
        case 2:  // Ground
            glColor3f(0.6f, 0.3f, 0.1f);  // Brown
            break;
        case 5:  // Vegetation
            glColor3f(0.1f, 0.8f, 0.1f);  // Green
            break;
        case 6:  // Building
            // glColor3f(0.5f, 0.5f, 0.5f);  // Gray
            // yellow
            glColor3f(0.8f, 0.8f, 0.0f); // yellow
            break;
        default: // Unclassified or other
            glColor3f(1.0f, 1.0f, 1.0f);  // White
            break;
    }
}

void renderColoredPointCloud() {
    glPointSize(2.0f);
    glBegin(GL_POINTS);
    for (const auto& point : pointCloud) {
        setColorForClassification(point.classification);
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();
}

void handleKeypress(unsigned char key, int x, int y) {
    float speed = 0.1f;
    switch (key) {
        case 'w': camZ -= speed; break;
        case 's': camZ += speed; break;
        case 'a': camX -= speed; break;
        case 'd': camX += speed; break;
        case 'q': camY -= speed; break;
        case 'e': camY += speed; break;
        case 27: exit(0); break;  // Escape key to exit
    }
    std::cout << "Key pressed: " << key << " | Camera Position: ("
              << camX << ", " << camY << ", " << camZ << ")\n";
    glutPostRedisplay();
}

void setPerspective() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // gluPerspective(45.0f, 1.0f, 0.1f, 100.0f);
    gluPerspective(60.0f, 1.0f, 0.1f, 500.0f);
    glMatrixMode(GL_MODELVIEW);
}

void initOpenGL() {
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
    glPointSize(3.0f);
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    setPerspective();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // Set up the camera
    glTranslatef(-camX, -camY, -camZ);
    glRotatef(camAngleX, 1.0f, 0.0f, 0.0f);
    glRotatef(camAngleY, 0.0f, 1.0f, 0.0f);

    // Render the point cloud
    renderColoredPointCloud();

    glutSwapBuffers();
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path to las file>" << std::endl;
        return 1;
    }

    loadPointCloud(argv[1]);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("3D Point Cloud Viewer with Classification Colors");

    initOpenGL();
    glutDisplayFunc(display);
    glutKeyboardFunc(handleKeypress);
    glutReshapeFunc(reshape);

    glutMainLoop();

    return 0;
}