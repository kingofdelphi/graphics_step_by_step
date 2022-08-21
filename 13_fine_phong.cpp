#include <SDL2/SDL.h>
#include <cmath>
#include <algorithm>
#define sign(x) ((x) >= 0 ? 1 : -1)
using namespace std;
const int SCREEN_WIDTH = 640, SCREEN_HEIGHT = 480;
void setpixel(SDL_Surface * screen, int x, int y, Uint32 color) {
  if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) return ;
  Uint32 * pixels = (Uint32*)screen->pixels;
  pixels[y * SCREEN_WIDTH + x] = color;
}
double z_buffer[SCREEN_HEIGHT][SCREEN_WIDTH];
struct Camera {
  double ze, zv, phi, theta, gama;
  Camera() : ze(-500), zv(-200), phi(0), theta(0), gama(0) { }
};
struct Color {
    double r, g, b;
    Color(double ar = 0, double ag = 0, double ab = 0) : r(ar), g(ag), b(ab) { }
    Uint32 getColor() {
        r = min(1.0, max(r, 0.0));
        g = min(1.0, max(g, 0.0));
        b = min(1.0, max(b, 0.0));
        int rv = r * 255;
        int gv = g * 255;
        int bv = b * 255;
        return (rv << 16) | (gv << 8) | bv;
    }
    Color operator*(const double v) const {
        return Color(r * v, g * v, b * v);
    }
    Color operator*(const Color & s) const {
        return Color(r * s.r, g * s.g, b * s.b);
    }
    static void boundVariable(double & val) {
        if (val > 1) val = 1;
        if (val < 0) val = 0; 
    }
    Color bound() const {
        Color res = *this;
        boundVariable(res.r);
        boundVariable(res.g);
        boundVariable(res.b);
        return res;
    }
    Color operator+(const Color & op2) const {
        return Color(r + op2.r, g + op2.g, b + op2.b);
    }
};
struct Point {
    double x, y, z;
    Point(double cx = 0, double cy = 0, double cz = 0):
        x(cx), y(cy), z(cz) {
        }
    //transformations
    Point rotateY(double t) const {
        double nz = z * cos(t) - x * sin(t);
        double nx = z * sin(t) + x * cos(t);
        return Point(nx, y, nz);
    }
    Point rotateZ(double t) const {
        double nx = x * cos(t) - y * sin(t);
        double ny = x * sin(t) + y * cos(t);
        return Point(nx, ny, z);
    }
    Point rotateX(double t) const {
        double ny = y * cos(t) - z * sin(t);
        double nz = y * sin(t) + z * cos(t);
        return Point(x, ny, nz);
    }
    //crude pipelining steps
    Point toVC(const Camera & cam) const {
        return rotateY(-cam.phi).rotateX(-cam.theta);
    }
    Point project(const Camera & cam) const {
        double f = (cam.ze - cam.zv) / (cam.ze - z);
        double xp = x * f, yp = y * f;
        return Point(xp, yp, cam.zv);
    }
    Point to2dview() const {
        return Point(x + SCREEN_WIDTH / 2, y + SCREEN_HEIGHT / 2, z);
    }
    //vector math
    Point operator-(const Point & b) const {
        return Point(x - b.x, y - b.y, z - b.z);
    }
    Point operator*(double factor) const {
        return Point(x * factor, y * factor, z * factor);
    }
    double operator*(const Point & b) const {
        return x * b.x + y * b.y + z * b.z;
    }
    Point operator+(const Point & b) const {
        return Point(x + b.x, y + b.y, z + b.z);
    }
    double distanceTo(const Point & b) const {
        return (*this - b).magnitude();
    }
    double magnitude() const {
        return sqrt(x * x + y * y + z * z);
    }
    Point normalize() const {
        double v = magnitude(); //assert v != 0
        return Point(x / v, y / v, z / v);
    }
    double crossZ(const Point & b) const {//returns only the z component of the cross product
        return x * b.y - y * b.x;
    }
    Point cross(const Point & b) const {
        double rx = y * b.z - z * b.y;
        double ry = -(x * b.z - z * b.x);
        double rz = x * b.y - y * b.x;
        return Point(rx, ry, rz);
    }
};
struct Vertex {
    Point pos, normal;
    Vertex(Point p, Point n) : pos(p), normal(n) { }
    Vertex toVC(const Camera & cam) const {
        return Vertex(pos.toVC(cam), normal.toVC(cam));
    }
    Vertex to2dview() const {
        return Vertex(pos.to2dview(), normal);
    }
    Vertex project(const Camera & cam) const {
        return Vertex(pos.project(cam), normal);
    }
};
double interpolate(double a1, double b1, double a2, double b2, double a) {
    return b1 + (b2 - b1) * (a - a1) / (a2 - a1);
}
struct LightSource {
    Point position;
    double ir, ig, ib;
    LightSource(double ax = 0, double ay = 0, double az = 0, double r = 0, double g = 0, double b = 0)
        : ir(r), ig(g), ib(b) {
            position = Point(ax, ay, az);
        }
    Color calculate_self_intensity(double dist) const {
        static const double MAX_DIST = 1000;
        double res = 1 - dist / MAX_DIST;
        if (res < 0) res = 0;
        return Color(ir, ig, ib) * res;
    }
    //returns normalized color
    Color calculateLightIntensity(Point N, const Point & pixel_pos, const Camera & cam) const {
        N = N.normalize();
        Point lsz = position.toVC(cam);
        double dist = pixel_pos.distanceTo(lsz);
        Color light_intensity = calculate_self_intensity(dist);
        double kd = 0.51; //coefficient of diffusion
        Point L = (lsz - pixel_pos).normalize(); //light normal
        double cosfact = N * L;
        if (cosfact < 0) return Color(0, 0, 0);
        cosfact = abs(cosfact);
        Color diffuse = light_intensity * cosfact * kd ;
        //specular begin
        Point V = (Point(0, 0, cam.ze) - pixel_pos).normalize(); //since our camera is placed at (0, 0, ze)
        Point R = N * 2.0 * cosfact  - L;
        double fct = abs(V * R);
        const double ks = 0.51;
        const double ns = 200;
        Color specular(0, 0, 0);
        double K = ks * pow(fct, ns);
        specular = light_intensity * K;
        Color res(specular + diffuse);
        return res;
    }
} light_source(20, -10, -50, 1, 1, 1);
Point normalInterpolate(double a, const Point & na, double b, const Point & nb, double pa) {
    double x = interpolate(a, na.x, b, nb.x, pa);
    double y = interpolate(a, na.y, b, nb.y, pa);
    double z = interpolate(a, na.z, b, nb.z, pa);
    return Point(x, y, z);
}
//normal is the surface normal used for calculating z for z buffering
//n1, n2 are the normals at x1 and x2 respectively
void scanline(SDL_Surface * screen, int x1, int x2, int y, double d, Point normal, Point n1, Point n2, Color color, const Camera & cam) {
    if (x1 > x2) {
        swap(x1, x2);
        swap(n1, n2);
    }
    static const Color ambient(0.2, 0.2, 0.2);
    for (int j = x1; j <= x2; ++j) {
        double D = cam.ze - cam.zv;
        double xp = j - SCREEN_WIDTH / 2, yp = y - SCREEN_HEIGHT / 2;
        double F = normal.x * xp + normal.y * yp - normal.z * D;
        double point_z = ((normal.x * xp + normal.y * yp) * cam.ze + d * D) / F;
        Point ncurrent = x1 == x2 ? n1 : normalInterpolate(x1, n1, x2, n2, j);
        if (point_z < z_buffer[y][j]) {
            z_buffer[y][j] = point_z;
            double f = (cam.ze - point_z) / D;
            Point ppos(xp * f, yp * f, point_z);
            Color col = color * (ambient + light_source.calculateLightIntensity(ncurrent, ppos, cam));
            setpixel(screen, j, y, col.getColor());
        }
    }
}
//z buffering implemented here
//parameters a b c normal are in world space
void triangleFill(SDL_Surface * screen, const Vertex & a, const Vertex & b, const Vertex & c, Point normal, Color color, const Camera & cam) {
    normal = normal.toVC(cam);
    Vertex vca = a.toVC(cam), vcb = b.toVC(cam), vcc = c.toVC(cam);
    double d = -(normal.x * vca.pos.x + normal.y * vca.pos.y + normal.z * vca.pos.z); // ax + by + cz + d = 0, d = -(ax + by + cz), put point vca
    Vertex pa = vca.project(cam).to2dview(), pb = vcb.project(cam).to2dview(), pc = vcc.project(cam).to2dview();
    if (pa.pos.y > pb.pos.y) swap(pa, pb);
    if (pb.pos.y > pc.pos.y) swap(pb, pc);
    if (pa.pos.y > pb.pos.y) swap(pa, pb);
    pa.pos.y = int(pa.pos.y);
    pb.pos.y = int(pb.pos.y);
    pc.pos.y = int(pc.pos.y);
    if (pa.pos.y == pc.pos.y) return ;
    double x1 = pa.pos.x, x2 = pa.pos.x;
    double m2 = (pc.pos.x - pa.pos.x) / (pc.pos.y - pa.pos.y);
    int dy = (pb.pos.y - pa.pos.y);
    if (pa.pos.y != pb.pos.y) {
        double m1 = (pb.pos.x - pa.pos.x) / (pb.pos.y - pa.pos.y);
        for (int i = pa.pos.y; i <= pb.pos.y; i++) {
            Point n1 = normalInterpolate(pa.pos.y, pa.normal, pb.pos.y, pb.normal, i);
            Point n2 = normalInterpolate(pa.pos.y, pa.normal, pc.pos.y, pc.normal, i);
            scanline(screen, x1, x2, i, d, normal, n1, n2, color, cam);
            x1 += m1;
            x2 += m2;
        }
    }
    if (pb.pos.y != pc.pos.y) {
        double m1 = (pc.pos.x - pb.pos.x) / (pc.pos.y - pb.pos.y);
        x1 = pb.pos.x + m1;
        for (int i = pb.pos.y + 1; i <= pc.pos.y; i++) {
            Point n1 = normalInterpolate(pb.pos.y, pb.normal, pc.pos.y, pc.normal, i);
            Point n2 = normalInterpolate(pa.pos.y, pa.normal, pc.pos.y, pc.normal, i);
            scanline(screen, x1, x2, i, d, normal, n1, n2, color, cam);
            x1 += m1;
            x2 += m2;
        }
    }
}
void drawSphere(SDL_Surface * screen, const Point & center, double radius, const Camera & cam) {
    int phi_steps = 10, theta_steps = 10;
    Color color(1, 0, 0); //sphere color
    Point vertices[phi_steps + 1][theta_steps + 1];
    double ang_phi = 2 * M_PI / phi_steps, ang_theta = M_PI / theta_steps;
    Point r(0, 0, radius);
    for (int i = 0; i <= phi_steps; ++i) {
        double theta = -M_PI / 2.0;
        for (int j = 0; j <= theta_steps; ++j) {
            vertices[i][j] = r.rotateX(theta).rotateY(i * ang_phi) + center;
            theta += ang_theta;
        }
    }
    Point normal[phi_steps + 1][theta_steps + 1];
    for (int i = 0; i < phi_steps; ++i) {
        for (int j = 0; j < theta_steps; ++j) {
            Point norm = (vertices[i][j + 1] - vertices[i][j]).cross(vertices[i + 1][j] - vertices[i][j]);
            normal[i][j] = normal[i][j] + norm;
            normal[i + 1][j] = normal[i + 1][j] + norm;
            normal[i][j + 1] = normal[i][j + 1] + norm;
            normal[i + 1][j + 1] = normal[i + 1][j + 1] + norm;
        }
    }
    for (int i = 0; i < phi_steps; ++i) {
        for (int j = 0; j < theta_steps; ++j) {
            Point norm = (vertices[i][j + 1] - vertices[i][j]).cross(vertices[i + 1][j] - vertices[i][j]);
            Vertex va(vertices[i][j], normal[i][j]);
            Vertex vb(vertices[i + 1][j], normal[i + 1][j]);
            Vertex vc(vertices[i][j + 1], normal[i][j + 1]);
            Vertex vd(vertices[i + 1][j + 1], normal[i + 1][j + 1]);
            triangleFill(screen, va, vb, vc, norm, color, cam);
            triangleFill(screen, vb, vc, vd, norm, color, cam);
        }
    }
}
void clearZBuffer() {
  for (int i =  0; i < SCREEN_HEIGHT; ++i) {
    for (int j = 0; j < SCREEN_WIDTH; ++j) z_buffer[i][j] = 999999999.0; //infinity
  }
}

int main(int argc, char ** argv) {
  SDL_Init(SDL_INIT_EVERYTHING);
  SDL_Window *window = SDL_CreateWindow("Graphics 3d", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
  SDL_Surface *screen = SDL_GetWindowSurface(window);
  SDL_Event event;
  bool run = 1;
  Camera camera;
  while (run) {
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) run = 0;
    }
    //logic
    const Uint8 *keys = SDL_GetKeyboardState(0);
    if (keys[SDL_SCANCODE_A]) camera.phi += 0.1;
    if (keys[SDL_SCANCODE_D]) camera.phi -= 0.1;
    
    if (keys[SDL_SCANCODE_Q]) camera.theta += 0.1;
    if (keys[SDL_SCANCODE_E]) camera.theta -= 0.1;
    const int delta = 2;
    if (keys[SDL_SCANCODE_Z]) camera.ze += delta, camera.zv += delta; //zoom in
    if (keys[SDL_SCANCODE_X]) camera.ze -= delta, camera.zv -= delta; //zoom out
    
    if (keys[SDL_SCANCODE_K]) light_source.position.z += 1;
    if (keys[SDL_SCANCODE_J]) light_source.position.z -= 1;
    
    //rendering
    SDL_FillRect(screen, &screen->clip_rect, 0);
    clearZBuffer();
    drawSphere(screen, Point(0, 0, 0), 50, camera);
    SDL_UpdateWindowSurface(window);
  }
  SDL_Quit();
  return 0;
}
