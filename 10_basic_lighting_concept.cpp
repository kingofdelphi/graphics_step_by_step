#include <SDL2/SDL.h>
#include <cmath>
#include <algorithm>
#include <iostream>
using namespace std;
const int SCREEN_WIDTH = 640, SCREEN_HEIGHT = 480;
#define sign(x) ((x) >= 0 ? 1 : -1)
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
Camera camera; //camera is made global
//normalized color i.e. 0 <= r, g, b <= 1
struct Color {
    double r, g, b;
    Color(double ar = 0, double ag = 0, double ab = 0) : r(ar), g(ag), b(ab) { }
    Uint32 getColor() const {
        if (r < 0 || r > 1) throw "error";
        if (g < 0 || g > 1) throw "error";
        if (b < 0 || b > 1) throw "error";
        int rv = r * 255;
        int gv = g * 255;
        int bv = b * 255;
        rv &= 255;
        gv &= 255;
        bv &= 255;
        return (rv << 16) | (gv << 8) | bv;
    }
    Color operator*(const double v) const {
        return Color(r * v, g * v, b * v);
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
double interpolate(double a1, double b1, double a2, double b2, double a) {
    return b1 + (b2 - b1) * (a - a1) / (a2 - a1);
}
struct Point {
    double x, y, z;
    Color color;
    Point(double cx = 0, double cy = 0, double cz = 0, const Color & pcol = Color(1, 0, 0)):
        x(cx), y(cy), z(cz), color(pcol) {
        }
    //transformations
    Point rotateY(double t) const {
        double nz = z * cos(t) - x * sin(t);
        double nx = z * sin(t) + x * cos(t);
        return Point(nx, y, nz, color);
    }
    Point rotateZ(double t) const {
        double nx = x * cos(t) - y * sin(t);
        double ny = x * sin(t) + y * cos(t);
        return Point(nx, ny, z, color);
    }
    Point rotateX(double t) const {
        double ny = y * cos(t) - z * sin(t);
        double nz = y * sin(t) + z * cos(t);
        return Point(x, ny, nz, color);
    }
    //crude pipelining steps
    Point toVC(const Camera & cam) const {
        return rotateY(-cam.phi).rotateX(-cam.theta);
    }
    Point project(const Camera & cam) const {
        double f = (cam.ze - cam.zv) / (cam.ze - z);
        double xp = x * f, yp = y * f;
        return Point(xp, yp, cam.zv, color);
    }
    Point to2dview() const {
        return Point(x + SCREEN_WIDTH / 2, y + SCREEN_HEIGHT / 2, z, color);
    }
    //vector math
    Point operator-(const Point & b) const {
        return Point(x - b.x, y - b.y, z - b.z, color);
    }
    Point operator*(double factor) const {
        return Point(x * factor, y * factor, z * factor, color);
    }
    double operator*(const Point & b) const {
        return x * b.x + y * b.y + z * b.z;
    }
    Point operator+(const Point & b) const {
        return Point(x + b.x, y + b.y, z + b.z, color);
    }
    double distanceTo(const Point & b) const {
        return (*this - b).magnitude();
    }
    double magnitude() const {
        return sqrt(x * x + y * y + z * z);
    }
    Point normalize() const {
        double v = magnitude(); //assert v != 0
        return Point(x / v, y / v, z / v, color);
    }
    double crossZ(const Point & b) const {//returns only the z component of the cross product
        return x * b.y - y * b.x;
    }
    Point cross(const Point & b) const {
        double rx = y * b.z - z * b.y;
        double ry = -(x * b.z - z * b.x);
        double rz = x * b.y - y * b.x;
        return Point(rx, ry, rz, color);
    }
};
Color colorInterpolate(double a, const Color & cola, double b, const Color & colb, double pa) {
    double r = interpolate(a, cola.r, b, colb.r, pa);
    double g = interpolate(a, cola.g, b, colb.g, pa);
    double blue = interpolate(a, cola.b, b, colb.b, pa);
    return Color(r, g, blue);
}
void line(SDL_Surface * screen, const Point & a, const Point & b, Uint32 color) {
    double dist = (b - a).magnitude();
    double theta = atan2(b.y - a.y, b.x - a.x);
    double fa = cos(theta), fb = sin(theta);
    for (double r = 0; r <= dist; r += 0.1) {
        double lx = a.x + fa * r, ly = a.y + fb * r;
        setpixel(screen, lx, ly, color);
    }
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
        Point lsz = position.toVC(cam);
        double dist = pixel_pos.distanceTo(lsz);
        Color light_intensity = calculate_self_intensity(dist);
        double kd = 0.51; //coefficient of diffusion
        Point L = (lsz - pixel_pos).normalize(); //light normal
        double cosfact = N * L;
        if (cosfact < 0) {
            N = Point(0, 0, 0) - N; //invert the normal's direction
        }
        cosfact = abs(cosfact);
        Color diffuse = light_intensity * cosfact * kd ;
        //specular begin
        Point V = (Point(0, 0, camera.ze) - pixel_pos).normalize(); //since our camera is placed at (0, 0, ze)
        Point R = N * 2.0 * cosfact  - L;
        double fct = abs(V * R);
        const double ks = 0.31;
        const double ns = 200;
        Color specular(0, 0, 0);
        double K = ks * pow(fct, ns);
        specular = light_intensity * K;
        Color res(specular + diffuse);
        return res;
    }
} light_source(20, -10, -50, 1, 1, 1);

bool pointInsideTriangle(const Point & p, const Point & a, const Point & b, const Point & c) {
    double x = (p - a).crossZ(b - a);
    double y = (p - b).crossZ(c - b);
    double z = (p - c).crossZ(a - c);
    return (x >= 0 && y >= 0 && z >= 0) || (x < 0 && y < 0 && z < 0);
}
//assumes a, b, c are sorted according to y coordinates, i.e. a.y <= b.y <= c.y
//returns the scanline range according to the yposition
void getEndPoints
(const Point & a, const Point & b, const Point & c, double ypos, double & xstart, double & xend, Color & cola, Color & colb) {
    xstart = xend = 0;//dummy
    if (a.y == c.y) return ; //straight line with slope 0, do nothing
    if (a.y == b.y || ypos > b.y) {
        //intersection of y = ypos with AC and BC   
        double intac = interpolate(a.y, a.x, c.y, c.x, ypos);
        double intbc = interpolate(b.y, b.x, c.y, c.x, ypos);
        cola = colorInterpolate(a.y, a.color, c.y, c.color, ypos);
        colb = colorInterpolate(b.y, b.color, c.y, c.color, ypos);
        xstart = intac;
        xend = intbc;
    } else {
        double intab = interpolate(a.y, a.x, b.y, b.x, ypos);
        double intac = interpolate(a.y, a.x, c.y, c.x, ypos);
        cola = colorInterpolate(a.y, a.color, b.y, b.color, ypos);
        colb = colorInterpolate(a.y, a.color, c.y, c.color, ypos);
        xstart = intab;
        xend = intac;
    }
    if (xstart > xend) {
        swap(xstart, xend);
        swap(cola, colb);
    }
}
//z buffering implemented here
void triangleFill(SDL_Surface * screen, Point a, Point b, Point c, const Camera & cam) {
    Point vca = a.toVC(cam), vcb = b.toVC(cam), vcc = c.toVC(cam);
    Point normal = (vcb - vca).cross(vcc - vca); //obtain the triangle normal, i.e. a, b, c components of the plane
    Point normalized = normal.normalize();
    double d = -(normal.x * vca.x + normal.y * vca.y + normal.z * vca.z); // ax + by + cz + d = 0, d = -(ax + by + cz), put point vca
    Point pa = vca.project(cam).to2dview(), pb = vcb.project(cam).to2dview(), pc = vcc.project(cam).to2dview();
    pa.y = int(pa.y);
    pb.y = int(pb.y);
    pc.y = int(pc.y);
    if (pa.y > pb.y) swap(pa, pb);
    if (pb.y > pc.y) swap(pb, pc);
    if (pa.y > pb.y) swap(pa, pb);
    if (pa.y == pc.y) return ;
    double miny = min(pa.y, min(pb.y, pc.y));
    double maxy = max(pa.y, max(pb.y, pc.y));
    double minx = min(pa.x, min(pb.x, pc.x));
    double maxx = max(pa.x, max(pb.x, pc.x));
    for (int i = miny; i <= maxy; ++i) {
        if (i < 0) continue;
        if (i >= SCREEN_HEIGHT) break;
        bool inside = 0;
        double xs, xe;
        Color cols, cole;
        bool calculated = 0;
        bool samesign = 0;
        for (int j = max(minx, 0.0); j <= maxx; ++j) {
            if (j >= SCREEN_WIDTH) break;
            if (pointInsideTriangle(Point(j, i, 0), pa, pb, pc)) {
                if (!inside) {
                    getEndPoints(pa, pb, pc, i, xs, xe, cols, cole);
                    inside = 1;
                }
                double D = cam.ze - cam.zv;
                double xp = j - SCREEN_WIDTH / 2, yp = i - SCREEN_HEIGHT / 2;
                double F = normal.x * xp + normal.y * yp - normal.z * D;
                double point_z = ((normal.x * xp + normal.y * yp) * cam.ze + d * D) / F;
                Color current = colorInterpolate(xs, cols, xe, cole, j);
                if (point_z < z_buffer[i][j]) {
                    z_buffer[i][j] = point_z;
                    Color dcol(0.2, 0.2, 0.2); // ambient ligp ht
                    double f = (cam.ze - point_z) / D;
                    Point vwcPos(xp * f, yp * f, point_z); //use inverse projection to calculate the viewing coordinates of the pixel
                    //only if the light is on the same side as that of camera wrt to this pixels' viewing coordinate point
                    //we add the contribution to dcol
                    if (!calculated) {
                        calculated = 1;
                        Point lght = light_source.position.toVC(cam);
                        double diff1 = (normal * lght) + d;
                        double diff2 = normal.z * cam.ze + d;
                        samesign = sign(diff1) == sign(diff2);
                    }
                    if (samesign) { //necessary condition for two points to lie on the same side of a plane
                        dcol = dcol + light_source.calculateLightIntensity(normalized, vwcPos, cam);
                    }
                    if (dcol.r > 1) dcol.r = 1;
                    if (dcol.g > 1) dcol.g = 1;
                    if (dcol.b > 1) dcol.b = 1;
                    current.r *= dcol.r;
                    current.g *= dcol.g;
                    current.b *= dcol.b;
                    setpixel(screen, j, i, current.getColor());
                }
            } else if (inside) break;
        }
    }
}
struct Cuboid {
    Point a, b, c, d, e, f, g, h;
    Cuboid(double ox, double oy, double oz, double length, double breadth, double height, const Color & cola, const Color & colb, const Color & colc, const Color & cold, const Color & cole, const Color & colf, const Color & colg, const Color & colh) {
        a = Point(ox, oy, oz, cola);
        b = Point(ox + length, oy, oz, colb);
        c = Point(ox + length, oy + breadth, oz, colc);
        d = Point(ox, oy + breadth, oz, cold);
        e = Point(ox, oy, oz + height, cole);
        f = Point(ox + length, oy, oz + height, colf);
        g = Point(ox + length, oy + breadth, oz + height, colg);
        h = Point(ox, oy + breadth, oz + height, colh);
    }
    void draw(SDL_Surface * screen, const Camera & cam) const {
        triangleFill(screen, a, b, c, cam);
        triangleFill(screen, a, d, c, cam);
        triangleFill(screen, e, f, g, cam);
        triangleFill(screen, e, h, g, cam);

        triangleFill(screen, a, d, e, cam);
        triangleFill(screen, h, d, e, cam);

        triangleFill(screen, b, f, g, cam);
        triangleFill(screen, c, b, g, cam);

        triangleFill(screen, a, e, b, cam);
        triangleFill(screen, e, b, f, cam);
    }

};
void clearZBuffer() {
    for (int i =  0; i < SCREEN_HEIGHT; ++i) {
        for (int j = 0; j < SCREEN_WIDTH; ++j) z_buffer[i][j] = 999999999.0; //infinity
    }
}

int main(int argc, char ** argv) {
    SDL_Init(SDL_INIT_EVERYTHING);
 	SDL_Window* window = SDL_CreateWindow("Graphics 3d", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
	SDL_Surface *screen = SDL_GetWindowSurface(window);
    SDL_Event event;
    bool run = 1;
    Cuboid cb(0, 0, 0, 50, -50, 100, Color(1, 0, 0), Color(1, 0, 0), Color(1, 0, 0), Color(1, 0, 0), Color(1, 0, 0), Color(1, 0, 0), Color(1, 0, 0), Color(1, 0, 0));
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
        cb.draw(screen, camera);
        Cuboid lght(light_source.position.x, light_source.position.y, light_source.position.z, 5, -5, 5, Color(0, 1, 0), Color(1, 0, 1), Color(1, 0, 0), Color(1, 0, 0), Color(1, 0, 0), Color(1, 0, 0), Color(1, 0, 0), Color(1, 0, 0));
        lght.draw(screen, camera);

        SDL_UpdateWindowSurface(window);
        SDL_Delay(10);
    }
    SDL_Quit();
}
