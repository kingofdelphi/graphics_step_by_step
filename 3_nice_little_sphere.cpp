#include <SDL2/SDL.h>
#include <cmath>
#include <algorithm>
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
struct Point {
  double x, y, z;
  Point(double cx = 0, double cy = 0, double cz = 0):
    x(cx), y(cy), z(cz) { }
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
  Point operator+(const Point & b) const {
    return Point(x + b.x, y + b.y, z + b.z);
  }

  double magnitude() const {
    return sqrt(x * x + y * y + z * z);
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
  double dot(const Point & b) const {
    return x * b.x + y * b.y + z * b.z;
  }
};
void line(SDL_Surface * screen, const Point & a, const Point & b, Uint32 color) {
  double dist = (b - a).magnitude();
  double theta = atan2(b.y - a.y, b.x - a.x);
  double fa = cos(theta), fb = sin(theta);
  for (double r = 0; r <= dist; r += 0.1) {
    double lx = a.x + fa * r, ly = a.y + fb * r;
    setpixel(screen, lx, ly, color);
  }
}
bool pointInsideTriangle(const Point & p, const Point & a, const Point & b, const Point & c) {
  double x = (p - a).crossZ(b - a);
  double y = (p - b).crossZ(c - b);
  double z = (p - c).crossZ(a - c);
  return (x >= 0 && y >= 0 && z >= 0) || (x < 0 && y < 0 && z < 0);
}
//z buffering implemented here
void triangleFill(SDL_Surface * screen, const Point & a, const Point & b, const Point & c, Uint32 color, const Camera & cam) {
  Point vca = a.toVC(cam), vcb = b.toVC(cam), vcc = c.toVC(cam);
  Point normal = (vcb - vca).cross(vcc - vca); //obtain the triangle normal, i.e. a, b, c components of the plane
  double d = -vca.dot(normal); // ax + by + cz + d = 0, d = -(ax + by + cz), put point vca
  Point pa = vca.project(cam).to2dview(), pb = vcb.project(cam).to2dview(), pc = vcc.project(cam).to2dview();
  double minx = min(pa.x, min(pb.x, pc.x)), miny = min(pa.y, min(pb.y, pc.y));
  double maxx = max(pa.x, max(pb.x, pc.x)), maxy = max(pa.y, max(pb.y, pc.y));
  for (int i = miny; i <= maxy; ++i) {
    if (i < 0) continue;
    if (i >= SCREEN_HEIGHT) break;
    bool inside = 0;
    for (int j = max(minx, 0.0); j <= maxx; ++j) {
      if (j >= SCREEN_WIDTH) break;
      if (pointInsideTriangle(Point(j, i, 0), pa, pb, pc)) {//some improvements possible
	inside = 1;
	double D = cam.ze - cam.zv;
	double xp = j - SCREEN_WIDTH / 2, yp = i - SCREEN_HEIGHT / 2;
	double F = normal.x * xp + normal.y * yp - normal.z * D;
	double point_z = ((normal.x * xp + normal.y * yp) * cam.ze + d * D) / F;
	if (point_z < z_buffer[i][j]) {
	  z_buffer[i][j] = point_z;
	  setpixel(screen, j, i, color);
	}
      } else if (inside) break;
    }
  }
}
void clearZBuffer() {
  for (int i =  0; i < SCREEN_HEIGHT; ++i) {
    for (int j = 0; j < SCREEN_WIDTH; ++j) z_buffer[i][j] = 999999999.0; //infinity
  }
}
//draws a sphere of given radius centered at given point, uses spherical coordinates for calculating position of a point on the sphere
//look the sphere image file to see how it is done, turn up the pages of your EM book :D
void drawSphere(SDL_Surface * screen, const Point & center, double radius, const Camera & cam) {
  int phi_steps = 20, theta_steps = 20;
  double ang_phi = 2 * M_PI / phi_steps, ang_theta = M_PI / theta_steps;
  Point r(0, 0, radius);
  for (int i = 0; i < phi_steps; ++i) {
    double theta = -M_PI / 2.0;
    for (int j = 0; j < theta_steps; ++j) {
      Point a = r.rotateX(theta).rotateY(i * ang_phi);
      Point b = r.rotateX(theta).rotateY(i * ang_phi + ang_phi);
      Point c = r.rotateX(theta + ang_theta).rotateY(i * ang_phi);
      Point d = r.rotateX(theta + ang_theta).rotateY(i * ang_phi + ang_phi);
      triangleFill(screen, a + center, b + center, c + center, 0xff0000, cam);
      triangleFill(screen, d + center, b + center, c + center, 0x00ff00, cam);
      theta += ang_theta;
    }
  }
}

int main(int argc, char ** argv) {
  SDL_Init(SDL_INIT_EVERYTHING);
 	SDL_Window* window = SDL_CreateWindow("Graphics 3d", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
  SDL_Surface *screen = SDL_GetWindowSurface(window);
  SDL_Event event;

  bool run = 1;
  Camera camera;
  const Uint8 *keys = SDL_GetKeyboardState(0);

  while (run) {
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDL_SCANCODE_ESCAPE)) run = 0;
    }
    //logic
    if (keys[SDL_SCANCODE_A]) camera.phi += 0.1;
    if (keys[SDL_SCANCODE_D]) camera.phi -= 0.1;
    
    if (keys[SDL_SCANCODE_Q]) camera.theta += 0.1;
    if (keys[SDL_SCANCODE_E]) camera.theta -= 0.1;
    const int delta = 2;
    if (keys[SDL_SCANCODE_Z]) camera.ze += delta, camera.zv += delta; //zoom in
    if (keys[SDL_SCANCODE_X]) camera.ze -= delta, camera.zv -= delta; //zoom out
    
    //rendering
    SDL_FillRect(screen, &screen->clip_rect, 0xFFFFFF);
    clearZBuffer();
    drawSphere(screen, Point(0, 0, 0), 100, camera);
    SDL_UpdateWindowSurface(window);
  }
  SDL_Quit();
  return 0;
}
