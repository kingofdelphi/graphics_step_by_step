#include <SDL2/SDL.h>
#include <cmath>
#ifdef EMSCRIPTEN
 #include <emscripten.h>
#endif
#include <iostream>

using namespace std;

const int SCREEN_WIDTH = 640, SCREEN_HEIGHT = 480;
void setpixel(SDL_Surface * screen, int x, int y, Uint32 color) {
  if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) return ;
  Uint32 * pixels = (Uint32*)screen->pixels;
  pixels[y * SCREEN_WIDTH + x] = color;
}

struct Camera {
  //ze is the eye position = zprp
  //zv is the position of view plane
  //phi, theta are spherical angles
  //gama is used to rotate the world about z axis e.g. to see the world
  //upside down gama is set to 180 degree
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
    return rotateY(-cam.phi).rotateX(-cam.theta).rotateZ(-cam.gama);
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
  double magnitude() const {
    return sqrt(x * x + y * y + z * z);
  }
};
void line(SDL_Surface * screen, const Point & a, const Point & b, Uint32 color) {
  double dist = (b - a).magnitude();
  double theta = atan2(b.y - a.y, b.x - a.x);
  double fa = cos(theta), fb = sin(theta);
  for (double r = 0; r <= dist; r += 0.5) {
    double lx = a.x + fa * r, ly = a.y + fb * r;
    setpixel(screen, lx, ly, color);
  }
}
struct Cuboid {
  Point a, b, c, d, e, f, g, h;
  Cuboid(double ox, double oy, double oz, double length, double breadth, double height) {
    a = Point(ox, oy, oz);
    b = Point(ox + length, oy, oz);
    c = Point(ox + length, oy + breadth, oz);
    d = Point(ox, oy + breadth, oz);
    e = Point(ox, oy, oz + height);
    f = Point(ox + length, oy, oz + height);
    g = Point(ox + length, oy + breadth, oz + height);
    h = Point(ox, oy + breadth, oz + height);
  }
  void draw(SDL_Surface * screen, const Camera & cam) const {
    Point pa = a.toVC(cam).project(cam).to2dview();
    Point pb = b.toVC(cam).project(cam).to2dview();
    Point pc = c.toVC(cam).project(cam).to2dview();
    Point pd = d.toVC(cam).project(cam).to2dview();
    Point pe = e.toVC(cam).project(cam).to2dview();
    Point pf = f.toVC(cam).project(cam).to2dview();
    Point pg = g.toVC(cam).project(cam).to2dview();
    Point ph = h.toVC(cam).project(cam).to2dview();
    line(screen, pa, pb, 0xff0000);
    line(screen, pb, pc, 0xff0000);
    line(screen, pc, pd, 0xff0000);
    line(screen, pd, pa, 0xff0000);
    
    line(screen, pe, pf, 0xff0000);
    line(screen, pf, pg, 0xff0000);
    line(screen, pg, ph, 0xff0000);
    line(screen, ph, pe, 0xff0000);
    
    line(screen, pa, pe, 0xff0000);
    line(screen, pb, pf, 0xff0000);
    line(screen, pc, pg, 0xff0000);
    line(screen, pd, ph, 0xff0000);
    
  }

};

SDL_Window* window;
SDL_Surface *screen;
bool run = 1;
Cuboid cb(0, 0, 0, 50, 50, 50);
Camera camera;

void main_loop() {
  SDL_Event event;
  const Uint8 *keys = SDL_GetKeyboardState(NULL);
	while (SDL_PollEvent(&event)) {
		if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) run = 0;
	}
	//logic

	if (keys[SDL_SCANCODE_A]) camera.phi += 0.1;
	if (keys[SDL_SCANCODE_D]) camera.phi -= 0.1;

	if (keys[SDL_SCANCODE_Q]) camera.theta += 0.1;
	if (keys[SDL_SCANCODE_E]) camera.theta -= 0.1;

	if (keys[SDL_SCANCODE_LEFT]) camera.gama += 0.1;
	if (keys[SDL_SCANCODE_RIGHT]) camera.gama -= 0.1;

	const int delta = 2;
	if (keys[SDL_SCANCODE_Z]) camera.ze += delta, camera.zv += delta; //zoom in
	if (keys[SDL_SCANCODE_X]) camera.ze -= delta, camera.zv -= delta; //zoom out


	//rendering

	SDL_FillRect(screen, &screen->clip_rect, 0xFFFFFF);
	cb.draw(screen, camera);
	SDL_UpdateWindowSurface(window);
	SDL_Delay(10);
}

int main(int argc, char **argv)
{
  SDL_Init(SDL_INIT_EVERYTHING);
  window = SDL_CreateWindow("Graphics 3d", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
  screen = SDL_GetWindowSurface(window);

  #ifdef EMSCRIPTEN
    emscripten_set_main_loop(main_loop, 0, 1);
  #else
    while (run) {
  	  main_loop();
    }
  #endif
  SDL_Quit();
  return 0;
}
