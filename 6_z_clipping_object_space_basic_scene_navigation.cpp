//demonstrates basic clipping on near z plane so that you can move through objects
//you'll have to interpolate the color as well during clipping if color information is
//different for each vertex but that would be trivial to implement(shown later) 
//
#include <SDL2/SDL.h>
#include <cmath>
#include <algorithm>
#include <iostream>
using namespace std;
const int SCREEN_WIDTH = 640, SCREEN_HEIGHT = 480;

//the position of the near clipping relative to the position of eye (ze)
//i.e. z_near = eye_z + DISTANCE_TO_EYE
//try placing this far from the eye and you will notice objects getting clipped
const double DISTANCE_TO_EYE = 100;
void setpixel(SDL_Surface * screen, int x, int y, Uint32 color) {
  if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) return ;
  Uint32 * pixels = (Uint32*)screen->pixels;
  pixels[y * SCREEN_WIDTH + x] = color;
}
double z_buffer[SCREEN_HEIGHT][SCREEN_WIDTH];
struct Camera ; //forward declaration
struct Point {
  double x, y, z;
  Point(double cx = 0, double cy = 0, double cz = 0):
    x(cx), y(cy), z(cz) { }
  //transformations
  Point translate(const Point & p) const {
    return Point(x + p.x, y + p.y, z + p.z);
  }
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

  Point toVC(const Camera & cam) const; // defined after the definition of Camera struct
  Point project(const Camera & cam) const; //defined after the definition of Camera struct
  
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

struct Camera {
  double ze, zv, phi, theta, gama;
  Point lookAt;//by default is origin
  Camera() : ze(-500), zv(-200), phi(0), theta(0), gama(0) { }
  //moves the camera lookAt position in the direction opposite to the camera normal by delta
  //this can also be done using basic rotation about axes or one can implement it using vector math
  //by obtaining the camera normal, invert the direction or normal, normalize the normal and then set
  //lookAt = lookAt + delta * camera_unit_dir
  //but you would have to first convert the spherical camera coordinates (r, phi, theta) into cartesian
  //to obtain the camera normal
  //this function first aligns the camera normal vector along the -ve z axis, as a result look at point will lie on the
  //z axis(camera normal is the vector drawn from lookAt towards the eye position(ze, phi, theta) , 
  //then increase z of lookAt by delta then undo the rotations
  void moveLookAt(double delta) {
    lookAt = (lookAt.rotateY(-phi).rotateX(-theta).translate(Point(0, 0, delta))).rotateX(theta).rotateY(phi);
  }
};

//-------------------------------------------------CRUDE PIPELINING STEPS----------------------------------------

Point Point::toVC(const Camera & cam) const {
  Point t(-cam.lookAt.x, -cam.lookAt.y, -cam.lookAt.z);  
  //first translate the lookAt point to the origin and then apply transformation steps as in previous tutorial
  return translate(t).rotateY(-cam.phi).rotateX(-cam.theta);
}

Point Point::project(const Camera & cam) const { //line plane intersection
  double f = (cam.ze - cam.zv) / (cam.ze - z);
  double xp = x * f, yp = y * f;
  return Point(xp, yp, cam.zv);
}

//---------------------------------------------------------------------------------------------------------------

//replace with bresenham to make it a whole lot faster
void line(SDL_Surface * screen, const Point & a, const Point & b, Uint32 color) {
  double dist = (b - a).magnitude();
  double theta = atan2(b.y - a.y, b.x - a.x);
  double fa = cos(theta), fb = sin(theta);
  for (double r = 0; r <= dist; r += 0.1) {
    double lx = a.x + fa * r, ly = a.y + fb * r;
    setpixel(screen, lx, ly, color);
  }
}

//used during object space triangle clipping
Point linePlaneIntersection(const Point & a, const Point & b, double z) {
  double u = (z - a.z) / (b.z - a.z); //assert a.z != b.z
  return Point(a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), z);
}

//a, b => world space;
void clipAndDraw(SDL_Surface * screen, Point a, Point b, Uint32 color, const Camera & cam) {
  a = a.toVC(cam);
  b = b.toVC(cam);
  if (a.z > b.z) swap(a, b);
  double znear = cam.ze + DISTANCE_TO_EYE;
  if (b.z < znear) return ;
  if (a.z < znear) {
    a = linePlaneIntersection(a, b, znear);
  }
  line(screen, a.project(cam).to2dview(), b.project(cam).to2dview(), color);
}


bool pointInsideTriangle(const Point & p, const Point & a, const Point & b, const Point & c) {
  double x = (p - a).crossZ(b - a);
  double y = (p - b).crossZ(c - b);
  double z = (p - c).crossZ(a - c);
  return (x >= 0 && y >= 0 && z >= 0) || (x < 0 && y < 0 && z < 0);
}
//z buffering implemented here
//CAUTION: now the function takes coordinates in camera space / viewing coordinates instead of world space
void triangleFill(SDL_Surface * screen, const Point & vca, const Point & vcb, const Point & vcc, Uint32 color, const Camera & cam) {
  Point normal = (vcb - vca).cross(vcc - vca); //obtain the triangle normal, i.e. a, b, c components of the plane
  double d = -(normal.x * vca.x + normal.y * vca.y + normal.z * vca.z); // ax + by + cz + d = 0, d = -(ax + by + cz), put point vca
  Point pa = vca.project(cam).to2dview(), pb = vcb.project(cam).to2dview(), pc = vcc.project(cam).to2dview();
  double minx = min(pa.x, min(pb.x, pc.x)), miny = min(pa.y, min(pb.y, pc.y));
  double maxx = max(pa.x, max(pb.x, pc.x)), maxy = max(pa.y, max(pb.y, pc.y));
  for (int i = miny; i <= maxy; ++i) {
    if (i < 0) continue;
    if (i >= SCREEN_HEIGHT) break;
    bool inside = 0;
    for (int j = max(minx, 0.0); j <= maxx; ++j) {
      if (j >= SCREEN_WIDTH) break;
      if (pointInsideTriangle(Point(j, i, 0), pa, pb, pc)) {
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

//CAUTION: takes coordinates in world space
//function maps the world coordinates to viewing coordinates and
//before projecting the triangle onto the screen, the necessary near z clipping is done

void ZClipTriangleFill(SDL_Surface * screen, const Point & a, const Point & b, const Point & c, Uint32 color, const Camera & cam) {
  Point vca = a.toVC(cam), vcb = b.toVC(cam), vcc = c.toVC(cam);
  //sort the points according to z coordinate, bubble sort
  if (vca.z > vcb.z) swap(vca, vcb);
  if (vcb.z > vcc.z) swap(vcb, vcc);
  if (vca.z > vcb.z) swap(vca, vcb);
  double z_near = cam.ze + DISTANCE_TO_EYE;
  if (vcc.z < z_near) return ; //the triangle is outside the near clipping plane, so no need to render it
  if (vca.z >= z_near) {//if vca.z >= z_near, no need to clip, directly render the single triangle
    triangleFill(screen, vca, vcb, vcc, color, cam);
  } else {//core
    if (vcb.z < z_near) { //if only one point i.e. vcc is inside, we obtain a triangle, vcb.z != vcc.z && vca.z != vcc.z, as vcc.z >= z_near
      //and vcb.z < znear, which also proves that vca.z < znear
      vcb = linePlaneIntersection(vcb, vcc, z_near);
      vca = linePlaneIntersection(vca, vcc, z_near);
      triangleFill(screen, vca, vcb, vcc, color, cam);
    } else { //only a is outside, vcb.z >= znear, vca.z < z_near, vcc.z >= z_near, so no divide by zero case
      Point ab = linePlaneIntersection(vca, vcb, z_near);
      Point ac = linePlaneIntersection(vca, vcc, z_near);
      triangleFill(screen, ab, vcb, vcc, color, cam);
      triangleFill(screen, ab, ac, vcc, color, cam);
    }
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
    ZClipTriangleFill(screen, a, b, c, 0xff0000, cam);
    ZClipTriangleFill(screen, a, d, c, 0xff0000, cam);
    ZClipTriangleFill(screen, e, f, g, 0x00ff00, cam);
    ZClipTriangleFill(screen, e, h, g, 0x00ff00, cam);
    ZClipTriangleFill(screen, a, d, e, 0xff00ff, cam);
    ZClipTriangleFill(screen, h, d, e, 0xff00ff, cam);
    ZClipTriangleFill(screen, b, f, g, 0, cam);
    ZClipTriangleFill(screen, c, b, g, 0, cam);
    ZClipTriangleFill(screen, a, e, b, 0x0000ff, cam);
    ZClipTriangleFill(screen, e, b, f, 0x0000ff, cam);
  }
};
void drawGridLines(SDL_Surface * screen, const Camera & cam) {
  double box_size = 50;
  static const double ZL = 500;
  for (int i = -10; i <= 10; ++i) {
    Point a(i * box_size, 0, -ZL), b(i * box_size, 0, ZL);
    clipAndDraw(screen, a, b, 0, cam);
  }
  for (int i = -10; i <= 10; ++i) {
    Point a(-ZL, 0, i * box_size), b(ZL, 0, i * box_size);
    clipAndDraw(screen, a, b, 0, cam);
  }

}
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
  Camera camera;
  Cuboid cb(0, 0, 0, 50, -50, 100);
  Cuboid cb2(-100, 0, -100, 400, -20, 250);
  while (run) {
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) run = 0;
    }
    //logic
    const Uint8 *keys = SDL_GetKeyboardState(0);
    if (keys[SDL_SCANCODE_A]) camera.phi += 0.01;
    if (keys[SDL_SCANCODE_D]) camera.phi -= 0.01;
    
    if (keys[SDL_SCANCODE_Q]) camera.theta -= 0.01;
    if (keys[SDL_SCANCODE_E]) camera.theta += 0.01;
    const int delta = 2;

    //how far is the camera's projection plane / screen from the lookAt point
    if (keys[SDL_SCANCODE_Z]) camera.ze += delta, camera.zv += delta; //zoom in
    if (keys[SDL_SCANCODE_X]) camera.ze -= delta, camera.zv -= delta; //zoom out
    //basic scene navigation
    if (keys[SDL_SCANCODE_W]) camera.moveLookAt(4); //move forward
    if (keys[SDL_SCANCODE_S]) camera.moveLookAt(-4);//move backward
    
    
    //rendering
    SDL_FillRect(screen, &screen->clip_rect, 0xFFFFFF);
    clearZBuffer();
    drawGridLines(screen, camera);
    cb.draw(screen, camera);
    cb2.draw(screen, camera);
    SDL_UpdateWindowSurface(window);
  }
  SDL_Quit();
  return 0;
}
