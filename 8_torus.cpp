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
	Point translate(const Point & b) const {
		return Point(x + b.x, y + b.y, z + b.z);
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
//a, b => world space;
void clipAndDraw(SDL_Surface * screen, Point a, Point b, Uint32 color, const Camera & cam) {
	a = a.toVC(cam);
	b = b.toVC(cam);
	if (a.z > b.z) swap(a, b);
	double znear = cam.ze + 10;
	if (b.z < znear) return ;
	if (a.z < znear) {
		double u = (znear - a.z) / (b.z - a.z); //b.z != a.z, otherwise both would be inside or outside, i.e. line doesnot intersect near plane
		a.x += u * (b.x - a.x);
		a.y += u * (b.y - a.y);
		a.z = znear;
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
void triangleFill(SDL_Surface * screen, const Point & a, const Point & b, const Point & c, Uint32 color, const Camera & cam) {
	Point vca = a.toVC(cam), vcb = b.toVC(cam), vcc = c.toVC(cam);
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
struct Triangle {
	Point a, b, c;
	Triangle(const Point & pa, const Point & pb, const Point & pc) : a(pa), b(pb), c(pc) { }
	void draw(SDL_Surface * screen, Uint32 color, const Camera & cam) const {
		triangleFill(screen, a, b, c, color, cam);
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
//helix is basically giving increasing height to each segment of a torus
//, so the implementations of torus and helix are nearly the same
void drawTorus(SDL_Surface * screen, const Camera & cam, double height, const Point & p, Uint32 cola, Uint32 colb) {
	int steps = 20;
	int rad_steps = 20;
	double step_angle = 2 * M_PI / steps;
	double rad_ang = 2 * M_PI / rad_steps;
	double R = 60, r = 20;
	for (int i = 1; i <= steps; ++i) {
		double theta = i * step_angle;
		double ox1 = p.x + R * sin(theta);
		double oy1 = 0;
		double oz1 = p.z + R * cos(theta);
		double ox2 = p.x + R * sin(theta - step_angle);
		double oy2 = 0;
		double oz2 = p.z + R * cos(theta - step_angle);
		Point preva, prevb;
		for (int j = 0; j <= rad_steps; ++j) { //= for completing a cycle
			double ang = j * rad_ang;
			double hy1 = oy1 + r * sin(ang);
			double hz1 = oz1 + r * cos(ang);
			double hy2 = oy2 + r * sin(ang);
			double hz2 = oz2 + r * cos(ang);
			Point pa(ox1, hy1, hz1);
			//orient the circle 
			pa = pa.translate(Point(-ox1, 0, -oz1)).rotateY(theta).translate(Point(ox1, 0, oz1));
			Point pb(ox2, hy2, hz2);
			pb = pb.translate(Point(-ox2, 0, -oz2)).rotateY(theta - step_angle).translate(Point(ox2, 0, oz2));
			if (j > 0) {
				triangleFill(screen, pa, pb, preva, cola, cam);
				triangleFill(screen, prevb, pb, preva, colb, cam);
			}
			preva = pa;
			prevb = pb;
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
 	SDL_Window* window = SDL_CreateWindow("Graphics 3d", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
	SDL_Surface *screen = SDL_GetWindowSurface(window);
	SDL_Event event;
	bool run = 1;
	Camera camera;
	double phi = 0;
	while (run) {
		while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE))
                run = 0;
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
		if (keys[SDLK_SPACE]) phi += 0.1;

		//rendering
		SDL_FillRect(screen, &screen->clip_rect, 0xFFFFFF);
		clearZBuffer();
		drawGridLines(screen, camera);
		drawTorus(screen, camera, 300, Point(0, 0, 0), 0xFF0000, 0x00FF00);
		drawTorus(screen, camera, 300, Point(0, 0, 0), 0x0000FF, 0);
		SDL_UpdateWindowSurface(window);
		SDL_Delay(10);
	}
	SDL_Quit();
	return 0;
}
