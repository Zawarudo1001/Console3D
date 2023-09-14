#include <stdio.h>
#include <cmath>
#include <conio.h>
#include <Windows.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <ctime>
#include <boost/qvm/quat.hpp>
#include <boost/qvm/vec.hpp>
#include <boost/qvm/quat_operations.hpp>
#include <boost/qvm/quat_vec_operations.hpp>
#include <iostream>

using namespace std;

#define _WIN32_WINNT 0x0500
#define fmin(a,b) (((a)<(b))?(a):(b))
#define fmax(a,b) (((a)>(b))?(a):(b))

const float pi = 3.141592741;
const int width = 120;
const int height = 30;

int rendering_mode = 2;

float x = -10;
float y = 0;
float z = 0;

float yaw = 0;
float roll = 0;
float pitch = 0;

float fov_angle = pi / 3;
float fov_dist = 5;
float fov_scale = 4;

float fmatrix[9];

float ray_vector[3];
float point[3];

char gradient[] = ".:!/r(l1Z4H9W8$@";

char screen[width * height + 1];
char zbuf[width * height + 1];

int gradientSize = int(sizeof(gradient)/sizeof(*gradient)) - 2;

float clamp(float value, float min, float max) { return fmax(fmin(value, max), min); }

struct Sphere {
	float x_center;
	float y_center;
	float z_center;
	float r;
};

struct point3 {
	float x;
	float y;
	float z;
};

struct vec3 {
	float x, y, z;
	vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {};
};

struct vec4 {
	float x, y, z, w;
	vec4(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {};
};

float length(vec3 &vect) {
	return powf((pow(vect.x, 2) + pow(vect.y, 2) + pow(vect.z, 2)), 0.5f);
}

struct vec2 {
	float x, y;
	vec2(float _x, float _y) : x(_x), y(_y) {};
};

struct Triangle {
	vec3 p1;
	vec3 p2;
	vec3 p3;
	Triangle(vec3 _p1, vec3 _p2, vec3 _p3) : p1(_p1), p2(_p2), p3(_p3){};
};

struct Object{
	Triangle* polygones;
	int polygonesCount;
};

Sphere Sphere1{ 0,5,0,1 };
Sphere Sphere2{ 5,0,0,1 };
Sphere Sphere3{ 5,5,0,1 };
Sphere Sphere4{ 0,0,5,1 };

const int SphereCount = 4;
const int ObjectsCount = 1;

Sphere SphereObjects[SphereCount] = { Sphere1 , Sphere2, Sphere4, Sphere3 };

Triangle CubePolygones[] = {
	{{ 4,-1,1 }, { 4,1,-1 }, { 4,-1,-1 }},
	{{ 4,1,-1 }, { 4,-1,1 }, { 4,1,1 }},
	{{4,-1,1}, { 6,-1,1 }, { 4,1,1 }},
	{{ 6,1,1 }, { 4,1,1 }, { 6,-1,1 }},
	{{ 4,1,-1 } ,{ 6,1,1 }, { 6,1,-1 }},
	{{ 4,1,1 }, { 6,1,1 }, { 4,1,-1 }},
	{{ 6,1,-1 }, { 6,-1,-1 }, { 4,-1,-1 }},
	{{ 4,-1,-1 }, { 4,1,-1 }, { 6,1,-1 }},
	{{ 6,-1,1 }, { 6,-1,-1 }, { 6,1,-1 }},
	{{ 6,1,-1 }, { 6,1,1 }, { 6,-1,1 }},
	{{ 4,-1,-1 }, { 6,-1,1 }, { 4,-1,1 }},
	{{ 6,-1,1 }, { 4,-1,-1 } , { 6,-1,-1 }}
};

Object Cube{ CubePolygones, sizeof(CubePolygones)/sizeof(Triangle) };

Triangle InterceptorPoly[] = {
	//планер
	{{0,0,0}, {0,-1,0}, {2,-1,0}},
	{{ 0,0,0 }, { 2,0,0 }, { 2,-1,0 }},
	{{ 7,0,0 }, { 2,0,0 }, { 2,-1,0 }},
	{{ 0,-1,0 }, { 0,-3,0 }, { 1,-3,0 }},
	{{0,-1,0}, {1,-1,0}, {1,-3,0}},
	{{1,-1,0}, {2,-1,0}, {1,-3,0}},
	{{0,0,0}, {0,1,0}, {2,1,0}},
	{{ 0,0,0 }, { 2,0,0 }, { 2,1,0 }},
	{{ 7,0,0 }, { 2,0,0 }, { 2,1,0 }},
	{{ 0,1,0 }, { 0,3,0 }, { 1,3,0 }},
	{{0,1,0}, {1,1,0}, {1,3,0}},
	{{1,1,0}, {2,1,0}, {1,3,0}},
	
	//элероны
	{{-0.5,0,0}, {0,-3,0}, {0,0,0}},
	{{-0.5,0,0}, {0,3,0}, {0,0,0}},

	//боковые элементы
	{{0.5,-1,0},{1.5,-0.5,0.8},{0.5,-0.5,0.8}},
	{{0.5,-1,0},{2,-1,0},{1.5,-0.5,0.8}},
	{{1.5,0.5,0.8},{0.5,1,0},{0.5,0.5,0.8}},
	{{2,1,0} ,{0.5,1,0} ,{1.5,0.5,0.8}},

	//кабина
	{{0.5,0.5,0.8},{1.5,0.5,0.8},{1.5,-0.5,0.8}},
	{{0.5,-0.5,0.8},{1.5,-0.5,0.8},{0.5,0.5,0.8}},
	{{0.5,-1,0} ,{-0.5,0,0}, {0.5,-0.5,0.8}},
	{{0.5,0.5,0.8}, {0.5,1,0}, {-0.5,0,0}},
	{{0.5,-0.5,0.8}, {0.5,0.5,0.8}, {-0.5,0,0}},

	//носовые элементы
	{{1.75,0.75,0.4},{7,0,0},{2,1,0}},
	{{1.75,-0.75,0.4},{2,-1,0},{7,0,0}},
	{{1.75,0.75,0.4},{7,0,0},{1.75,-0.75,0.4}},

	//стекло кабины
	{{1.5,0.5,0.8},{1.5,-0.5,0.8},{1.75,0.75,0.4}},
	{{1.75,0.75,0.4},{1.5,-0.5,0.8},{1.75,-0.75,0.4}}

};

Object Interceptor{ InterceptorPoly, sizeof(InterceptorPoly) / sizeof(Triangle) };

Object objects[ObjectsCount] = { Interceptor };


void SetWindow(int Width, int Height)
{
	_COORD coord;
	coord.X = Width;
	coord.Y = Height;
	_SMALL_RECT Rect;
	Rect.Top = 0;
	Rect.Left = 0;
	Rect.Bottom = Height - 1;
	Rect.Right = Width - 1;
	HANDLE Handle = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleScreenBufferSize(Handle, coord);
	SetConsoleWindowInfo(Handle, TRUE, &Rect);
}

void hidecursor()
{
	HANDLE consoleHandle = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_CURSOR_INFO info;
	info.dwSize = 100;
	info.bVisible = FALSE;
	SetConsoleCursorInfo(consoleHandle, &info);
}

void MulMatrix(short M, short N, short K, float* A, float* B, float* C)
{
	for (int i = 0; i < M; ++i)
	{
		float * c = C + i * N;
		for (int j = 0; j < N; ++j)
			c[j] = 0;
		for (int k = 0; k < K; ++k)
		{
			const float *b = B + k * N;
			float a = A[i*K + k];
			for (int j = 0; j < N; ++j)
				c[j] += a * b[j];
		}
	}
}

vec3 MatrixV3(vec3 &n, float* S) {
	vec3 D{ 0,0,0 };
	D.x = S[0] * n.x + S[1] * n.y + S[2] * n.z;
	D.y = S[3] * n.x + S[4] * n.y + S[5] * n.z;
	D.z = S[6] * n.x + S[7] * n.y + S[8] * n.z;
	return D;
}

float dot(vec3 &a, vec3 &b) {
	return (a.x * b.x + a.y * b.y + a.z * b.z);
}

void norm(vec3 &n) {
	float veclen = powf(pow(n.x, 2) + pow(n.y, 2) + pow(n.z, 2), 0.5);
	n.x /= veclen;
	n.y /= veclen;
	n.z /= veclen;
}


vec2 sphIntersect(vec3 ro, vec3 rd, vec3 ce, float ra)
{
	vec3 m{ ro.x - ce.x, ro.y - ce.y, ro.z - ce.z };

	float b = dot(m, rd);
	float c = dot(m, m) - ra * ra;

	float d = b * b - c;

	if (d >= 0) {
		float sqrtfd = sqrtf(d);
		float t1 = -b - sqrtfd;
		float t2 = -b + sqrtfd;
		if (t1 < 0.f and t2 < 0.f) return vec2{ -1.f, 0 };
		return vec2{ t1, t2 };
	}
	return vec2{ -1.f, 0 };
}


vec3 cross(vec3 &v_A, vec3 &v_B) {
	vec3 c_P{ 0,0,0 };
	c_P.x = v_A.y * v_B.z - v_A.z * v_B.y;
	c_P.y = -(v_A.x * v_B.z - v_A.z * v_B.x);
	c_P.z = v_A.x * v_B.y - v_A.y * v_B.x;
	return c_P;
}

vec4 triIntersect(vec3 ro, vec3 rd, Triangle triangle)
{
	vec3 v0 = triangle.p1;
	vec3 v1 = triangle.p2;
	vec3 v2 = triangle.p3;

	vec3 v1v0 = vec3{ v1.x - v0.x, v1.y - v0.y, v1.z - v0.z };
	vec3 v2v0 = vec3{ v2.x - v0.x, v2.y - v0.y, v2.z - v0.z };

	vec3 rov0 = vec3{ ro.x - v0.x, ro.y - v0.y, ro.z - v0.z };
	vec3  n = cross(v1v0, v2v0);
	vec3  q = cross(rov0, rd);
	float d = 1.0 / dot(rd, n);

	float v = d * dot(q, v1v0);
	q = { -q.x, -q.y, -q.z };
	float u = d * dot(q, v2v0);
	n = { -n.x, -n.y, -n.z };
	float t = d * dot(n, rov0);
	if (u < 0.0 || v<0.0 || (u + v)>1.0) t = -1.0;
	return vec4{-n.x, -n.y, -n.z, t};
}




void RayCast(vec3 ray_vector, vec3 ro, short x_step, short y_step, char *screen) {

	float nearestIntersect = 9999.f;
	int nearestObject = -1;
	vec4 Intersect{ 0,0,0,0 };
	vec3 n{ 0,0,0 };

	for (int i = 0; i < ObjectsCount; i++) {
		//теперь можно работать с объектами, а не с набором полигонов

		for (int k = 0; k < objects[i].polygonesCount; k++) {

			Intersect = triIntersect(ro, ray_vector, objects[i].polygones[k]);

			if (Intersect.w > 0 and Intersect.w < nearestIntersect) {
				nearestIntersect = Intersect.w;
				n = vec3{ Intersect.x, Intersect.y, Intersect.z };
				nearestObject = i;
			}

		}
	}
	if (nearestObject == -1) {
		screen[y_step * 120 + x_step] = ' ';
	}
	else {
		norm(n);
		vec3 light = { -1, 1, 0};
		norm(light);
		float albedo = 1.0f;

		float diff = (dot(n, light) * 0.5 + 0.5) * albedo;
		int color = (int)(diff * 20);
		color = clamp(color, 0, gradientSize);
		char pixel = gradient[color];

		screen[y_step * 120 + x_step] = pixel;
	}
}

void R(float a, float b, float g) {
	float rot_matrix[9]{

		cos(a)*cos(b),		-sin(a)*cos(g) - cos(a)*sin(b)*sin(g),		sin(a)*sin(g) - cos(a)*sin(b)*cos(g),

		sin(a)*cos(b),		cos(a)*cos(g) - sin(a)*sin(b)*sin(g),	 -cos(a)*sin(g) - sin(a)*sin(b)*cos(g),

		sin(b),		cos(b)*sin(g),		cos(b)*cos(g)

	};
	memcpy(fmatrix, rot_matrix, 36);
}

void coordTransform(Triangle &polygon) {

	//поворот относительно центра треугольного полигона
	
	vec3 center{ (polygon.p1.x + polygon.p2.x + polygon.p3.x)/3, (polygon.p1.y + polygon.p2.y + polygon.p3.y) / 3, (polygon.p1.z + polygon.p2.z + polygon.p3.z) / 3 };

	vec3 p1_r = { polygon.p1.x - center.x, polygon.p1.y - center.y , polygon.p1.z - center.z };
	vec3 p2_r = { polygon.p2.x - center.x, polygon.p2.y - center.y , polygon.p2.z - center.z };
	vec3 p3_r = { polygon.p3.x - center.x, polygon.p3.y - center.y , polygon.p3.z - center.z };

	polygon.p1 = MatrixV3(p1_r, fmatrix);
	polygon.p1 = { polygon.p1.x + center.x, polygon.p1.y + center.y, polygon.p1.z + center.z };

	polygon.p2 = MatrixV3(p2_r, fmatrix);
	polygon.p2 = { polygon.p2.x + center.x, polygon.p2.y + center.y, polygon.p2.z + center.z };

	polygon.p3 = MatrixV3(p3_r, fmatrix);
	polygon.p3 = { polygon.p3.x + center.x, polygon.p3.y + center.y, polygon.p3.z + center.z };
};


void coordTransform(Triangle &polygon, vec3 point) {
	//поворот относительно заданной точки

	vec3 p1_r = { polygon.p1.x - point.x, polygon.p1.y - point.y, polygon.p1.z - point.z };
	vec3 p2_r = { polygon.p2.x - point.x, polygon.p2.y - point.y, polygon.p2.z - point.z };
	vec3 p3_r = { polygon.p3.x - point.x, polygon.p3.y - point.y, polygon.p3.z - point.z };
	
	polygon.p1 = MatrixV3(p1_r, fmatrix);
	polygon.p1 = { polygon.p1.x + point.x, polygon.p1.y + point.y, polygon.p1.z + point.z };

	polygon.p2 = MatrixV3(p2_r, fmatrix);
	polygon.p2 = { polygon.p2.x + point.x, polygon.p2.y + point.y, polygon.p2.z + point.z };

	polygon.p3 = MatrixV3(p3_r, fmatrix);
	polygon.p3 = { polygon.p3.x + point.x, polygon.p3.y + point.y, polygon.p3.z + point.z };

};

//нужно добавить вычисление центральной точки у объекта

void rotateObject(Object &obj, float a, float b, float c) {
	R(a, b, c);
	for (int i = 0; i < obj.polygonesCount; i++) {
		coordTransform(obj.polygones[i]);
	}
}

void rotateObject(Object &obj, float a, float b, float c, vec3 point) {
	R(a, b, c);
	for (int i = 0; i < obj.polygonesCount; i++) {
		coordTransform(obj.polygones[i], { point.x, point.y, point.z });
	}
}


void render(char * screen, int mode) {
	memcpy(zbuf, screen, sizeof screen);
	short counter_x = 0;
	short counter_y = 0;

	R(yaw, pitch, roll);

	switch (mode) {

	//для широкоугольных камер
	case 1: {
		float ground_angle, vert_angle;
		vec3 ro{ x,y,z };
		ground_angle = fov_angle / 2;
		for (short i = 0; i < 120; i++) {
			vert_angle = fov_angle / 4;
			for (short j = 0; j < 30; j++) {
				vec3 default_vector{ cos(ground_angle)*cos(vert_angle), sin(ground_angle) * cos(vert_angle), sin(vert_angle) };
				vec3 ray_vector = MatrixV3(default_vector, fmatrix);
				RayCast(ray_vector, ro, i, j, screen);
				vert_angle -= fov_angle / 60.f;
			}
			ground_angle -= fov_angle / 120.f;
		}
		break;
	}

	//перспективная проекция 
	case 2: {
			vec3 ro{ x,y,z };
			float xpos, ypos;
			xpos = 1.;
			for (short i = 0; i < 120; i++) {
				counter_y = 0;
				ypos = 1.;
				for (float j = 0; j < 30; j++) {
					vec3 default_vector{ fov_dist, xpos, ypos * 0.5f };

					norm(default_vector);

					vec3 ray_vector = MatrixV3(default_vector, fmatrix);

					RayCast(ray_vector, ro, i, j, screen);

					ypos -= 1.f / 15;
				}
				xpos -= 1.f / 60;
			}
			break;
		}

	//ортографическая проекция
	case 3: {
			float xpos, ypos;
			xpos = fov_scale;
			for (float i = 0; i < 120; i++) {
				counter_y = 0;
				ypos = fov_scale;
				for (float j = 0; j < 30; j++) {
					vec3 ro{ 0, xpos, ypos * 0.5f };
					vec3 default_vector{ 1, 0, 0 };

					vec3 point = MatrixV3(ro, fmatrix);
					vec3 ray_vector = MatrixV3(default_vector, fmatrix);

					point.x += x;
					point.y += y;
					point.z += z;
					RayCast(ray_vector, point, i, j, screen);
					ypos -= fov_scale / 15.f;
				}
				xpos -= fov_scale / 60.f;
			}
			break;
		}
	}//end switch
}

int main()
{
	/*
	const auto axis = boost::qvm::vec<float, 3>{ { 0.0, 1.0, 0.0} };
	float rotationAngle = (3.14159 / 2.0);

	boost::qvm::quat<float> q = boost::qvm::rot_quat(axis, rotationAngle);

	const boost::qvm::vec<double, 3> rotateVec = q * boost::qvm::vec<double, 3>{ {1.0, 0.0, 0.0}};
	
	cout << rotateVec.a[0] << rotateVec.a[1] << rotateVec.a[2] << endl;
	return 0;
	*/


	SetWindow(width, height);
	hidecursor();
	HWND consoleWindow = GetConsoleWindow();
	SetWindowLong(consoleWindow, GWL_STYLE, GetWindowLong(consoleWindow, GWL_STYLE) & ~WS_MAXIMIZEBOX & ~WS_SIZEBOX);

	screen[width * height] = '\0';

	render(screen, rendering_mode);

	while (true)
	{
		rotateObject(objects[0], 0, 0.05, 0.05, { 5,0,0 });

		if (GetAsyncKeyState(0x44)){ yaw -= 0.0125 * cos(roll); pitch -= 0.0125 * sin(roll);}
		
		if (GetAsyncKeyState(0x41)){ yaw += 0.0125 * cos(roll); pitch += 0.0125 * sin(roll);}

		if (GetAsyncKeyState(0x53)){ yaw += 0.0125 * sin(roll); pitch -= 0.0125 * cos(roll);}

		if (GetAsyncKeyState(0x57)){ yaw -= 0.0125 * sin(roll); pitch += 0.0125 * cos(roll);}

		if (GetAsyncKeyState(0x5a)){ x -= 0.25f; }
		if (GetAsyncKeyState(0x51)){ x += 0.25f; }

		if (GetAsyncKeyState(0x56)) { y -= 0.25f; }
		if (GetAsyncKeyState(0x43)) { y += 0.25f; }

		if (GetAsyncKeyState(0x45)) { z += 0.25f; }
		if (GetAsyncKeyState(0x58)) { z -= 0.25f; }

		if (GetAsyncKeyState(0x4c)) { roll += 0.0125; }
		if (GetAsyncKeyState(0x4b)) { roll -= 0.0125; }

		if (GetAsyncKeyState(0x52)) {
			R(yaw, pitch, roll);
			vec3 default_vector { 1, 0, 0 };
			vec3 ray_vector = MatrixV3(default_vector, fmatrix);
			x += ray_vector.x * 0.25f;
			y += ray_vector.y * 0.25f;
			z += ray_vector.z * 0.25f;
		}

		if (GetAsyncKeyState(0x46)) {
			R(yaw, pitch, roll);
			vec3 default_vector{ 1, 0, 0 };
			vec3 ray_vector = MatrixV3(default_vector, fmatrix);
			x -= ray_vector.x * 0.25f;
			y -= ray_vector.y * 0.25f;
			z -= ray_vector.z * 0.25f;
		}

		if (GetAsyncKeyState(0x31)) { rendering_mode = 1; }
		if (GetAsyncKeyState(0x32)) { rendering_mode = 2; }
		if (GetAsyncKeyState(0x33)) { rendering_mode = 3; }
		
		auto start = std::chrono::high_resolution_clock::now();
		render(screen, rendering_mode);
		// wait for end of frame
		auto elapsed = std::chrono::high_resolution_clock::now() - start;

		unsigned char frametime = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

		//printf("\nExecution time: %dms\n", frametime);

		if (frametime < 17) std::this_thread::sleep_for(std::chrono::milliseconds(17 - frametime));			//set frame limit 60 fps

		bool frame = memcmp(zbuf, screen, sizeof screen);
		if (frame != 0) printf(screen);
	}
}
