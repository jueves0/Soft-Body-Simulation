#include <stdio.h>
#include "raylib.h"
#include <math.h>


float g = 0.001;
float friction = 1;
float spring_constant = 0.3;


float Xa[10000];
float Ya[10000];

float Xb[10000];
float Yb[10000];

int count = 0;


int main() {
	int Xdim = 30;
	int Ydim = 20;
	int N = Xdim*Ydim;
	int N2 = (Xdim - 1)*(Ydim) + (Ydim - 1)*(Xdim) + (Xdim)*(Ydim) + (Xdim - 1)*(Ydim - 1);
	//coordinates of points
	float X[N];
	float Y[N];
	//momentum of points
	float momentX[N];
	float momentY[N];
	//bonds between points
	int BND1[N2];
	int BND2[N2];
	int C = 0;
	float displace[N2];
	int i;
	int j;
	float scale = 10;
	for (i = 0; i < N; ++i) {
		if (i%Xdim < Xdim - 1) {
			BND1[C] = i;
			BND2[C] = i + 1;
			displace[C] = scale;
			++C;
		}
		if (i < N - Xdim) {
			BND1[C] = i;
			BND2[C] = i + Xdim;
			displace[C] = scale;
			++C;
		}
		if ((i < N - Xdim) * (i%Xdim < Xdim - 1)) {
			BND1[C] = i;
			BND2[C] = i + 1 + Xdim;
			displace[C] = sqrt(2*pow(scale,2));
			++C;
		}
		if ((i < N - Xdim) * (i%Xdim > 0)) {
			BND1[C] = i;
			BND2[C] = i - 1 + Xdim;
			displace[C] = sqrt(2*pow(scale,2));
			++C;
		}
	}
	//initialize points to a grid
	SetTargetFPS(60);
	for (i = 0; i < N; ++i) {
		momentX[i] = 0;
		momentY[i] = 0;
		X[i] = 50 + scale*(float)(i%Xdim);
		Y[i] = 50 + scale*(float)((i)/Xdim);
	}
	int width = 800;
	int height = 800;
	InitWindow(width, height, "Soft body simulation");
	int adding = 0;
	int xf = 0;
	int yf = 0;
	while (!WindowShouldClose()) {
		if (IsKeyPressed(88)) {
			//remove one of the lines/obstacles
			if (count > 0) {
				count -= 1;
			}
		}
		BeginDrawing();
		ClearBackground(WHITE);
		if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
			adding = 1;
			xf = GetMousePosition().x;
			yf = GetMousePosition().y;
		}
		if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)*adding) {
			adding = 0;
			if (GetMousePosition().x != xf) {
				Xa[count] = (float)xf;
				Ya[count] = (float)yf;
				Xb[count] = (float)GetMousePosition().x;
				Yb[count] = (float)GetMousePosition().y;
				++count;
			}
		}
		//update momentum according to rules
		for (i = 0; i < C; ++i) {
			//printf("\nBOND #1: %i\n", BND1[i]);
			//printf("BOND #2: %i\n", BND2[i]);
			//printf("displacement: %f\n", displace[i]);
			int i1 = BND1[i];
			int i2 = BND2[i];
			float x1 = X[i1];
			float y1 = Y[i1];
			float x2 = X[i2];
			float y2 = Y[i2];
			float dista = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
			float force = spring_constant*(dista - displace[i]);
			momentX[i1] += ((x2 - x1)/dista)*force;
			momentY[i1] += ((y2 - y1)/dista)*force;
			momentX[i2] += ((x1 - x2)/dista)*force;
			momentY[i2] += ((y1 - y2)/dista)*force;
		}
		if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
			//then set it to move towards mouse
			float posX = (float)GetMousePosition().x;
			float posY = (float)GetMousePosition().y;
			for (i = 0; i < N; ++i) {
				momentX[i] += (posX - X[i])/10000;
				momentY[i] += (posY - Y[i])/10000;
			}
		}
		for (i = 0; i < N; ++i) {
			//first gravity
			momentY[i] += g;
			if (Y[i] > height) {
				//then it is hitting the floor
				momentY[i] = (-fabs(momentY[i]) - g);
			}
			//check for collisions with any of the lines
			for (j = 0; j < count; ++ j) {
				//1; it has to be in bounds of the line
				//2; it has to switch which side it is one
				float slope = (Ya[j] - Yb[j])/(Xa[j] - Xb[j]);
				//y = mx + b
				//b = y - mx
				float intercept = Ya[j] - (slope*Xa[j]);
				float solCurr = slope*(X[i]) - Y[i] + intercept;
				float solWillbe = slope*(X[i] + momentX[i]) - Y[i] - momentY[i] + intercept;
				float left = fmin(Xa[j], Xb[j]);
				float right = fmax(Xa[j], Xb[j]);
				if (((solCurr > 0) != (solWillbe > 0)) * ((X[i] > left) * (X[i] < right))) {
					//then based on current trajectory, it should pass through
					//so... just remove momentum
					momentX[i] = 0;
					momentY[i] = 0;
					j = count; /*break secondary loop*/
				}
			}
			//now forces from connected points
			momentX[i] = friction*momentX[i];
			momentY[i] = friction*momentY[i];
		}
		//then now update position according to momentum
		if (IsKeyPressed(82)) {
			for (i = 0; i < N; ++i) {
				momentX[i] = -momentX[i];
				momentY[i] = -momentY[i];
			}
		}
		for (i = 0; i < N; ++i) {
			X[i] += momentX[i];
			Y[i] += momentY[i];
		}
		for (i = 0; i < count; ++i) {
			DrawLine(Xa[i], Ya[i], Xb[i], Yb[i], BLACK);
		}
		if (adding) {
			DrawLine(xf, yf, GetMousePosition().x, GetMousePosition().y, BLUE);
		}
		for (i = 0; i < N; ++i) {
			DrawCircle(X[i], Y[i], 1, BLUE);
		}
		for (i = 0; i < C; ++i) {
			int ind1 = BND1[i];
			int ind2 = BND2[i];
			DrawLine(X[ind1], Y[ind1], X[ind2], Y[ind2], BLACK);
		}
		DrawText("Press R to reverse direction", width/1.7, 0.1*height, 20, BLUE);
		DrawText("Press X to remove line", width/1.7, 0.2*height, 20, BLUE);
		EndDrawing();
	}
}
