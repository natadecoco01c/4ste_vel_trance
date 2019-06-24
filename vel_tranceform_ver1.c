#include <math.h>
#include <stdio.h>

#define DEBUG true

double X, Y; //center of rotation
double x[4] = { 1, 1, -1, -1 }, y[4] = { -1, 1, 1, -1 }; //motor position
double rad[4];
double ang[4];
double r[4];
double R;
double ori_vel_x, ori_vel_y, ori_vel_ang;
double cmd_vel[4];
double rotate_nor;
double VEL_slope;
double VEL_size;
double turned_VEL_x, turned_VEL_y;

void calc_VEL_slope(void) {
	VEL_slope = ori_vel_y / ori_vel_x;
#if DEBUG==true
	printf("VEL_slope=%lf\n", VEL_slope);
#endif
}

void calc_VEL_size(void) {
	VEL_size = hypot(ori_vel_x, ori_vel_y);
}

void calc_R(void) {
	R = VEL_size / ori_vel_ang;
}

void calc_turned_VEL(void) {
	turned_VEL_x = -ori_vel_y;
	turned_VEL_y = ori_vel_x;
}

void calc_X_Y(void) {
	if ((ori_vel_x == 0) && (ori_vel_y == 0)) {
		X = 0;
		Y = 0;
	} else {
		double lambda;
		lambda = sqrt(
				pow(R, 2) / (pow(turned_VEL_x, 2) + pow(turned_VEL_y, 2)));
		X = lambda * turned_VEL_x;
		Y = lambda * turned_VEL_y;
	}
#if DEBUG==true
	printf("(X,Y)=(%lf,%lf)\n", X, Y);
#endif
}

void calc_r(void) {
	for (int i = 0; i < 4; i++) {
		r[i] = hypot(x[i] - X, y[i] - Y);
#if DEBUG==true
		printf("r[%d]=%lf\n", i, r[i]);
#endif
	}
}

void calc_motor_rad(void) {
	if (ori_vel_ang != 0) {
		for (int i = 0; i < 4; i++) {
			double Rad = atan(-1.0 / ((y[i] - Y) / (x[i] - X)));
			if(-6.3<=Rad&&Rad<=6.3){
			rad[i] = atan(-1.0 / ((y[i] - Y) / (x[i] - X)));
			}
#if DEBUG==true
			printf("rrad[%d]=%lf\n", i, rad[i]);
#endif
		}
	} else {
		if (ori_vel_x != 0 || ori_vel_y != 0) {
			for (int i = 0; i < 4; i++) {
				rad[i] = atan(VEL_slope);
#if DEBUG==true
				printf("rad[%d]=%lf\n", i, rad[i]);
#endif
			}
		} else {
			for (int i = 0; i < 4; i++) {
#if DEBUG==true
				printf("0rad[%d]=%lf\n", i, rad[i]);
#endif
			}
		}
	}
}
void calc_motor_vel(void) {
	if (ori_vel_ang != 0) {
		for (int i = 0; i < 4; i++) {
			cmd_vel[i] = r[i] * ori_vel_ang;
		}
	} else {
		for (int i = 0; i < 4; i++) {
			cmd_vel[i] = VEL_size;
		}
	}
}

int main(void) {
	while (1) {
		scanf("\n%lf %lf %lf", &ori_vel_x, &ori_vel_y, &ori_vel_ang);
		if ((ori_vel_ang != 0) && (ori_vel_x != 0 || ori_vel_y != 0)) {
			calc_VEL_slope();
			calc_VEL_size();
			calc_R();
			calc_turned_VEL();
			calc_X_Y();
			calc_r();
			calc_motor_rad();
			calc_motor_vel();
			printf("1\n");
		} else if (ori_vel_ang == 0) {
			calc_VEL_slope();
			calc_VEL_size();
			calc_motor_rad();
			calc_motor_vel();
			printf("2\n");
		} else {
			calc_X_Y();
			calc_r();
			calc_motor_rad();
			calc_motor_vel();
			printf("3\n");
		}
		printf("\ncmd_vel0=%lf, cmd_vel1=%lf, cmd_vel2=%lf,cmd_vel3=%lf\n",
				cmd_vel[0], cmd_vel[1], cmd_vel[2], cmd_vel[3]);
	}
	return 0;
}
