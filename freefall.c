#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#define G_CONST 6.67430e-11
#define TERMINAL_HEIGHT 20

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void clear_screen() {
    printf("\033[H\033[J");
}

void rk4_step(double *y, double *v, double dt, double g, double drag_factor, double mass) {
    double k1_y = -(*v);
    double k1_v = g - (drag_factor / mass) * (*v) * (*v);
    double y_temp = *y + 0.5 * dt * k1_y;
    double v_temp = *v + 0.5 * dt * k1_v;
    double k2_y = -v_temp;
    double k2_v = g - (drag_factor / mass) * (v_temp * v_temp);
    y_temp = *y + 0.5 * dt * k2_y;
    v_temp = *v + 0.5 * dt * k2_v;
    double k3_y = -v_temp;
    double k3_v = g - (drag_factor / mass) * (v_temp * v_temp);
    y_temp = *y + dt * k3_y;
    v_temp = *v + dt * k3_v;
    double k4_y = -v_temp;
    double k4_v = g - (drag_factor / mass) * (v_temp * v_temp);
    *y += (dt / 6.0) * (k1_y + 2*k2_y + 2*k3_y + k4_y);
    *v += (dt / 6.0) * (k1_v + 2*k2_v + 2*k3_v + k4_v);
}

int main(void) {
    int mode;
    printf("Select simulation mode:\n");
    printf("1. Basic free-fall (constant g, no drag)\n");
    printf("2. Advanced free-fall (realistic: g from celestial body, air drag with RK4 integration)\n");
    printf("Enter 1 or 2: ");
    scanf("%d", &mode);
    
    if (mode == 1) {
        double height, gravity;
        printf("\n--- Basic Free-Fall Calculation ---\n");
        printf("Enter height (in meters): ");
        scanf("%lf", &height);
        printf("Enter gravitational acceleration (in m/s^2): ");
        scanf("%lf", &gravity);
        double free_fall_time = sqrt(2.0 * height / gravity);
        printf("\nCalculated free-fall time (ignoring air resistance): %lf seconds\n", free_fall_time);
        double dt = free_fall_time / (TERMINAL_HEIGHT * 50);
        double t = 0.0;
        while (t <= free_fall_time) {
            double y = height - 0.5 * gravity * t * t;
            if (y < 0) {
                y = 0;
            }
            int row = (int)((y / height) * (TERMINAL_HEIGHT - 1));
            clear_screen();
            for (int i = 0; i < row; i++) {
                printf("\n");
            }
            printf("   O   \n");
            for (int i = row; i < TERMINAL_HEIGHT - 1; i++) {
                printf("\n");
            }
            usleep(50000);
            t += dt;
        }
        printf("Object has hit the ground!\n");
    } else if (mode == 2) {
        double height, obj_mass, obj_diameter;
        double body_mass, body_radius;
        double drag_coefficient, air_density;
        printf("\n--- Advanced Free-Fall Simulation ---\n");
        printf("Enter height (in meters): ");
        scanf("%lf", &height);
        printf("Enter object's mass (in kg): ");
        scanf("%lf", &obj_mass);
        printf("Enter object's diameter (in meters): ");
        scanf("%lf", &obj_diameter);
        printf("Enter celestial body's mass (in kg): ");
        scanf("%lf", &body_mass);
        printf("Enter celestial body's radius (in meters): ");
        scanf("%lf", &body_radius);
        printf("Enter drag coefficient (e.g., 0.47 for a sphere): ");
        scanf("%lf", &drag_coefficient);
        printf("Enter air density (in kg/m^3, e.g., 1.225): ");
        scanf("%lf", &air_density);
        double g = (G_CONST * body_mass) / (body_radius * body_radius);
        printf("\nCalculated gravitational acceleration: %lf m/s^2\n", g);
        double area = M_PI * pow(obj_diameter / 2.0, 2.0);
        double drag_factor = 0.5 * air_density * drag_coefficient * area;
        double t = 0.0;
        double dt = 0.001;
        double y = height;
        double v = 0.0;
        double frame_interval = 0.02;
        double next_frame_time = 0.0;
        while (y > 0) {
            rk4_step(&y, &v, dt, g, drag_factor, obj_mass);
            if (y < 0)
                y = 0;
            t += dt;
            if (t >= next_frame_time) {
                next_frame_time += frame_interval;
                clear_screen();
                int row = (int)((y / height) * (TERMINAL_HEIGHT - 1));
                for (int i = 0; i < row; i++) {
                    printf("\n");
                }
                printf("   O   \n");
                for (int i = row; i < TERMINAL_HEIGHT - 1; i++) {
                    printf("\n");
                }
                usleep(20000);
            }
        }
        double v_terminal = sqrt((g * obj_mass) / drag_factor);
        printf("Object has hit the ground after %lf seconds.\n", t);
        printf("Final velocity: %lf m/s (estimated terminal velocity: %lf m/s)\n", v, v_terminal);
    } else {
        printf("Invalid mode selected.\n");
    }
    
    return 0;
}
