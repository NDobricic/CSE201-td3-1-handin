#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {
    double *new_array = new double[new_size]();
    memcpy(new_array, array, length * sizeof(double));
    delete[] array;

    return new_array;
}

double* shrink_array(double* array, int length, int new_size) {
    double *new_array = new double[new_size]();
    memcpy(new_array, array, new_size * sizeof(double));
    delete[] array;

    return new_array;
}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {
    if (current_size == max_size) {
        max_size += 5;
        array = extend_array(array, current_size, max_size);
    }

    array[current_size] = element;
    current_size++;

    return array;
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {
    current_size = std::max(current_size - 1, 0);

    if (current_size <= max_size - 5) {
        max_size -= 5;
        array = shrink_array(array, current_size, max_size);
    }

    return array;
}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {
    // YOU CAN MODIFY THIS FUNCTION TO RECORD THE TELEMETRY

    bool hit_target, hit_obstacle;
    double v0_x, v0_y, x, y, t;
    double PI = 3.14159265;
    double g = 9.8;

    v0_x = magnitude * cos(angle * PI / 180);
    v0_y = magnitude * sin(angle * PI / 180);

    t = 0;
    x = 0;
    y = 0;

    hit_target = false;
    hit_obstacle = false;
    while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
        telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
        telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
        telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);

        double * target_coordinates = find_collision(x, y, targets, tot_targets);
        if (target_coordinates != NULL) {
            remove_target(targets, tot_targets, target_coordinates);
            hit_target = true;
        } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
            hit_obstacle = true;
        } else {
            t = t + simulation_interval;
            y = v0_y * t  - 0.5 * g * t * t;
            x = v0_x * t;
        }
    }

    return hit_target;
}

void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {

    int new_size = 0;
    for (int i = 0; i < tot_telemetries; i++) {
        new_size += telemetries_sizes[i];
    }

//    std::cout << "new_size: " << new_size << std::endl;

    if (new_size > global_telemetry_max_size) {
        global_telemetry_max_size = new_size;
        global_telemetry = extend_array(global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
    } else if (new_size <= global_telemetry_max_size - 5) {
        global_telemetry_max_size = new_size;
        global_telemetry = shrink_array(global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
    }

    global_telemetry_current_size = new_size;

//    std::cout << "curr and max size: " << global_telemetry_current_size << " " << global_telemetry_max_size << std::endl;

    double *current_element[tot_telemetries];
    for (int i = 0; i < tot_telemetries; i++) {
        current_element[i] = telemetries[i];
    }

    int current_size = 0;
    while (current_size < global_telemetry_current_size) {
        int min_time = 9999999;
        int min_time_index = 0;
        for (int i = 0; i < tot_telemetries; i++) {
            if (current_element[i] < telemetries[i] + telemetries_sizes[i] - 1) {
                if (*current_element[i] < min_time) {
                    min_time = *current_element[i];
                    min_time_index = i;
                }
            }
        }

//        std::cout << min_time_index << " " << current_size << " " << *current_element[min_time_index] << std::endl;
        memcpy(global_telemetry + current_size, current_element[min_time_index], 3 * sizeof(double));
        current_element[min_time_index] += 3;
        current_size += 3;

//        std::cout << "global telemetry: ";
//        for (int i = 0; i < current_size; i++) {
//            std::cout << global_telemetry[i] << " ";
//        }
//        std::cout << std::endl;
    }
}
