/**
 * @file config.h
 * @author printfKrzysztof (pritnfKrzysztof@github.null)
 * @brief
 * @version 0.1
 * @date 2023-01-03
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CONFIG_HEADER
#define CONFIG_HEADER

/*---- SOFTWARE ----*/
#define REFRESH_RATE            5                            // The framerate at which yolo analizes data (Hz)
//#define DEBUG                                              // Debug program - gives more information about matrixes

/*---- CAMERA ----*/
#define CAMERA_HORIZONTAL_ANGLE 140                          // Horizontalal degrees of lens
#define CAMERA_VERTICAL_ANGLE   70                           // Vertical degrees of lens
#define CAMERA_RESOLUTION_X     640.00                       // Camera resoltuion in x axis - pixels (horizontal)
#define CAMERA_RESOLUTION_Y     480.00                       // Camera resoltuion in y axis - pixels (vertical)

/*---- KALMAN PREDICTIONS ----*/
#define DELTA_V                 10                           // Acceptable change of speed in 1s (accel m/s^2)
#define THETA_1                 1                            // Error of measurement for ALPHA_XY (deg)
#define THETA_2                 1                            // Error of measurement for ALPHA_Z (deg)
#define THETA_3                 1                            // Error of measurement for DELTA_ALPHA (deg)
#define DEFAULT_DRONE_SIZE      0.5                          // Default size of drone in meters

//! Do not edit automatic calculated section
#define DEG_TO_RAD              3.1415/ 180 
#define MIDDLE_X                CAMERA_RESOLUTION_X / 2    
#define MIDDLE_Y                CAMERA_RESOLUTION_Y / 2    
#define X_TO_DEG                (double)(CAMERA_RESOLUTION_X / CAMERA_HORIZONTAL_ANGLE)
#define Y_TO_DEG                (double)(CAMERA_RESOLUTION_Y / CAMERA_VERTICAL_ANGLE)
#define RAD_TO_X                DEG_TO_RAD * (double)(CAMERA_HORIZONTAL_ANGLE / CAMERA_RESOLUTION_X)
#define RAD_TO_Y                DEG_TO_RAD * (double)(CAMERA_VERTICAL_ANGLE / CAMERA_RESOLUTION_Y)

#endif // !CONFIG_HEADER
