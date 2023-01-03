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
#define REFRESH_RATE            60                           //The framerate at which yolo analizes data

/*---- CAMERA ----*/
#define CAMERA_HORIZONTAL_ANGLE 140                          //Horizontalal degrees of lens
#define CAMERA_VERTICAL_ANGLE   70                           //Vertical degrees of lens
#define CAMERA_RESOLUTION_X     640.00                       //Camera resoltuion in x axis - pixels (horizontal)
#define CAMERA_RESOLUTION_Y     480.00                       //Camera resoltuion in y axis - pixels (vertical)

/*---- KALMAN PREDICTIONS ----*/
#define DELTA_V                 10                           //Acceptable change of speed in 1 sec
#define THETA_1                 1                            //Error of measurement for ALPHA_XY
#define THETA_2                 1                            //Error of measurement for ALPHA_Z
#define THETA_3                 1                            //Error of measurement for DELTA_ALPHA
#define DEFAULT_DRONE_SIZE      0.5                          //Default size of drone in meters

//! Do not edit automatic calculated section
#define MIDDLE_X                CAMERA_RESOLUTION_X          /2
#define MIDDLE_Y                CAMERA_RESOLUTION_Y          /2
#define X_TO_DEG                (double)(CAMERA_RESOLUTION_X /CAMERA_HORIZONTAL_ANGLE)    //Always cast on double in C/C++
#define Y_TO_DEG                (double)(CAMERA_RESOLUTION_Y /CAMERA_VERTICAL_ANGLE)    //Always cast on double in C/C++

#endif // !CONFIG_HEADER
