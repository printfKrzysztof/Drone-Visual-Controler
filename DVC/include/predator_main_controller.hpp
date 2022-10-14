/**
 * @file predator_main_controller.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-10-04
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef PREDATOR_MAIN_CONTROLLER_HPP
#define PREDATOR_MAIN_CONTROLLER_HPP

#define FLAG_NEW_MESSAGE 0b00000001
#define FLAG_USER_LOCKED 0b00000010
typedef enum
{

    DVC_STATE_TAKEOFF = 0,
    DVC_STATE_SEARCH,
    DVC_STATE_FOLLOW,
    DVC_STATE_LANDING,

} DVC_STATE;

#endif // PREDATOR_MAIN_CONTROLLER_HPP
