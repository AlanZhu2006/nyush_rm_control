#pragma once
#ifndef ROBOT_CONFIG_SELECT_H
#define ROBOT_CONFIG_SELECT_H

#if defined(ROBOT_TYPE_infantry)
#include "robot_configs/robot_infantry.h"
#elif defined(ROBOT_TYPE_hero)
#include "robot_configs/robot_hero.h"
#elif defined(ROBOT_TYPE_sentry)
#include "robot_configs/robot_sentry.h"
#else
#error "Unknown or missing ROBOT_TYPE_*. Use ROBOT_TYPE=infantry|hero|sentry"
#endif

#endif // ROBOT_CONFIG_SELECT_H
