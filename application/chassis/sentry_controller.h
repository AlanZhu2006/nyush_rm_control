/**
 * @file sentry_controller.h
 * @brief 哨兵 Swerve 底盘入口，供 chassis.c 在 ROBOT_TYPE_sentry 时调用
 */
#ifndef SENTRY_CONTROLLER_H
#define SENTRY_CONTROLLER_H

void SentryChassisInit(void);
void SentryChassisTask(void);

#endif // SENTRY_CONTROLLER_H
