#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include "daemon.h"


/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();
const App_Fault_s *RobotCMDGetFault(void);

#endif // !ROBOT_CMD_H