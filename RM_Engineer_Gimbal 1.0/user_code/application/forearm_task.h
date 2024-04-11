#ifndef FOREARM_TASK_H
#define FOREARM_TASK_H

#include "cmsis_os.h"
#include "main.h"


//任务开始空闲一段时间
#define FOREARM_TASK_INIT_TIME 30



/**
  * @brief           forearm_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void forearm_task(void *pvParameters);



#endif