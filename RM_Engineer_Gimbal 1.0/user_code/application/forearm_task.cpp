#include "forearm_task.h"

#include "system_config.h" 

#include "minepush.h"
#include "forearm.h"
#include "auto.h"
#include "Communicate.h"

/**
  * @brief          foreaarm_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void forearm_task(void *pvParameters) 
{
    //空闲一段时间
    vTaskDelay(FOREARM_TASK_INIT_TIME);
    minepush.init();
    forearm.init();
    Auto.init();

    while(true) 
    { 
        //设置模式
        minepush.set_mode();
        forearm.set_mode();
        //反馈数据
        minepush.feedback_update();
        forearm.feedback_update();
        //设置控制量
        Auto.motor_status_measure();
        Auto.auto_control_set();
        minepush.set_control();
        forearm.set_control();
        //解算
        minepush.solve();
        forearm.solve();
        //电流输出
        minepush.output();
        forearm.output();
        photospin.output();
        //系统延时
        vTaskDelay(MINE_CONTROL_TIME_MS);
    }
}
