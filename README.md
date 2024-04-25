# 20240404

宁工NewLegends战队工程2324赛季电控_youg

此代码适用于宁波工程学院222324赛季工程机器人底盘mcu

Developing，绝大部分临时移植的代码冗余还未完善

# **24赛季**

## 版本更新


1. 架构修改调试完成（还没有）

2. 自动取矿加入（还没有）

3. 整体电机初步调参（还没有）

4. 加入自定义控制器（还没有）

5. 加入工程接线图（还没有）

6. UI的调整（还没有）

7. 裁判系统更新（还没有）

## 版本问题 （后面[]为对应修改版本）

1. 【可能的】视觉兑矿（不确定）

# 操作方式(新)

## 遥控器控制 左摇杆上下控制臂的运动
*左拨杆向上，自右向左的终端姿态变化*
-----------------------------------------------------------------------------
左拨杆 | 右拨杆 | 摇杆         | 功能            | En           | CAN_ID
----  | ------ | ----------- | --------------- | ------------ | -------------
上     | 上    |   左上下     |yaw轴            | yaw          | dm_can1_2
上     | 中    |   左上下     | pitch轴         | pitch        | dm_can1_4
上     | 下    |   左上下     |roll轴           | roll         | can2_1

*左拨杆向中，xzy的肘部位置变化*
------------------------------------------------------------------------------
左拨杆 | 右拨杆 | 摇杆         | 功能            | En           | CAN_ID
----  | ------ | ----------- | --------------- | ------------ | -----
中     | 上    |   左上下     |出爪x            | stretch      | can2_3
中     | 中    |   左上下     |抬升z            | lift         | can1_3 & can1_4 
中     | 下    |   左上下     |横移y            | slid         | can2_2

*左拨杆向下，功能性配置的状态切换*
------------------------------------------------------------------------------
左拨杆 | 右拨杆 | 摇杆         | 功能            | En           | CAN_ID
----  | ------ | ----------- | --------------- | -------------- | -----
下     | 上    |   左上下     |云台模式         | gimbal?       | non
下     | 中    |   左上下     |non             | non           | non
下     | 下    |   左上下     |底盘断电         | non           | non

*其他功能*
-----------------------------------------------------------------------------
左拨杆 | 右拨杆 | 摇杆         | 功能            | En           | CAN_ID
----  | ------ | ----------- | --------------- | ------------ | -------------
无     | 无    |   拨轮       | 吸盘开关         | non          | non

## 键鼠


键盘    |   鼠标     |   功能
------  | --------  | --------
F       | 滚轮      | 救援
G       |           | 地矿
B       | 滚轮      | 出卡
Z       |           | 自动取矿
X       |           | 自动收矿
C       |           | 自动出矿
V       |           | 自动兑换
------------------------------------------------------------------


    手动自动        功能            按键（应是长按时开启模式）

    手动            救援出回            （暂时闲置）
    手动            吸盘开关            shift + f（开）/g（关）//待测试
    手动            图传yaw旋转         shift + q/e     
    手动            图传pitch          ctrl + q/e（归到中档可能需要多试几次）    

    自动            归位                   z  
    自动            小资源岛取矿抬升        x  
    自动            小资源岛取矿前爪        v
    自动            小资源岛抬矿            c


    自动            大资源岛                space
    自动            兑矿基础位              r         

## 气路设计

    24 唯一的设计就是气泵和一个电磁阀，没了。还是得要两个继电器
    md

## 代码结构简介
arm_ctrl 

algorithm 中的 matrix robotic utils 均为从上交开源直接嫖的
    希望未来能用上


# **23赛季内容**

## 代码结构简介
22赛季将代码结构重置，大体结构和英雄步兵等保持一致
大部分编写代码在user_code中
module文件包含绝大部分的控制和解算

    auto    并未调试完成使用的自动程序代码，每节完成需要向上板发送指令确定进行下一步（由于同时进行机械结构会冲突，有可能击杀自己）

    chassis     底盘控制解算

    communicate    本赛季新加的底盘上板之间的板间通讯（之前是使用遥控器数据线一分为二的方式同时控制两块板）

    config  部分重要参数放置位置

    lift    为了减轻上板使用电机过多导致通道拥挤将抬升放在底盘控制，以及一个吸盘控制

application文件夹为freertos使用服务
algorithm放置算法

    first_order_filter  一节低通滤波，如用
    pid     不作过多描述



## 底盘构成

底盘电机：can2  ID 为1 2 3 4 右前，左前，左后，右后
    
    ///////
    2     1
    3     4   
    ///////
