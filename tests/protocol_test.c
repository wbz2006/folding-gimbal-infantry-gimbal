#include <stdio.h>
#include <stddef.h>
#include "robot_def.h"
_Static_assert(sizeof(chassis_mode_e) == 4, "enum wire width changed");
_Static_assert(sizeof(Chassis_Ctrl_Cmd_s) == 24, "chassis command length changed");
_Static_assert(sizeof(Chassis_Upload_Data_s) == 65, "chassis feedback length changed");
_Static_assert(CHASSIS_FOLDED_ROTATE == 4, "legacy enum value changed");
_Static_assert(offsetof(Gimbal_Upload_Data_s, fold_state) == sizeof(attitude_t) + 2 * sizeof(float), "local angle layout");
_Static_assert(sizeof(App_Fault_s) == 4, "fault layout changed");
_Static_assert(offsetof(Chassis_Upload_Data_s, fault) == 49, "chassis fault offset changed");
_Static_assert(sizeof(Gimbal_Upload_Data_s) == 58, "gimbal feedback layout changed");
_Static_assert(sizeof(Shoot_Upload_Data_s) == 4, "shoot feedback layout changed");
int main(void)
{
    printf("wire:%zu,%zu,%zu,%zu,%zu,%zu,%zu,%zu;enum:%d,%d,%d,%d,%d;local:%zu,%zu,%zu\n",
        sizeof(Chassis_Ctrl_Cmd_s), offsetof(Chassis_Ctrl_Cmd_s,vx), offsetof(Chassis_Ctrl_Cmd_s,vy),
        offsetof(Chassis_Ctrl_Cmd_s,wz), offsetof(Chassis_Ctrl_Cmd_s,offset_angle),
        offsetof(Chassis_Ctrl_Cmd_s,chassis_mode), offsetof(Chassis_Ctrl_Cmd_s,chassis_speed_buff),
        sizeof(Chassis_Upload_Data_s), CHASSIS_ZERO_FORCE, CHASSIS_ROTATE, CHASSIS_NO_FOLLOW,
        CHASSIS_FOLLOW_GIMBAL_YAW, CHASSIS_FOLDED_ROTATE, sizeof(Gimbal_Ctrl_Cmd_s),
        sizeof(Gimbal_Upload_Data_s), sizeof(attitude_t));
}
