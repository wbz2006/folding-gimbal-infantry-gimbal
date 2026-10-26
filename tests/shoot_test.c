#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../application/shoot/shoot.c"

UART_HandleTypeDef huart1, huart5, huart9;
CAN_HandleTypeDef hcan1, hcan2, hcan3;
static DJIMotorInstance test_motors[3];
static size_t motor_index;

uint32_t HAL_GetTick(void) { return 0; }
float DWT_GetDeltaT(uint32_t *count) { (void)count; return 0.001f; }
float DWT_GetTimeline_ms(void) { return 0.0f; }
void DWT_Delay(float seconds) { (void)seconds; }

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    DaemonInstance *d = calloc(1, sizeof(*d));
    d->reload_count = config->reload_count ? config->reload_count : 100;
    d->temp_count = config->init_count ? config->init_count : d->reload_count;
    d->has_reloaded = !config->require_first_reload;
    return d;
}

void DaemonReload(DaemonInstance *d)
{ if (d) { d->temp_count = d->reload_count; d->has_reloaded = 1; d->offline_reported = 0; } }
uint8_t DaemonIsOnline(DaemonInstance *d) { return d && d->has_reloaded && d->temp_count > 0; }
uint8_t DaemonHasReloaded(DaemonInstance *d) { return d && d->has_reloaded; }
void DaemonSetFeedbackValid(DaemonInstance *d, uint8_t valid) { if (d) d->feedback_valid = valid != 0; }
uint8_t DaemonIsFeedbackValid(DaemonInstance *d) { return d && d->feedback_valid; }
void DaemonSetFaultFlags(DaemonInstance *d, uint32_t flags) { if (d) d->fault_flags = flags; }
void DaemonAddFaultFlags(DaemonInstance *d, uint32_t flags) { if (d) d->fault_flags |= flags; }
void DaemonClearFaultFlags(DaemonInstance *d, uint32_t flags) { if (d) d->fault_flags &= ~flags; }
uint32_t DaemonGetFaultFlags(DaemonInstance *d) { return d ? d->fault_flags : UINT32_MAX; }

DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    (void)config;
    DJIMotorInstance *m = &test_motors[motor_index++];
    Daemon_Init_Config_s daemon_config = {.reload_count = 2, .require_first_reload = 1};
    m->daemon = DaemonRegister(&daemon_config);
    return m;
}
void DJIMotorStop(DJIMotorInstance *motor) { if (motor) motor->stopped = 1; }
void DJIMotorEnable(DJIMotorInstance *motor) { if (motor) motor->stopped = 0; }
void DJIMotorSetRef(DJIMotorInstance *motor, float ref) { if (motor) motor->ref = ref; }
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e loop) { (void)motor; (void)loop; }

static void PublishShootCommand(Shoot_Ctrl_Cmd_s command)
{
    static Publisher_t *publisher;
    if (publisher == NULL)
        publisher = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    PubPushMessage(publisher, &command);
}

int main(void)
{
    ShootInit();
    ShootTask();
    assert(shoot_feedback_data.fault_flags & APP_FAULT_COMMAND_STALE);
    assert(shoot_feedback_data.fault_flags & APP_FAULT_FEEDBACK_STALE);
    assert(friction_l->stopped && friction_r->stopped && loader->stopped);

    Shoot_Ctrl_Cmd_s command = {
        .shoot_mode = SHOOT_OFF,
        .load_mode = LOAD_STOP,
        .lid_mode = LID_OPEN,
        .friction_mode = FRICTION_OFF,
        .bullet_speed = BULLET_SPEED_NONE,
        .shoot_rate = 0.0f,
    };
    PublishShootCommand(command);
    DaemonReload(friction_l->daemon); DaemonReload(friction_r->daemon); DaemonReload(loader->daemon);
    ShootTask();
    assert(shoot_feedback_data.fault_flags == APP_FAULT_NONE);

    command.load_mode = (loader_mode_e)99;
    PublishShootCommand(command);
    ShootTask();
    assert(shoot_feedback_data.fault_flags & APP_FAULT_INVALID_COMMAND);
    assert(friction_l->stopped && friction_r->stopped && loader->stopped);

    command.load_mode = LOAD_STOP;
    PublishShootCommand(command);
    ShootTask();
    shoot_cmd_daemon->temp_count = 0;
    ShootTask();
    assert(shoot_feedback_data.fault_flags & APP_FAULT_COMMAND_STALE);
    assert(friction_l->stopped && friction_r->stopped && loader->stopped);
    puts("PASS production shooter: first command gating, motor feedback fault, invalid command and stale command stop");
}
