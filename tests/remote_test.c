#include <assert.h>
#include <stdio.h>
#include REMOTE_SOURCE
UART_HandleTypeDef huart1, huart5, huart9;
USARTInstance *USARTRegister(USART_Init_Config_s *config)
{ (void)config; static USARTInstance instance; return &instance; }
void USARTServiceInit(USARTInstance *instance) { (void)instance; }
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t value) { (void)value; }
void CANServiceTask(void) {}
int main(void)
{
    assert(!RemoteControlIsOnline());
    RemoteControlInit(&huart5);
    assert(!RemoteControlIsOnline());
    // DBUS: neutral sticks/dial (1024), left middle (3), right up (1).
    const uint8_t frame[18] = {0x00, 0x04, 0x20, 0x00, 0x01, 0xd8,
                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x04};
    memcpy(rc_usart_instance->recv_buff, frame, sizeof(frame));
    rc_usart_instance->recv_len = 17;
    RemoteControlRxCallback();
    assert(!RemoteControlIsOnline());
    rc_usart_instance->recv_len = 18;
    RemoteControlRxCallback();
    assert(RemoteControlIsOnline());
    assert(rc_ctrl[TEMP].rc.rocker_l_ == 0 && rc_ctrl[TEMP].rc.rocker_l1 == 0);
    assert(rc_ctrl[TEMP].rc.rocker_r_ == 0 && rc_ctrl[TEMP].rc.rocker_r1 == 0);
    assert(rc_ctrl[TEMP].rc.dial == 0);
    RC_ctrl_t saved = rc_ctrl[TEMP];
    for (unsigned length = 0; length <= 19; ++length)
    {
        if (length == 18) continue;
        rc_daemon_instance->temp_count = 1;
        rc_usart_instance->recv_len = length;
        RemoteControlRxCallback();
        assert(rc_daemon_instance->temp_count == 1);
        assert(memcmp(&saved, &rc_ctrl[TEMP], sizeof(saved)) == 0);
    }
    rc_usart_instance->recv_len = 18;
    for (int left = 0; left <= 3; ++left)
        for (int right = 0; right <= 3; ++right)
        {
            rc_daemon_instance->temp_count = 1;
            rc_usart_instance->recv_buff[5] = 0x08 | (left << 6) | (right << 4);
            RemoteControlRxCallback();
            if (left && right)
            {
                assert(rc_daemon_instance->temp_count == rc_daemon_instance->reload_count);
                assert(rc_ctrl[TEMP].rc.switch_left == left && rc_ctrl[TEMP].rc.switch_right == right);
                saved = rc_ctrl[TEMP];
            }
            else
            {
                assert(rc_daemon_instance->temp_count == 1);
                assert(memcmp(&saved, &rc_ctrl[TEMP], sizeof(saved)) == 0);
            }
        }
    rc_usart_instance->recv_buff[5] = 0x08;
    for (unsigned i = 0; i <= rc_daemon_instance->reload_count; ++i)
    {
        RemoteControlRxCallback();
        DaemonTask();
    }
    assert(!RemoteControlIsOnline());
    memcpy(rc_usart_instance->recv_buff, frame, sizeof(frame));
    RemoteControlRxCallback();
    assert(RemoteControlIsOnline());
    puts("PASS production DBUS: valid frame, all lengths/switches, invalid traffic timeout and recovery");
}
