#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include REMOTE_SOURCE
UART_HandleTypeDef huart1, huart5, huart9;
USARTInstance *USARTRegister(USART_Init_Config_s *config)
{ (void)config; static USARTInstance instance; return &instance; }
void USARTServiceInit(USARTInstance *instance) { (void)instance; }
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    DaemonInstance *d = calloc(1, sizeof(*d));
    d->reload_count = config->reload_count; d->temp_count = config->reload_count;
    d->has_reloaded = !config->require_first_reload;
    return d;
}
void DaemonReload(DaemonInstance *d)
{ d->temp_count = d->reload_count; d->has_reloaded = 1; d->offline_reported = 0; }
uint8_t DaemonIsOnline(DaemonInstance *d) { return d && d->has_reloaded && d->temp_count; }
int main(void)
{
    assert(!RemoteControlIsOnline());
    RemoteControlInit(&huart5);
    assert(!DaemonIsOnline(rc_daemon_instance));
    assert(!RemoteControlIsOnline()); // 首个有效遥控器帧前保持离线
    RemoteControlRxCallback();
    assert(RemoteControlIsOnline());
    rc_daemon_instance->temp_count = 0;
    assert(!RemoteControlIsOnline());
    RCLostCallback(NULL);
    assert(!RemoteControlIsOnline());
    RemoteControlRxCallback();
    assert(RemoteControlIsOnline());
    puts("PASS actual RemoteControlIsOnline: startup, first frame, offline, lost callback, fresh-frame recovery");
}
