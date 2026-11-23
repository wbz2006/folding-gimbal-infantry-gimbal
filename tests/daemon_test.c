#include <assert.h>
#include <stdio.h>
#include "daemon.h"

static int offline_calls;
static int owner;

static void OfflineCallback(void *owner_id)
{
    assert(owner_id == &owner);
    ++offline_calls;
}

uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t value) { (void)value; }
void CANServiceTask(void) {}
void BuzzerInit(void) {}
void BuzzerTask(void) {}

int main(void)
{
    assert(DAEMON_COUNT_FROM_MS(20U) == 2U);
    assert(DAEMON_COUNT_FROM_MS(50U) == 5U);
    assert(DAEMON_COUNT_FROM_MS(100U) == 10U);
    Daemon_Init_Config_s config = {
        .reload_count = 3,
        .callback = OfflineCallback,
        .owner_id = &owner,
        .require_first_reload = 1,
    };
    DaemonInstance *daemon = DaemonRegister(&config);
    assert(daemon != NULL && !DaemonIsOnline(daemon) && !DaemonHasReloaded(daemon));
    assert(!DaemonIsFeedbackValid(daemon));
    assert(DaemonGetFaultFlags(daemon) == 0);
    DaemonSetFeedbackValid(daemon, 1);
    assert(DaemonIsFeedbackValid(daemon));
    DaemonSetFaultFlags(daemon, 0x5U);
    DaemonAddFaultFlags(daemon, 0x2U);
    assert(DaemonGetFaultFlags(daemon) == 0x7U);
    DaemonClearFaultFlags(daemon, 0x1U);
    assert(DaemonGetFaultFlags(daemon) == 0x6U);
    DaemonSetFaultFlags(daemon, 0);
    DaemonSetFeedbackValid(daemon, 0);
    for (int i = 0; i < 5; ++i)
        DaemonTask();
    assert(!DaemonIsOnline(daemon) && offline_calls == 0); // 首次有效数据前离线，不触发恢复回调

    DaemonReload(daemon);
    assert(DaemonIsOnline(daemon) && DaemonHasReloaded(daemon));
    DaemonTask(); DaemonTask();
    assert(DaemonIsOnline(daemon));
    DaemonTask();
    assert(!DaemonIsOnline(daemon) && offline_calls == 0);
    DaemonTask();
    assert(offline_calls == 1);
    DaemonTask(); DaemonTask();
    assert(offline_calls == 1); // 离线期间只通知一次

    DaemonReload(daemon);
    assert(DaemonIsOnline(daemon));
    for (int i = 0; i < 3; ++i)
        DaemonTask();
    assert(!DaemonIsOnline(daemon));
    DaemonTask();
    assert(offline_calls == 2); // 新鲜数据恢复后再次离线可重新报告
    puts("PASS daemon freshness: first-sample gating, timeout edge, callback latch and recovery");
}
