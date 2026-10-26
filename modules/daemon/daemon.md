# daemon

<p align='right'>neozng1@hnu.edu.cn</p>

用于监测模块和应用运行情况的module(和官方代码中的deteck task作用相同)

## 使用范例

要使用该module，则包含`daemon.h`的头文件，并在使用daemon的文件中保留一个daemon的指针。

初始化的时候，要传入以下参数：

```c
typedef struct
{
    uint16_t reload_count;     // 超时daemon周期数
    uint16_t init_count;       // 可选的启动计数
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用
    void *owner_id;            // id取拥有daemon的实例的地址,如DJIMotorInstance*,cast成void*类型
    uint8_t require_first_reload; // 首个有效数据前保持离线
} Daemon_Init_Config_s;
```

`reload_count`是“喂狗”时的重载值，单位是 `DaemonTask()` 调用次数。当前 `StartDAEMONTASK()` 每 10 ms 调用一次，因此可以用 `DAEMON_COUNT_FROM_MS(timeout_ms)` 将毫秒超时转换为计数。实际超时受任务调度周期影响。

设置 `require_first_reload = 1` 后，首次调用 `DaemonReload()` 前始终返回离线；之后计数降为零时转为离线，并只触发一次离线回调。适合遥控器、传感器、电机反馈和应用消息等必须先收到真实数据的来源。默认值为 0，保留原有启动宽限行为。

`offline_callback`是模块离线的回调函数，即当包含daemon的模块发生离线情况时，该函数会被调用以应对离线情况，如重启模块，重新初始化等。如果没有则传入`NULL`即可。

daemon还提供application状态汇总接口：application继续判断自身的复合反馈有效性和业务故障，通过`DaemonSetFeedbackValid()`、`DaemonSetFaultFlags()`或增量的`DaemonAddFaultFlags()`/`DaemonClearFaultFlags()`写入所属daemon，再由`DaemonIsFeedbackValid()`和`DaemonGetFaultFlags()`读取并填入反馈结构。daemon不解释故障位含义；故障位定义和跨应用反馈结构保留在`robot_def.h`，业务状态机仍由对应application负责。

`owner_id`即模块取自身地址并通过强制类型转换化为`void*`类型，用于拥有多个实例的模块在`offline_callback`中区分自身。如多个电机都使用一个相同的`offline_callback`，那么在调用回调函数的时候就可以通过该指针来访问某个特定的电机。

> 这种方法也称作“parent pointer”，即**保存拥有指向自身的指针对象的地址**。这样就可以在特定的情况下通过自身来访问自己的父对象。



## 具体实现

`DaemonTask()` 由 `StartDAEMONTASK()` 以 100 Hz 调用。注释详细，请参见 `daemon.c`。
