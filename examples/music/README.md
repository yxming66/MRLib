# 音乐项目接入示例

1. board 初始化并通过 BSP_PWM_Configure 登记 PWM；蜂鸣器选择允许变频的独占 timer，app 将 BSP_PWM_FindEndpoint 的结果传给 BUZZER_Init 并检查结果。见 [PWM 接入](../../docs/PWM_BINDINGS.md)。
2. 链接 `mr::module_cloudmusic`，将 `resources/music/nokia.c` 等选用的曲目添加到应用 target。
3. app 保存播放器和播放列表，并提供同步输出回调。`app/buzzer_sink.h` 展示了现有蜂鸣器设备的接法。

```c
#include "module/cloudmusic/cloudmusic.h"
#include "music.h"                 /* resources/music 加入 include 路径 */
#include "app/buzzer_sink.h"

static CloudMusic_t music;
static const Audio_Score_t* const playlist[] = {&mr_music_nokia, &mr_music_rm};
/* buzzer 为已初始化的 app 持有设备；now_ms 为毫秒 */
CloudMusic_Config_t config = {playlist, 2, NULL, true, false};
Audio_Sink_t sink = {&buzzer, MusicBuzzerSink};
/* 逐项检查返回值 */
CloudMusic_Init(&music, sink, &config);
CloudMusic_Start(&music, now_ms);
/* 周期调用 */
CloudMusic_Update(&music, now_ms);
```

```cmake
target_link_libraries(robot_firmware PRIVATE mr::module_cloudmusic)
target_sources(robot_firmware PRIVATE
    external/MR_TestLib/resources/music/nokia.c
    external/MR_TestLib/resources/music/rm.c)
target_include_directories(robot_firmware PRIVATE
    external/MR_TestLib/resources/music)
```

遥控侧先将通道值归一化到 [-1,1]，由 app 判断拨杆是否进入音乐菜单，再使用 `app/remote_menu.h` 产生一次性 `CloudMusic_Control_t`。DR16 的头文件和拨杆枚举无需进入播放器。
`toggle_pause_event` 是切换暂停，`play_event` 是播放/恢复；根据项目体验决定菜单含义。
告警抢占时先暂停音乐，再写告警输出；告警结束后显式恢复。不能让两个服务同时操作一个 PWM。
