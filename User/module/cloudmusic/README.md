# 音符播放与 CloudMusic

这两个服务只接收单调毫秒时间和同步音符输出回调，不依赖 HAL、RTOS、DR16 或内置曲库。

- `module/audio/player.h`：非阻塞音符/间隔/单曲循环、暂停续播、停止；不轮询线程、不占用静态全局播放器。
- `module/cloudmusic/cloudmusic.h`：外部播放列表、可选启动曲、上一首/下一首、暂停、停止、顺序播放与列表循环。
- `component/audio/score.h`：纯数据的 `Tone_t`、`Audio_Score_t`。休止符为 `NOTE_REST`；音符持续时间必须大于零。
- `resources/music`：可选曲谱，每首曲目独立 `.c`，模块不自动引用。
- `examples/music/app`：PWM 接入和归一化遥控菜单示例。

`Audio_PlayerInit` / `CloudMusic_Init` 不访问输出。对已经使用的实例重新 Init 之前，应用必须 Stop；播放器、回调上下文、播放列表和音符数组必须在整个使用期间有效。所有方法在同一调度上下文调用，跨线程事件由 app 排队。

`now_ms` 是毫秒，不是 RTOS tick；轮询、暂停及恢复时间差须小于 `2^31` ms。暂停保存当前音符或间隔的剩余时间，暂停启动不先发声。稀疏轮询会延长节拍，不会一次性突发补播历史音符；应按音符精度要求安排 Update 周期。

输出回调同步接受 `(context, note, octave)`，返回零表示成功，休止符表示静音。设备错误会停止播放、保留错误并尝试静音；不会把失败当成曲终而自动换歌。修复后显式 Start/Play/Next 可以重试。Stop 优先于同一批中的其他控制事件，停止后 Update 不会自行重启。

CloudMusic 的配置为空列表且无启动曲时保持静音；没有固定默认菜单、特性宏或编译时启动曲。`start_paused` 同时适用于启动曲和列表；`loop_playlist=false` 播到列表结尾后停止，手工上一首/下一首仍可环绕。遥控拨杆、边沿防抖、播放与告警的优先级由 app 决定。

旧的 `CloudMusic_Input_t`、`MUSIC_t`、特性宏和 `BUZZER_Player*/PlayScore/PlayTone` 已删除。迁移后用 `Audio_Player*` 播单曲，用 CloudMusic 管理列表，蜂鸣器设备只输出 PWM/音符；接入示例见 `examples/music/README.md`。
