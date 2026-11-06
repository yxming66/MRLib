# 可选音乐资源

每首曲谱独立为一个 `.c`，声明位于 `music.h`。只编译/链接应用播放列表引用的曲目，通用播放器和 CloudMusic 不会引入任何曲谱。
`score_macros.h` 是这些资源共享的节拍常量；`scores/*.inc` 保留原曲谱数据。

例如 `nokia.c` 导出 `mr_music_nokia`；`rm.c` 导出 `mr_music_rm`。
更多符号以 `music.h` 为准。曲目数据未经本次改写，仅改变资源装配方式。
