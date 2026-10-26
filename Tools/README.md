# Tools —— 项目内开发工具

本目录放工程自带的 Keil 编译脚本和主机侧控制逻辑测试。

## build.ps1 —— 命令行编译

在仓库根目录执行:

```powershell
# 增量编译(默认)
powershell -File Tools\build.ps1

# 全部重新编译
powershell -File Tools\build.ps1 -Rebuild

# 编译其他工程 / 指定 uVision 路径(机器不同时)
powershell -File Tools\build.ps1 -Project "E:\...\xxx.uvprojx" -UV4 "D:\Keil_v5\UV4\UV4.exe"
```

要点:

- 编译日志写到 `Tools\logs\<工程名>_build.log`;
- 退出码:0 = 编译成功(无错误;UV4 的 0=干净 / 1=仅有警告 都算成功),
  非 0 = 编译失败(UV4 返回 >=2);
- 唯一环境依赖是 uVision 的 `UV4.exe` 路径,默认 `E:\keil5\UV4\UV4.exe`,
  换机器用 `-UV4` 参数指定即可;
- 常见提示:链接警告 `L6314W: No section matches pattern *(.RAM_D3)` 是模板
  预存警告(工程未使用 `__RAM_D3_` 段),不影响编译。

## 机械臂状态测试

```powershell
python Tools/tests/test_arm_states.py
# 指定主机 C 编译器
python Tools/tests/test_arm_states.py --cc gcc
```

需要 Python 3 和主机 GCC。测试在临时目录编译真实的机械臂控制、头文件及运动学代码，
仅替换硬件接口；覆盖四种 FK/IK 开关组合、两组零位/比例、旋转切换时长、反馈偏差、
静止旋转时位置指令保持和业务/调试模式切换。测试不会改写固件配置或工程文件，
不替代 Keil 固件编译及上机角度标定。
