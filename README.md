# SolidWorks 手指机构角度与坐标扫描插件

在 SolidWorks 装配体中驱动两个角度配合，记录选定指尖点的三维坐标，用于建立关节角度与指尖位置之间的数据映射。

## 主要功能

- 设置两个关节的扫描范围及粗、中、细步长，也可固定一个关节。
- 快速扫描并导出目标角度、角度读数、到达状态及点坐标 XYZ。
- 扫描时可开启或关闭干涉计算。
- 递归搜索已加载子装配中的角度配合。
- 通过 `Pick J1 mate` / `Pick J2 mate` 直接选择配合，避免同名配合造成歧义。
- 选择扫描点，并支持平面角度测量、实时点与角度跟踪。

直接选择配合时，角度读数优先来自配合的 D1 尺寸。该读数与机构求解后的几何夹角并非始终等价，约束冲突时仍需结合模型状态检查采样结果。

## 项目文件

主要项目位于 `SW_Plugin_v3/SW_Plugin_fixed/solidworks_plugin/`：

- `SWAnkleInterference.cs`：当前 C# 插件源码。
- `SWAnkleInterference.csproj`：x64 / .NET Framework 4.7.2 项目。
- `Properties/AssemblyInfo.cs`：程序集信息。
- `register.bat`：COM 插件注册脚本。
- `SWAnkleInterference.py`：保留的 Python 版本，功能以当前 C# 插件为准。
- 子目录中的 `README.md`：早期版本说明，当前构建与使用以本页为准。

本地采样数据、图表、压缩包和编译输出不纳入版本控制。

## 构建

需要 Windows、SolidWorks、Visual Studio / Build Tools，以及 .NET Framework 4.7.2 开发工具。

从本机 SolidWorks 安装目录的 `api/redist` 中获取以下程序集，放到项目目录的 `bin/Debug/` 中，以匹配当前项目的引用路径：

- `SolidWorks.Interop.sldworks.dll`
- `SolidWorks.Interop.swconst.dll`
- `SolidWorks.Interop.swpublished.dll`

这些 SolidWorks 程序集不随源码仓库分发。使用安装版本对应的 API 文件。

在 Visual Studio 打开项目，选择 `Release`、`x64` 并生成，或在开发者命令行执行：

```powershell
msbuild "SW_Plugin_v3/SW_Plugin_fixed/solidworks_plugin/SWAnkleInterference.csproj" /p:Configuration=Release /p:Platform=x64
```

生成文件位于项目目录的 `bin/Release/`。

## 安装与使用

1. 关闭 SolidWorks，构建当前源码。
2. 在项目目录中以管理员身份运行 `register.bat`。
3. 重新打开 SolidWorks，在插件列表中启用本插件。
4. 打开总装配；需要运动的子装配设为 Flexible，并确保组件已解析且未被压缩。
5. 在特征树中选中关节配合，分别点击 `Pick J1 mate`、`Pick J2 mate`。
6. 选中要记录的指尖点，点击 `Pick scan point`。
7. 设置角度范围与步长；只采集坐标时可关闭干涉计算。
8. 开始扫描，完成后导出 CSV。

注册脚本会检查 DLL 是否早于源码；源码修改后需要重新构建。注册使用当前 DLL 的文件路径，注册后移动项目目录需要重新注册。
