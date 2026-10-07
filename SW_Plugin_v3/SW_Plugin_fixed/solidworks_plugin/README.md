# SolidWorks Ankle Interference Checker Plugin

人形机器人脚踝关节干涉检查SolidWorks插件

## 功能

1. **干涉分析** - 按步长2°遍历两个关节的所有角度组合，检测干涉状态
2. **平面角度测量** - 测量任意两个平面之间的角度
3. **结果导出** - 导出CSV格式的分析报告

## 文件说明

- `SWAnkleInterference.py` - Python版本（需要pywin32）
- `SWAnkleInterference.cs` - C#版本（推荐，编译为DLL）

## 安装步骤（C#版本）

### 1. 创建Visual Studio项目

1. 打开 Visual Studio
2. 新建 **类库 (.NET Framework)** 项目
3. 目标框架选择 **.NET Framework 4.7.2** 或更高

### 2. 添加SolidWorks引用

1. 在解决方案资源管理器中右键 **引用**
2. 选择 **添加引用**
3. 浏览到SolidWorks安装目录，添加：
   - `SldWorks.dll` (通常在 `C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\`)`
   - `SwConst.dll`

### 3. 配置项目

1. 在 **项目属性** 中勾选：
   - ✅ 为COM互操作注册
   - ✅ 使程序集COM可见

2. 在 AssemblyInfo.cs 中添加：
```csharp
[assembly: ComVisible(true)]
```

### 4. 生成新GUID

1. 在 Visual Studio 中打开 **工具 > 创建 GUID**
2. 点击 **新建GUID**，复制新的GUID
3. 替换代码中的 `Guid("A1B2C3D4-E5F6-7890-ABCD-EF1234567890")`

### 5. 编译项目

1. **生成 > 重新生成解决方案**
2. 成功后在 `bin\Debug\` 或 `bin\Release\` 文件夹中生成 `SWAnkleInterference.dll`

### 6. 注册插件

**方法1：使用SolidWorks Tools**
1. 打开SolidWorks
2. **工具 > 插件**
3. 点击 **添加**
4. 浏览选择生成的 `SWAnkleInterference.dll`

**方法2：手动注册**
```batch
regasm /codebase "path\to\SWAnkleInterference.dll"
```

## 使用方法

### 配置关节名称

在代码中修改关节名称为你的模型中的实际名称：

```csharp
private string m_Joint1Name = "Joint1";  // 修改为实际名称
private string m_Joint2Name = "Joint2";  // 修改为实际名称
```

或者在运行界面中输入。

### 运行干涉分析

1. 打开SolidWorks装配体文件
2. 点击菜单 **Ankle Interference Tool > Run Analysis**
3. 等待分析完成（会显示进度）
4. 查看结果摘要
5. 导出CSV报告

### 测量平面角度

1. 在模型中选择第一个平面
2. 输入平面名称到 Plane 1 文本框
3. 输入第二个平面名称到 Plane 2 文本框
4. 点击 **Measure Angle**

## 配置参数

在代码中修改运动范围：

```csharp
private double m_MinAngle = -45.0;  // 最小角度
private double m_MaxAngle = 45.0;   // 最大角度
private double m_StepSize = 2.0;    // 步长
```

## 输出格式

CSV导出包含以下列：
- `Step` - 步骤编号
- `J1_Angle` - 关节1角度（度）
- `J2_Angle` - 关节2角度（度）
- `Interference` - 是否干涉（True/False）
- `Interference_Count` - 干涉数量
- `Components` - 干涉的组件名称
- `Timestamp` - 时间戳

## 技术支持

如有问题，请检查：
1. 是否正确添加了SolidWorks引用
2. COM注册是否成功
3. 关节名称是否与模型中一致
4. SolidWorks是否以管理员权限运行

## 版本

- v1.0 - 2026-03-18
- 初始版本
