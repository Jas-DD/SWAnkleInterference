# SolidWorks Fingertip Workspace Scanner

A SolidWorks add-in for exploring robotic finger motion and collecting the relationship between two joint angles and fingertip position in 3D.

Developed for a robotic hand project, this tool automates angle-mate sweeps inside a CAD assembly and exports position samples for workspace analysis and downstream control algorithms. The original interference checker evolved into a coordinate-sampling tool for finger and thumb mechanisms.

## Engineering Context

A linked finger mechanism does not necessarily reach every requested combination of joint angles. Manually moving the assembly and recording fingertip positions is time-consuming, especially when the joints live inside nested subassemblies.

This add-in brings joint selection, motion scanning, coordinate capture, and CSV export into one SolidWorks workflow. The resulting samples can support lookup tables and interpolation for mapping a desired fingertip position back to joint commands. Those control algorithms are downstream uses of the data, rather than part of this add-in.

## Features

- **Two-joint scanning:** configure angle ranges and coarse, medium, and fine steps, or hold one joint fixed.
- **Fingertip coordinate sampling:** select a point and record its XYZ position alongside target angles, angle readbacks, and reachability status.
- **Optional interference checks:** enable collision analysis when needed, or disable it for faster coordinate collection.
- **Nested assembly support:** recursively search loaded subassemblies for angle mates.
- **Direct mate selection:** use `Pick J1 mate` and `Pick J2 mate` to identify joints without relying only on names.
- **Additional measurement tools:** plane-angle measurement and real-time point and angle tracking.
- **CSV export:** take collected samples into plotting, analysis, and embedded-control workflows.

## Implementation

The current add-in is written in **C#**, targets **.NET Framework 4.7.2 / x64**, and uses the **SolidWorks COM API** with a **Windows Forms** interface.

The source project is located in [`SW_Plugin_v3/SW_Plugin_fixed/solidworks_plugin/`](SW_Plugin_v3/SW_Plugin_fixed/solidworks_plugin/).

| File | Purpose |
| --- | --- |
| `SWAnkleInterference.cs` | Current add-in implementation |
| `SWAnkleInterference.csproj` | C# project and build configuration |
| `Properties/AssemblyInfo.cs` | Assembly metadata |
| `register.bat` | COM registration script |
| `SWAnkleInterference.py` | Earlier Python implementation retained for reference |

The original `SWAnkleInterference` name remains in the assembly and source files. Local datasets, plots, distribution archives, and build outputs are excluded from this repository.

## Build

Requirements: Windows, SolidWorks, Visual Studio or MSBuild tools, and the .NET Framework 4.7.2 developer tools.

Copy the following assemblies from your SolidWorks installation's `api/redist` directory into the project's `bin/Debug/` directory, matching the reference paths in the current project:

- `SolidWorks.Interop.sldworks.dll`
- `SolidWorks.Interop.swconst.dll`
- `SolidWorks.Interop.swpublished.dll`

SolidWorks interop assemblies are not distributed with this repository. Use the API assemblies supplied with your installation.

Open the project in Visual Studio and build **Release / x64**, or run this command from a developer shell at the repository root:

```powershell
msbuild "SW_Plugin_v3/SW_Plugin_fixed/solidworks_plugin/SWAnkleInterference.csproj" /p:Configuration=Release /p:Platform=x64
```

Build output is written to the project's `bin/Release/` directory.

## Run a Scan

1. Close SolidWorks and build the current source.
2. Run `register.bat` in the project directory as an administrator.
3. Reopen SolidWorks and enable the add-in in the Add-ins dialog.
4. Open the top-level assembly. Set moving subassemblies to **Flexible** and ensure their components are resolved and not suppressed.
5. Select each joint's angle mate in the feature tree, then click `Pick J1 mate` or `Pick J2 mate`.
6. Select the fingertip point to record and click `Pick scan point`.
7. Set the joint ranges and step sizes. Disable interference calculation when only coordinate samples are needed.
8. Start the scan and export the results as CSV.

## Measurement Considerations

When a mate is selected directly, angle readback prioritizes its D1 dimension. A mate dimension is not always equivalent to the solved geometric angle, particularly when constraints conflict. Inspect the assembly state and validate collected samples before using them for control.

The registration script checks whether the DLL is older than the source. Rebuild after source changes, and register again if you move the project directory because COM registration uses the DLL's location.
