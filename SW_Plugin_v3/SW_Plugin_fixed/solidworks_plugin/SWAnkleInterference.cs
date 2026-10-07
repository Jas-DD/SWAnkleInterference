// ========================================================================
// SolidWorks Ankle Mechanism Interference Checker
// 基于 SW2024 实际 API 编写，所有接口均经过验证可编译
//
// 关节驱动策略：
//   使用 IAssemblyDoc.EditMate4() 直接修改角度/限位配合参数驱动装配体到目标角度。
//   用户在面板里填写“配合名称”（特征树里看到的名称），例如 LimitAngle1 / LimitAngle2。
//
// 干涉检测：
//   使用 IAssemblyDoc.InterferenceDetectionManager 的标准流程。
//
// 平面角度：
//   支持从当前选择集读取（推荐），或按名称查找。
// ========================================================================

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using System.Reflection;
using Microsoft.Win32;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.swpublished;

namespace SWAnkleInterference
{
    // =====================================================================
    // 运行时动态查找 SolidWorks 安装路径
    // =====================================================================
    internal static class SwPathResolver
    {
        public static string FindInstallPath()
        {
            try
            {
                using (RegistryKey baseKey = Registry.LocalMachine.OpenSubKey(@"SOFTWARE\SolidWorks"))
                {
                    if (baseKey == null) return null;
                    string best = null;
                    int bestYear = 0;
                    foreach (string sub in baseKey.GetSubKeyNames())
                    {
                        string[] parts = sub.Split(' ');
                        if (parts.Length >= 2 && int.TryParse(parts.Last(), out int yr) && yr > bestYear)
                        { bestYear = yr; best = sub; }
                    }
                    if (best == null) return null;
                    using (RegistryKey k = baseKey.OpenSubKey(best + @"\Setup"))
                        return k?.GetValue("SldWorks Path") as string;
                }
            }
            catch { return null; }
        }
    }

    // =====================================================================
    // 结果数据
    // =====================================================================
    public class InterferenceResult
    {
        public int      Step              { get; set; }
        public string   Level             { get; set; }
        public double   Joint1Angle       { get; set; }
        public double   Joint2Angle       { get; set; }
        public double   TargetJoint1Angle { get; set; } = double.NaN;
        public double   TargetJoint2Angle { get; set; } = double.NaN;
        public bool     IsInterference    { get; set; }
        public int      InterferenceCount { get; set; }
        public string   ComponentNames    { get; set; }
        public string   Warning           { get; set; }
        public DateTime Timestamp         { get; set; }
        public double   ProjAngYZDeg      { get; set; } = double.NaN;
        public double   ProjAngXZDeg      { get; set; } = double.NaN;
        public double   ProjAngXYDeg      { get; set; } = double.NaN;
        public double   LinePlaneNormalDeg { get; set; } = double.NaN;
        public double   DeltaX            { get; set; } = double.NaN;
        public double   DeltaY            { get; set; } = double.NaN;
        public double   DeltaZ            { get; set; } = double.NaN;
        public double   R11               { get; set; } = double.NaN;
        public double   R12               { get; set; } = double.NaN;
        public double   R13               { get; set; } = double.NaN;
        public double   R21               { get; set; } = double.NaN;
        public double   R22               { get; set; } = double.NaN;
        public double   R23               { get; set; } = double.NaN;
        public double   R31               { get; set; } = double.NaN;
        public double   R32               { get; set; } = double.NaN;
        public double   R33               { get; set; } = double.NaN;
        public double   PitchDeg          { get; set; } = double.NaN;
        public double   RollDeg           { get; set; } = double.NaN;
        public double   YawDeg            { get; set; } = double.NaN;
        public double   TrackXmm          { get; set; } = double.NaN;
        public double   TrackYmm          { get; set; } = double.NaN;
        public double   TrackZmm          { get; set; } = double.NaN;
    }

    public class RealtimeTrackSample
    {
        public int      Index          { get; set; }
        public DateTime Timestamp      { get; set; }
        public double   ElapsedSeconds { get; set; }
        public double   Xmm            { get; set; } = double.NaN;
        public double   Ymm            { get; set; } = double.NaN;
        public double   Zmm            { get; set; } = double.NaN;
        public double   AngleDeg       { get; set; } = double.NaN;
        public string   Warning        { get; set; }
    }

    // =====================================================================
    // 主插件类
    // =====================================================================
    [ComVisible(true)]
    [Guid("A1B2C3D4-E5F6-7890-ABCD-EF1234567893")]
    [ProgId("SWAnkleInterference.AnkleInterferenceAddIn")]
    public class AnkleInterferenceAddIn : ISwAddin
    {
        #region 字段
        private ISldWorks        m_SwApp;
        private SldWorks         m_SwAppObj;
        private ICommandManager  m_CmdMgr;
        private int              m_iCookie;
        private readonly int     m_CmdGroupId = 100;
        private bool             m_CommandUiCreated;
        private BackgroundWorker m_Worker;
        private List<InterferenceResult> m_Results = new List<InterferenceResult>();
        private string           m_LiveCsvPath;
        private static bool      EnableVerboseDriveLog = false;
        private static bool      s_ResolverRegistered;
        private bool             m_EnableLinePlaneProjection;
        private byte[]           m_ProjLinePersist;
        private byte[]           m_ProjPlanePersist;
        private int              m_ProjLineType;
        private int              m_ProjPlaneType;
        private byte[]           m_ScanPointPersist;
        private byte[]           m_ScanPointComponentPersist;
        private int              m_ScanPointType;
        private string           m_ScanPointName;
        private byte[]           m_ScanJ1AnglePersist1;
        private byte[]           m_ScanJ1AnglePersist2;
        private byte[]           m_ScanJ1AngleComponentPersist1;
        private byte[]           m_ScanJ1AngleComponentPersist2;
        private int              m_ScanJ1AngleType1;
        private int              m_ScanJ1AngleType2;
        private byte[]           m_ScanJ2AnglePersist1;
        private byte[]           m_ScanJ2AnglePersist2;
        private byte[]           m_ScanJ2AngleComponentPersist1;
        private byte[]           m_ScanJ2AngleComponentPersist2;
        private int              m_ScanJ2AngleType1;
        private int              m_ScanJ2AngleType2;
        private readonly List<RealtimeTrackSample> m_RealtimeTrackSamples = new List<RealtimeTrackSample>();
        private byte[]           m_TrackPointPersist;
        private byte[]           m_TrackPointComponentPersist;
        private byte[]           m_TrackAnglePersist1;
        private byte[]           m_TrackAnglePersist2;
        private byte[]           m_TrackAngleComponentPersist1;
        private byte[]           m_TrackAngleComponentPersist2;
        private int              m_TrackPointType;
        private int              m_TrackAngleType1;
        private int              m_TrackAngleType2;
        private string           m_TrackPointName;
        private string           m_TrackAngleName1;
        private string           m_TrackAngleName2;
        private string           m_Joint1PickedMateName;
        private string           m_Joint2PickedMateName;
        private string           m_Joint1PickedMateDocPath;
        private string           m_Joint2PickedMateDocPath;
        #endregion

        #region 配置
        // 对应装配体中的 LimitMate/AngleMate 配合名称（直接填特征树中的名字即可）
        // 兼容旧写法：如果用户仍填了 "D1@配合名"，会在 SetJointAngle 内自动截取 '@' 后的配合名。
        public string Joint1DimName  = "AngleMate1";
        public string Joint2DimName  = "AngleMate2";
        public double J1Min        = -45.0;
        public double J1Max        =  45.0;
        public double J2Min        = -45.0;
        public double J2Max        =  45.0;
        public double StepSize     =  15.0;
        public double StepSizeCoarse = 15.0;
        public double StepSizeMedium = 7.50;
        public double StepSizeFine   = 3.25;
        public bool   FixJoint1    = false;
        public bool   FixJoint2    = false;
        public double FixedJ1Value =   0.0;
        public double FixedJ2Value =   0.0;
        public bool   EnableInterferenceCheck = true;
        public bool   EnableFastCoordinateAngleSampling = true;
        #endregion

        // ----------------------------------------------------------------
        // COM 注册 / 注销（SW2019+ 标准，不依赖 ISwAddin）
        // ----------------------------------------------------------------
        [ComRegisterFunction]
        public static void RegisterFunction(Type t)
        {
            try
            {
                string guid = t.GUID.ToString("B").ToUpper();
                using (var rk = Registry.LocalMachine.CreateSubKey(
                    @"SOFTWARE\SolidWorks\AddIns\" + guid))
                {
                    rk.SetValue(null, 0);
                    rk.SetValue("Description", "脚踝干涉分析插件");
                    rk.SetValue("Title", "脚踝干涉工具");
                }
                using (var rk2 = Registry.CurrentUser.CreateSubKey(
                    @"SOFTWARE\SolidWorks\AddInsStartup\" + guid))
                {
                    rk2.SetValue(null, 0);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("COM 注册失败: " + ex.Message);
            }
        }

        [ComUnregisterFunction]
        public static void UnregisterFunction(Type t)
        {
            try
            {
                string guid = t.GUID.ToString("B").ToUpper();
                Registry.LocalMachine.DeleteSubKey(
                    @"SOFTWARE\SolidWorks\AddIns\" + guid, false);
                Registry.CurrentUser.DeleteSubKey(
                    @"SOFTWARE\SolidWorks\AddInsStartup\" + guid, false);
            }
            catch { }
        }

        // ----------------------------------------------------------------
        // SW 插件入口点
        // ----------------------------------------------------------------
        public bool ConnectToSW(object ThisSW, int Cookie)
        {
            try
            {
                void Log(string msg)
                {
                    try
                    {
                        var path = Path.Combine(Path.GetTempPath(), "SWAnkleInterference.log");
                        File.AppendAllText(path, DateTime.Now.ToString("s") + " " + msg + System.Environment.NewLine);
                    }
                    catch { }
                }

                if (!s_ResolverRegistered)
                {
                    s_ResolverRegistered = true;
                    AppDomain.CurrentDomain.AssemblyResolve += ResolveFromAddinDir;
                }

                m_SwAppObj = (SldWorks)ThisSW;
                m_SwApp = (ISldWorks)m_SwAppObj;
                m_iCookie = Cookie;

                // Critical for .NET add-ins: lets SolidWorks call back into this object safely.
                // Missing this is a common cause of silent load failure / unexpected crashes.
                m_SwApp.SetAddinCallbackInfo2(0, this, m_iCookie);

                // DO NOT create command UI here: SolidWorks can crash during add-in load
                // if toolbars/command groups are created too early.
                // Defer UI creation to the first idle tick.
                m_SwAppObj.OnIdleNotify += OnIdleNotify;
                Log("ConnectToSW ok (" + GetBuildVersionTag() + "); subscribed OnIdleNotify.");
                return true;
            }
            catch (Exception ex)
            {
                try
                {
                    var path = Path.Combine(Path.GetTempPath(), "SWAnkleInterference.log");
                    File.AppendAllText(path, DateTime.Now.ToString("s") + " ConnectToSW failed: " + ex + System.Environment.NewLine);
                }
                catch { }
                return false;
            }
        }

        public bool DisconnectFromSW()
        {
            if (m_Worker != null && m_Worker.IsBusy)
                m_Worker.CancelAsync();
            try { if (m_SwApp != null) m_SwApp.CommandInProgress = false; } catch { }
            try { if (m_SwAppObj != null) m_SwAppObj.OnIdleNotify -= OnIdleNotify; } catch { }
            RemoveCommandGroup();
            m_CmdMgr = null;
            m_SwApp  = null;
            m_SwAppObj = null;
            return true;
        }

        // Create toolbar/menu when SW is idle (UI initialized).
        private int OnIdleNotify()
        {
            if (m_CommandUiCreated || m_SwApp == null) return 0;
            m_CommandUiCreated = true;

            try
            {
                m_CmdMgr = m_SwApp.GetCommandManager(m_iCookie);
                AddCommandGroup();
            }
            catch
            {
                // If anything goes wrong here, don't keep retrying and don't crash SolidWorks.
            }
            finally
            {
                try { if (m_SwAppObj != null) m_SwAppObj.OnIdleNotify -= OnIdleNotify; } catch { }
            }
            return 0;
        }

        private void AddCommandGroup()
        {
            int errors = 0;
            ICommandGroup grp = m_CmdMgr.CreateCommandGroup2(
                m_CmdGroupId, "脚踝干涉工具", "脚踝关节干涉检测插件",
                "脚踝干涉工具", -1, true, ref errors);
            if (grp == null) return;
            grp.AddCommandItem2("打开分析面板", -1, "打开脚踝干涉分析面板",
                "打开面板", 0, "OpenAnalysisPanel", "OpenAnalysisPanelEnable", 0,
                (int)swCommandItemType_e.swMenuItem |
                (int)swCommandItemType_e.swToolbarItem);
            grp.HasToolbar = true;
            grp.HasMenu    = true;
            grp.Activate();
        }

        private void RemoveCommandGroup()
        {
            try { m_CmdMgr?.RemoveCommandGroup(m_CmdGroupId); } catch { }
        }

        private static System.Reflection.Assembly ResolveFromAddinDir(object sender, ResolveEventArgs e)
        {
            try
            {
                var dir = Path.GetDirectoryName(typeof(AnkleInterferenceAddIn).Assembly.Location);
                var name = new System.Reflection.AssemblyName(e.Name).Name + ".dll";
                var candidate = Path.Combine(dir ?? "", name);
                return File.Exists(candidate) ? System.Reflection.Assembly.LoadFrom(candidate) : null;
            }
            catch { return null; }
        }

        // SolidWorks command callback must be public and typically returns int.
        [ComVisible(true)]
        public int OpenAnalysisPanel()
        {
            try { new AnalysisPanel(this).Show(); } catch { }
            return 0;
        }

        [ComVisible(true)]
        public int OpenAnalysisPanelEnable() => 1;

        // ----------------------------------------------------------------
        // 驱动关节角度
        //
        // 通过 IAssemblyDoc.EditMate4() 直接修改配合的 Angle 值驱动。
        // 为保证每次都停在精确目标角度，将 AngleAbsUpperLimit/AngleAbsLowerLimit
        // 也同时收缩到目标 Angle。
        // ----------------------------------------------------------------
        private bool SetJointAngle(IModelDoc2 doc, IAssemblyDoc assy, string mateNameRaw, double angleDeg)
        {
            try
            {
                if (doc == null || assy == null) return false;
                if (string.IsNullOrWhiteSpace(mateNameRaw)) return false;

                // 兼容旧写法：D1@MateName -> 截取 MateName
                string mateName = NormalizeMateName(mateNameRaw);

                // 角度配合/限位角度配合：通过“隐藏尺寸名”驱动（最稳，避免 EditMate4 签名差异）
                // - 普通角度配合：D1@MateName
                // - 限位角度配合：D1@MateName, $UPPERLIMIT_ANGLE@MateName, $LOWERLIMIT_ANGLE@MateName
                // 单位：SystemValue 统一使用弧度
                double rad = angleDeg * Math.PI / 180.0;

                string d1Name = "D1@" + mateName;
                string upName = "$UPPERLIMIT_ANGLE@" + mateName;
                string loName = "$LOWERLIMIT_ANGLE@" + mateName;

                bool okD1 = TrySetDimSystemValue(doc, d1Name, rad, out var d1Err);
                bool okUp = TrySetDimSystemValue(doc, upName, rad, out var upErr);
                bool okLo = TrySetDimSystemValue(doc, loName, rad, out var loErr);

                // Fallback: locate actual dimensions by selecting the mate feature and enumerating display dimensions.
                if (!okD1)
                {
                    var found = TryGetMateDimensionsBySelection(doc, mateName).ToList();
                    if (found.Count > 0)
                    {
                        int setCount = 0;
                        foreach (var (nm, dim) in found)
                        {
                            try { dim.SystemValue = rad; setCount++; } catch { }
                        }
                        if (EnableVerboseDriveLog)
                            LogStatic($"DriveAngle FALLBACK mate='{mateName}' angleDeg={angleDeg:F4} setDims={setCount} names=[{string.Join(" | ", found.Select(x => x.name))}]");
                        okD1 = setCount > 0;
                    }
                    else
                    {
                        LogStatic($"DriveAngle FAIL mate='{mateName}' angleDeg={angleDeg:F4} D1='{d1Name}' err='{d1Err}' upper='{upName}' err='{upErr}' lower='{loName}' err='{loName}' (no dims found by selection)");
                    }
                }

                if (!okD1)
                {
                    string subErr = "";
                    if (TrySetJointAngleInSubassemblies(doc, mateName, rad, angleDeg, out subErr))
                    {
                        try { doc.EditRebuild3(); } catch { }
                        return true;
                    }

                    LogStatic($"DriveAngle FAIL mate='{mateName}' angleDeg={angleDeg:F4} D1='{d1Name}' err='{d1Err}' upper='{upName}' ok={okUp} err='{upErr}' lower='{loName}' ok={okLo} err='{loErr}'");
                    if (!string.IsNullOrWhiteSpace(subErr))
                        LogStatic($"DriveAngle SUBASSEMBLY FAIL mate='{mateName}' detail='{subErr}'");
                    return false;
                }

                if (EnableVerboseDriveLog)
                {
                    if (okUp || okLo)
                        LogStatic($"DriveAngle LIMIT mate='{mateName}' angleDeg={angleDeg:F4} D1=OK upperOK={okUp} lowerOK={okLo}");
                    else
                        LogStatic($"DriveAngle ANGLE mate='{mateName}' angleDeg={angleDeg:F4} D1=OK (no limit dims found)");
                }

                // 触发重建，确保配合位置生效
                try { doc.EditRebuild3(); } catch { }
                return true;
            }
            catch
            {
                return false;
            }
        }

        private bool TrySetJointAngleInSubassemblies(IModelDoc2 rootDoc, string mateName, double rad, double angleDeg, out string error)
        {
            error = "";
            try
            {
                var visited = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
                visited.Add(GetDocumentIdentity(rootDoc));
                return TrySetJointAngleInSubassembliesRecursive(rootDoc, mateName, rad, angleDeg, visited, out error);
            }
            catch (Exception ex)
            {
                error = ex.Message;
                return false;
            }
        }

        private bool TrySetJointAngleInSubassembliesRecursive(IModelDoc2 parentDoc, string mateName, double rad, double angleDeg, HashSet<string> visited, out string error)
        {
            error = "";
            IAssemblyDoc parentAssy = parentDoc as IAssemblyDoc;
            if (parentAssy == null)
            {
                error = "parent is not assembly";
                return false;
            }

            object[] comps = null;
            try { comps = parentAssy.GetComponents(false) as object[]; } catch { }
            if (comps == null || comps.Length == 0)
            {
                error = "no subassembly components loaded";
                return false;
            }

            var errors = new List<string>();
            foreach (object item in comps)
            {
                IComponent2 comp = item as IComponent2;
                if (comp == null || IsComponentSuppressed(comp)) continue;

                IModelDoc2 childDoc = null;
                try { childDoc = comp.GetModelDoc2() as IModelDoc2; } catch { }
                if (childDoc == null || !IsAssemblyDocument(childDoc)) continue;

                string id = GetDocumentIdentity(childDoc);
                if (!visited.Add(id)) continue;

                if (TrySetJointAngleInDocument(childDoc, mateName, rad, angleDeg, out var childErr))
                {
                    if (EnableVerboseDriveLog)
                        LogStatic($"DriveAngle SUBASSEMBLY component='{SafeComponentName(comp)}' mate='{mateName}' angleDeg={angleDeg:F4}");
                    try { childDoc.EditRebuild3(); } catch { }
                    return true;
                }

                if (!string.IsNullOrWhiteSpace(childErr))
                    errors.Add(SafeComponentName(comp) + ": " + childErr);

                if (TrySetJointAngleInSubassembliesRecursive(childDoc, mateName, rad, angleDeg, visited, out childErr))
                    return true;

                if (!string.IsNullOrWhiteSpace(childErr))
                    errors.Add(SafeComponentName(comp) + ": " + childErr);
            }

            error = errors.Count > 0
                ? string.Join(" ; ", errors.Take(5))
                : "mate not found in loaded subassemblies; make sure subassembly is resolved and flexible";
            return false;
        }

        private bool TrySetJointAngleInDocument(IModelDoc2 doc, string mateName, double rad, double angleDeg, out string error)
        {
            error = "";
            try
            {
                string d1Name = "D1@" + mateName;
                string upName = "$UPPERLIMIT_ANGLE@" + mateName;
                string loName = "$LOWERLIMIT_ANGLE@" + mateName;

                bool okD1 = TrySetDimSystemValue(doc, d1Name, rad, out var d1Err);
                TrySetDimSystemValue(doc, upName, rad, out var upErr);
                TrySetDimSystemValue(doc, loName, rad, out var loErr);

                if (!okD1)
                {
                    var found = TryGetMateDimensionsBySelection(doc, mateName).ToList();
                    foreach (var item in found)
                    {
                        try { item.dim.SystemValue = rad; okD1 = true; } catch { }
                    }
                }

                if (!okD1)
                {
                    error = $"doc='{GetDocumentDebugName(doc)}' D1='{d1Name}' err='{d1Err}'";
                    return false;
                }

                if (EnableVerboseDriveLog)
                    LogStatic($"DriveAngle DOC doc='{GetDocumentDebugName(doc)}' mate='{mateName}' angleDeg={angleDeg:F4}");
                return true;
            }
            catch (Exception ex)
            {
                error = ex.Message;
                return false;
            }
        }

        private bool SetJointAngleForJoint(IModelDoc2 rootDoc, IAssemblyDoc rootAssy, int jointIndex, string mateNameRaw, double angleDeg)
        {
            string pickedMate = jointIndex == 1 ? m_Joint1PickedMateName : m_Joint2PickedMateName;
            string pickedPath = jointIndex == 1 ? m_Joint1PickedMateDocPath : m_Joint2PickedMateDocPath;

            if (!string.IsNullOrWhiteSpace(pickedMate))
            {
                double rad = angleDeg * Math.PI / 180.0;
                string err = "";
                if (TrySetPickedMateAngle(rootDoc, pickedMate, pickedPath, rad, angleDeg, out err))
                    return true;

                LogStatic($"DriveAngle PICKED FAIL joint={jointIndex} mate='{pickedMate}' doc='{pickedPath}' angleDeg={angleDeg:F4} err='{err}'");
            }

            return SetJointAngle(rootDoc, rootAssy, mateNameRaw, angleDeg);
        }

        private bool TrySetPickedMateAngle(IModelDoc2 rootDoc, string mateName, string docPath, double rad, double angleDeg, out string error)
        {
            error = "";
            try
            {
                if (!string.IsNullOrWhiteSpace(docPath))
                {
                    IModelDoc2 pickedDoc = FindLoadedDocumentByPath(rootDoc, docPath);
                    if (pickedDoc != null && TrySetJointAngleInDocument(pickedDoc, mateName, rad, angleDeg, out error))
                    {
                        try { pickedDoc.EditRebuild3(); } catch { }
                        try { rootDoc?.EditRebuild3(); } catch { }
                        return true;
                    }
                }

                if (TrySetJointAngleInDocument(rootDoc, mateName, rad, angleDeg, out error))
                    return true;

                return TrySetJointAngleInSubassemblies(rootDoc, mateName, rad, angleDeg, out error);
            }
            catch (Exception ex)
            {
                error = ex.Message;
                return false;
            }
        }

        private static IModelDoc2 FindLoadedDocumentByPath(IModelDoc2 rootDoc, string docPath)
        {
            if (rootDoc == null || string.IsNullOrWhiteSpace(docPath)) return null;
            try
            {
                string rootPath = "";
                try { rootPath = rootDoc.GetPathName(); } catch { }
                if (string.Equals(rootPath, docPath, StringComparison.OrdinalIgnoreCase))
                    return rootDoc;

                var visited = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
                visited.Add(GetDocumentIdentity(rootDoc));
                return FindLoadedDocumentByPathRecursive(rootDoc, docPath, visited);
            }
            catch
            {
                return null;
            }
        }

        private static IModelDoc2 FindLoadedDocumentByPathRecursive(IModelDoc2 parentDoc, string docPath, HashSet<string> visited)
        {
            IAssemblyDoc parentAssy = parentDoc as IAssemblyDoc;
            if (parentAssy == null) return null;

            object[] comps = null;
            try { comps = parentAssy.GetComponents(false) as object[]; } catch { }
            if (comps == null) return null;

            foreach (object item in comps)
            {
                IComponent2 comp = item as IComponent2;
                if (comp == null || IsComponentSuppressed(comp)) continue;

                IModelDoc2 childDoc = null;
                try { childDoc = comp.GetModelDoc2() as IModelDoc2; } catch { }
                if (childDoc == null || !IsAssemblyDocument(childDoc)) continue;

                string id = GetDocumentIdentity(childDoc);
                if (!visited.Add(id)) continue;

                string path = "";
                try { path = childDoc.GetPathName(); } catch { }
                if (string.Equals(path, docPath, StringComparison.OrdinalIgnoreCase))
                    return childDoc;

                IModelDoc2 found = FindLoadedDocumentByPathRecursive(childDoc, docPath, visited);
                if (found != null) return found;
            }

            return null;
        }

        private static bool TrySetDimSystemValue(IModelDoc2 doc, string dimFullName, double systemValue, out string error)
        {
            error = "";
            try
            {
                object paramObj = doc.Parameter(dimFullName);
                if (paramObj == null)
                {
                    error = "Parameter() returned null";
                    return false;
                }

                IDimension dim = paramObj as IDimension;
                if (dim == null)
                {
                    error = "Not an IDimension";
                    return false;
                }

                dim.SystemValue = systemValue;
                return true;
            }
            catch (Exception ex)
            {
                error = ex.Message;
                return false;
            }
        }

        private static string NormalizeMateName(string mateNameRaw)
        {
            if (string.IsNullOrWhiteSpace(mateNameRaw)) return "";

            string mateName = mateNameRaw.Trim();
            int at = mateName.IndexOf('@');
            if (at >= 0 && at < mateName.Length - 1)
                mateName = mateName.Substring(at + 1).Trim();
            return mateName;
        }

        private static bool IsAssemblyDocument(IModelDoc2 doc)
        {
            if (doc == null) return false;
            return doc is IAssemblyDoc;
        }

        private static bool IsComponentSuppressed(IComponent2 comp)
        {
            try
            {
                int state = comp.GetSuppression();
                return state == (int)swComponentSuppressionState_e.swComponentSuppressed;
            }
            catch
            {
                return false;
            }
        }

        private static string SafeComponentName(IComponent2 comp)
        {
            try { return comp.Name2 ?? "(component)"; } catch { }
            return "(component)";
        }

        private static string GetDocumentDebugName(IModelDoc2 doc)
        {
            if (doc == null) return "(null)";
            try
            {
                string title = doc.GetTitle();
                if (!string.IsNullOrWhiteSpace(title)) return title;
            }
            catch { }
            try
            {
                string path = doc.GetPathName();
                if (!string.IsNullOrWhiteSpace(path)) return path;
            }
            catch { }
            return "(unnamed)";
        }

        private static string GetDocumentIdentity(IModelDoc2 doc)
        {
            if (doc == null) return Guid.NewGuid().ToString();
            try
            {
                string path = doc.GetPathName();
                if (!string.IsNullOrWhiteSpace(path)) return path;
            }
            catch { }
            return GetDocumentDebugName(doc);
        }

        private static bool TryReadJointAngleDeg(IModelDoc2 doc, string mateNameRaw, out double angleDeg, out string error)
        {
            angleDeg = double.NaN;
            error = "";
            try
            {
                if (doc == null) { error = "doc is null"; return false; }
                if (string.IsNullOrWhiteSpace(mateNameRaw)) { error = "mate name is empty"; return false; }

                string mateName = NormalizeMateName(mateNameRaw);

                object paramObj = doc.Parameter("D1@" + mateName);
                IDimension dim = paramObj as IDimension;
                if (dim != null)
                {
                    angleDeg = dim.SystemValue * 180.0 / Math.PI;
                    return !double.IsNaN(angleDeg) && !double.IsInfinity(angleDeg);
                }

                var found = TryGetMateDimensionsBySelection(doc, mateName).ToList();
                foreach (var item in found)
                {
                    if (item.dim == null) continue;
                    angleDeg = item.dim.SystemValue * 180.0 / Math.PI;
                    return !double.IsNaN(angleDeg) && !double.IsInfinity(angleDeg);
                }

                if (TryReadJointAngleDegInSubassemblies(doc, mateName, out angleDeg, out error))
                    return true;

                if (string.IsNullOrWhiteSpace(error))
                    error = "angle dimension not found";
                return false;
            }
            catch (Exception ex)
            {
                error = ex.Message;
                return false;
            }
        }

        private static bool TryReadJointAngleDegInSubassemblies(IModelDoc2 rootDoc, string mateName, out double angleDeg, out string error)
        {
            angleDeg = double.NaN;
            error = "";
            try
            {
                var visited = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
                visited.Add(GetDocumentIdentity(rootDoc));
                return TryReadJointAngleDegInSubassembliesRecursive(rootDoc, mateName, visited, out angleDeg, out error);
            }
            catch (Exception ex)
            {
                error = ex.Message;
                return false;
            }
        }

        private static bool TryReadJointAngleDegInSubassembliesRecursive(IModelDoc2 parentDoc, string mateName, HashSet<string> visited, out double angleDeg, out string error)
        {
            angleDeg = double.NaN;
            error = "";
            IAssemblyDoc parentAssy = parentDoc as IAssemblyDoc;
            if (parentAssy == null)
            {
                error = "parent is not assembly";
                return false;
            }

            object[] comps = null;
            try { comps = parentAssy.GetComponents(false) as object[]; } catch { }
            if (comps == null || comps.Length == 0)
            {
                error = "no subassembly components loaded";
                return false;
            }

            var errors = new List<string>();
            foreach (object item in comps)
            {
                IComponent2 comp = item as IComponent2;
                if (comp == null || IsComponentSuppressed(comp)) continue;

                IModelDoc2 childDoc = null;
                try { childDoc = comp.GetModelDoc2() as IModelDoc2; } catch { }
                if (childDoc == null || !IsAssemblyDocument(childDoc)) continue;

                string id = GetDocumentIdentity(childDoc);
                if (!visited.Add(id)) continue;

                if (TryReadJointAngleDegInDocument(childDoc, mateName, out angleDeg, out var childErr))
                    return true;

                if (!string.IsNullOrWhiteSpace(childErr))
                    errors.Add(SafeComponentName(comp) + ": " + childErr);

                if (TryReadJointAngleDegInSubassembliesRecursive(childDoc, mateName, visited, out angleDeg, out childErr))
                    return true;

                if (!string.IsNullOrWhiteSpace(childErr))
                    errors.Add(SafeComponentName(comp) + ": " + childErr);
            }

            error = errors.Count > 0
                ? string.Join(" ; ", errors.Take(5))
                : "mate not found in loaded subassemblies";
            return false;
        }

        private static bool TryReadJointAngleDegInDocument(IModelDoc2 doc, string mateName, out double angleDeg, out string error)
        {
            angleDeg = double.NaN;
            error = "";
            try
            {
                object paramObj = doc.Parameter("D1@" + mateName);
                IDimension dim = paramObj as IDimension;
                if (dim != null)
                {
                    angleDeg = dim.SystemValue * 180.0 / Math.PI;
                    return !double.IsNaN(angleDeg) && !double.IsInfinity(angleDeg);
                }

                var found = TryGetMateDimensionsBySelection(doc, mateName).ToList();
                foreach (var item in found)
                {
                    if (item.dim == null) continue;
                    angleDeg = item.dim.SystemValue * 180.0 / Math.PI;
                    return !double.IsNaN(angleDeg) && !double.IsInfinity(angleDeg);
                }

                error = $"doc='{GetDocumentDebugName(doc)}' angle dimension not found";
                return false;
            }
            catch (Exception ex)
            {
                error = ex.Message;
                return false;
            }
        }

        private bool TryReadJointAngleForJoint(IModelDoc2 rootDoc, int jointIndex, string mateNameRaw, out double angleDeg, out string error)
        {
            angleDeg = double.NaN;
            error = "";

            string pickedMate = jointIndex == 1 ? m_Joint1PickedMateName : m_Joint2PickedMateName;
            string pickedPath = jointIndex == 1 ? m_Joint1PickedMateDocPath : m_Joint2PickedMateDocPath;

            if (!string.IsNullOrWhiteSpace(pickedMate))
            {
                IModelDoc2 pickedDoc = FindLoadedDocumentByPath(rootDoc, pickedPath);
                if (pickedDoc != null && TryReadJointAngleDegInDocument(pickedDoc, pickedMate, out angleDeg, out error))
                    return true;

                if (TryReadJointAngleDeg(rootDoc, pickedMate, out angleDeg, out error))
                    return true;
            }

            return TryReadJointAngleDeg(rootDoc, mateNameRaw, out angleDeg, out error);
        }

        private bool HasPickedJointMate(int jointIndex)
        {
            string pickedMate = jointIndex == 1 ? m_Joint1PickedMateName : m_Joint2PickedMateName;
            return !string.IsNullOrWhiteSpace(pickedMate);
        }

        public bool CaptureJointMateFromSelection(int jointIndex, out string message)
        {
            message = "";
            try
            {
                IModelDoc2 doc = m_SwApp?.ActiveDoc as IModelDoc2;
                if (doc == null)
                {
                    message = "No active SolidWorks document.";
                    return false;
                }

                ISelectionMgr selMgr = doc.SelectionManager as ISelectionMgr;
                if (selMgr == null || selMgr.GetSelectedObjectCount2(-1) < 1)
                {
                    message = "Select one angle mate in the FeatureManager tree first.";
                    return false;
                }

                object selObj = selMgr.GetSelectedObject6(1, -1);
                IFeature feat = selObj as IFeature;
                if (feat == null)
                {
                    message = "Selected object is not a mate feature. Select the angle mate in the tree.";
                    return false;
                }

                string mateName = feat.Name;
                if (string.IsNullOrWhiteSpace(mateName))
                {
                    message = "Could not read selected mate name.";
                    return false;
                }

                IModelDoc2 ownerDoc = TryGetFeatureOwnerDocument(feat) ?? doc;
                if (!DocumentContainsMateDimension(ownerDoc, mateName))
                    ownerDoc = FindLoadedDocumentContainingMate(doc, mateName) ?? ownerDoc;

                string ownerPath = "";
                try { ownerPath = ownerDoc.GetPathName(); } catch { }

                if (!DocumentContainsMateDimension(ownerDoc, mateName))
                {
                    message = "Selected mate does not expose an angle dimension named D1. Make sure it is an angle/limit angle mate.";
                    return false;
                }

                if (jointIndex == 1)
                {
                    m_Joint1PickedMateName = mateName;
                    m_Joint1PickedMateDocPath = ownerPath;
                    Joint1DimName = mateName;
                }
                else
                {
                    m_Joint2PickedMateName = mateName;
                    m_Joint2PickedMateDocPath = ownerPath;
                    Joint2DimName = mateName;
                }

                message = $"J{jointIndex} picked mate: {mateName}" +
                          (string.IsNullOrWhiteSpace(ownerPath) ? "" : $" | {Path.GetFileName(ownerPath)}");
                return true;
            }
            catch (Exception ex)
            {
                message = "Pick mate failed: " + ex.Message;
                return false;
            }
        }

        private static IModelDoc2 TryGetFeatureOwnerDocument(IFeature feat)
        {
            try
            {
                dynamic d = feat;
                object model = null;
                try { model = d.GetOwnerDocument(); } catch { }
                if (model is IModelDoc2 md) return md;
            }
            catch { }
            return null;
        }

        private static bool DocumentContainsMateDimension(IModelDoc2 doc, string mateName)
        {
            try
            {
                if (doc == null || string.IsNullOrWhiteSpace(mateName)) return false;
                object paramObj = doc.Parameter("D1@" + mateName);
                if (paramObj is IDimension) return true;
                return TryGetMateDimensionsBySelection(doc, mateName).Any();
            }
            catch
            {
                return false;
            }
        }

        private static IModelDoc2 FindLoadedDocumentContainingMate(IModelDoc2 rootDoc, string mateName)
        {
            try
            {
                if (DocumentContainsMateDimension(rootDoc, mateName)) return rootDoc;
                var visited = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
                visited.Add(GetDocumentIdentity(rootDoc));
                return FindLoadedDocumentContainingMateRecursive(rootDoc, mateName, visited);
            }
            catch
            {
                return null;
            }
        }

        private static IModelDoc2 FindLoadedDocumentContainingMateRecursive(IModelDoc2 parentDoc, string mateName, HashSet<string> visited)
        {
            IAssemblyDoc parentAssy = parentDoc as IAssemblyDoc;
            if (parentAssy == null) return null;

            object[] comps = null;
            try { comps = parentAssy.GetComponents(false) as object[]; } catch { }
            if (comps == null) return null;

            foreach (object item in comps)
            {
                IComponent2 comp = item as IComponent2;
                if (comp == null || IsComponentSuppressed(comp)) continue;

                IModelDoc2 childDoc = null;
                try { childDoc = comp.GetModelDoc2() as IModelDoc2; } catch { }
                if (childDoc == null || !IsAssemblyDocument(childDoc)) continue;

                string id = GetDocumentIdentity(childDoc);
                if (!visited.Add(id)) continue;

                if (DocumentContainsMateDimension(childDoc, mateName))
                    return childDoc;

                IModelDoc2 found = FindLoadedDocumentContainingMateRecursive(childDoc, mateName, visited);
                if (found != null) return found;
            }

            return null;
        }

        private static bool TryMeasureMateReferenceAngleDeg(IModelDoc2 doc, string mateNameRaw, out double angleDeg, out string error)
        {
            angleDeg = double.NaN;
            error = "";
            try
            {
                IFeature feat = TryFindMateFeature(doc, mateNameRaw);
                if (feat == null) { error = "mate feature not found"; return false; }

                object specific = null;
                try { specific = feat.GetSpecificFeature2(); } catch { }
                object[] refs = TryGetMateReferenceObjects(specific);
                if (refs == null || refs.Length < 2)
                {
                    object def = null;
                    try { def = feat.GetDefinition(); } catch { }
                    refs = TryGetMateReferenceObjects(def);
                }

                if (refs == null || refs.Length < 2)
                {
                    error = "mate reference entities not found";
                    return false;
                }

                string measureErr = "";
                if (!TryMeasureAngleBetweenObjects(doc, refs[0], refs[1], out angleDeg, out measureErr))
                {
                    error = measureErr;
                    return false;
                }

                angleDeg = NormalizeAngleDeg(angleDeg);
                angleDeg = ResolveMateComplementAngle(doc, feat, specific, mateNameRaw, angleDeg);
                return !double.IsNaN(angleDeg) && !double.IsInfinity(angleDeg);
            }
            catch (Exception ex)
            {
                error = ex.Message;
                return false;
            }
        }

        private static IFeature TryFindMateFeature(IModelDoc2 doc, string mateNameRaw)
        {
            if (doc == null || string.IsNullOrWhiteSpace(mateNameRaw)) return null;

            string mateName = mateNameRaw.Trim();
            int at = mateName.IndexOf('@');
            if (at >= 0 && at < mateName.Length - 1)
                mateName = mateName.Substring(at + 1).Trim();

            try
            {
                doc.ClearSelection2(true);
                bool selOk =
                    doc.Extension.SelectByID2(mateName, "MATE", 0, 0, 0, false, 0, null, 0) ||
                    doc.Extension.SelectByID2(mateName, "FEATURE", 0, 0, 0, false, 0, null, 0);
                if (selOk)
                {
                    var selMgr = doc.SelectionManager as ISelectionMgr;
                    return selMgr?.GetSelectedObject6(1, -1) as IFeature;
                }
            }
            catch { }
            finally
            {
                try { doc.ClearSelection2(true); } catch { }
            }

            return null;
        }

        private static double ResolveMateComplementAngle(IModelDoc2 doc, IFeature feat, object specific, string mateNameRaw, double measuredAngleDeg)
        {
            measuredAngleDeg = NormalizeAngleDeg(measuredAngleDeg);
            if (double.IsNaN(measuredAngleDeg) || double.IsInfinity(measuredAngleDeg)) return measuredAngleDeg;

            double complementDeg = NormalizeAngleDeg(180.0 - measuredAngleDeg);
            if (Math.Abs(complementDeg - measuredAngleDeg) < 1e-6) return measuredAngleDeg;

            double targetDeg = double.NaN;
            string readErr = "";
            TryReadJointAngleDeg(doc, mateNameRaw, out targetDeg, out readErr);

            double deviationDeg = double.NaN;
            if (!double.IsNaN(targetDeg) && !double.IsInfinity(targetDeg) &&
                TryGetMateMisalignmentDeg(feat, specific, out deviationDeg))
            {
                double e1 = Math.Abs(Math.Abs(targetDeg - measuredAngleDeg) - deviationDeg);
                double e2 = Math.Abs(Math.Abs(targetDeg - complementDeg) - deviationDeg);
                if (Math.Abs(e1 - e2) > 1e-6)
                    return e1 <= e2 ? measuredAngleDeg : complementDeg;
            }

            if (!double.IsNaN(targetDeg) && !double.IsInfinity(targetDeg))
            {
                double d1 = Math.Abs(targetDeg - measuredAngleDeg);
                double d2 = Math.Abs(targetDeg - complementDeg);
                if (Math.Min(d1, d2) <= 0.5 && Math.Abs(d1 - d2) > 1e-6)
                    return d1 <= d2 ? measuredAngleDeg : complementDeg;
            }

            return measuredAngleDeg;
        }

        private static bool TryGetMateMisalignmentDeg(IFeature feat, object specific, out double deviationDeg)
        {
            deviationDeg = double.NaN;
            try
            {
                IMate2 mate2 = specific as IMate2;
                if (mate2 == null)
                {
                    try { mate2 = feat?.GetSpecificFeature2() as IMate2; } catch { }
                }
                if (mate2 == null) return false;

                double deviation = mate2.GetCurrentMisalignedDeviation();
                if (double.IsNaN(deviation) || double.IsInfinity(deviation)) return false;

                deviationDeg = Math.Abs(deviation) * 180.0 / Math.PI;
                if (deviationDeg > 180.0) deviationDeg = NormalizeAngleDeg(deviationDeg);
                return !double.IsNaN(deviationDeg) && !double.IsInfinity(deviationDeg);
            }
            catch
            {
                return false;
            }
        }

        private static object[] TryGetMateReferenceObjects(object mateObj)
        {
            var refs = new List<object>();
            if (mateObj == null) return refs.ToArray();

            try
            {
                IMate2 mate2 = mateObj as IMate2;
                if (mate2 != null)
                {
                    int n = mate2.GetMateEntityCount();
                    for (int i = 0; i < n; i++)
                    {
                        object entObj = mate2.MateEntity(i);
                        object r = TryGetMateEntityReference(entObj);
                        if (r != null) refs.Add(r);
                    }
                    if (refs.Count >= 2) return refs.ToArray();
                }
            }
            catch { }

            try
            {
                dynamic d = mateObj;
                object entsObj = null;
                try { entsObj = d.EntitiesToMate; } catch { }
                if (entsObj is object[] arr)
                {
                    foreach (object ent in arr)
                    {
                        object r = TryGetMateEntityReference(ent) ?? ent;
                        if (r != null) refs.Add(r);
                    }
                    if (refs.Count >= 2) return refs.ToArray();
                }
            }
            catch { }

            return refs.ToArray();
        }

        private static object TryGetMateEntityReference(object entObj)
        {
            if (entObj == null) return null;

            try
            {
                IMateEntity2 ent2 = entObj as IMateEntity2;
                if (ent2 != null) return ent2.Reference;
            }
            catch { }

            try
            {
                dynamic d = entObj;
                object r = null;
                try { r = d.Reference; } catch { }
                if (r != null) return r;
            }
            catch { }

            return null;
        }

        private static bool TryMateUsesSupplementalAngle(IFeature feat, object specific)
        {
            try
            {
                dynamic s = specific;
                try { if ((bool)s.FlipDimension) return true; } catch { }
                try { if ((bool)s.Flipped) return true; } catch { }
            }
            catch { }

            try
            {
                object def = feat?.GetDefinition();
                dynamic d = def;
                try { if ((bool)d.FlipDimension) return true; } catch { }
                try { if ((bool)d.Flipped) return true; } catch { }
            }
            catch { }

            return false;
        }

        private static double NormalizeAngleDeg(double angleDeg)
        {
            if (double.IsNaN(angleDeg) || double.IsInfinity(angleDeg)) return double.NaN;
            angleDeg = Math.Abs(angleDeg);
            while (angleDeg >= 360.0) angleDeg -= 360.0;
            if (angleDeg > 180.0) angleDeg = 360.0 - angleDeg;
            return angleDeg;
        }

        private static IEnumerable<(string name, IDimension dim)> TryGetMateDimensionsBySelection(IModelDoc2 doc, string mateName)
        {
            var dims = new List<(string, IDimension)>();
            if (doc == null || string.IsNullOrWhiteSpace(mateName)) return dims;

            try
            {
                doc.ClearSelection2(true);

                bool selOk =
                    doc.Extension.SelectByID2(mateName, "MATE", 0, 0, 0, false, 0, null, 0) ||
                    doc.Extension.SelectByID2(mateName, "FEATURE", 0, 0, 0, false, 0, null, 0);

                if (!selOk) return dims;

                var selMgr = doc.SelectionManager as ISelectionMgr;
                object selObj = selMgr?.GetSelectedObject6(1, -1);
                IFeature feat = selObj as IFeature;

                if (feat == null) return dims;

                IDisplayDimension dd = feat.GetFirstDisplayDimension() as IDisplayDimension;
                while (dd != null)
                {
                    try
                    {
                        string ddName = dd.GetNameForSelection() as string;
                        IDimension d = dd.GetDimension2(0) as IDimension;
                        if (d != null) dims.Add((ddName ?? "", d));
                    }
                    catch { }

                    dd = feat.GetNextDisplayDimension(dd) as IDisplayDimension;
                }
            }
            catch { }
            finally
            {
                try { doc.ClearSelection2(true); } catch { }
            }

            return dims;
        }

        internal static void LogStatic(string msg)
        {
            try
            {
                var path = Path.Combine(Path.GetTempPath(), "SWAnkleInterference.log");
                File.AppendAllText(path, DateTime.Now.ToString("s") + " " + msg + System.Environment.NewLine);
            }
            catch { }
        }

        public static string GetBuildVersionTag()
        {
            try
            {
                string path = Assembly.GetExecutingAssembly().Location;
                DateTime t = File.GetLastWriteTime(path);
                return "Ver." + t.ToString("MMddHHmmss");
            }
            catch
            {
                return "Ver.unknown";
            }
        }

        public bool TryDriveMateAngle(string mateName, double angleDeg, out string message)
        {
            message = "";
            try
            {
                IModelDoc2 doc = m_SwApp?.ActiveDoc as IModelDoc2;
                IAssemblyDoc assy = doc as IAssemblyDoc;
                if (assy == null)
                {
                    message = "未打开装配体文档（.SLDASM）。";
                    return false;
                }

                bool ok = SetJointAngle(doc, assy, mateName, angleDeg);
                message = ok ? "OK" : "驱动失败（未找到可驱动的尺寸/参数，或写入失败）。";
                return ok;
            }
            catch (Exception ex)
            {
                message = "驱动异常: " + ex.Message;
                LogStatic("TryDriveMateAngle exception: " + ex);
                return false;
            }
        }

        public bool TryDriveJointAngle(int jointIndex, string mateName, double angleDeg, out string message)
        {
            message = "";
            try
            {
                IModelDoc2 doc = m_SwApp?.ActiveDoc as IModelDoc2;
                IAssemblyDoc assy = doc as IAssemblyDoc;
                if (assy == null)
                {
                    message = "No active assembly document.";
                    return false;
                }

                bool ok = SetJointAngleForJoint(doc, assy, jointIndex, mateName, angleDeg);
                message = ok ? "OK" : "Drive failed: picked mate/name was not found or could not be written.";
                return ok;
            }
            catch (Exception ex)
            {
                message = "Drive exception: " + ex.Message;
                LogStatic("TryDriveJointAngle exception: " + ex);
                return false;
            }
        }

        // ----------------------------------------------------------------
        // 干涉检测（使用 SW2024 实际 API）
        // ----------------------------------------------------------------
        private IInterferenceDetectionMgr CreateInterferenceMgr(IAssemblyDoc assy)
        {
            try
            {
                var mgr = assy?.InterferenceDetectionManager;
                if (mgr == null) return null;
                mgr.UseTransform                     = true;
                mgr.TreatCoincidenceAsInterference   = false;
                mgr.IncludeMultibodyPartInterferences = false;
                return mgr;
            }
            catch
            {
                return null;
            }
        }

        private (int count, string components) CheckInterference(IInterferenceDetectionMgr mgr)
        {
            try
            {
                if (mgr == null) return (-1, "检测异常: 干涉检测管理器为空");
                object[] results = mgr.GetInterferences() as object[];

                if (results == null || results.Length == 0)
                    return (0, "");

                var names = new List<string>();
                foreach (object r in results)
                {
                    // SW2024: IInterference (not IInterferenceData)
                    IInterference iData = r as IInterference;
                    if (iData == null) continue;

                    // Components property returns array of IComponent2
                    var compNames = new List<string>();
                    object[] comps = iData.Components as object[];
                    if (comps != null)
                    {
                        foreach (object c in comps)
                        {
                            IComponent2 comp = c as IComponent2;
                            if (comp != null) compNames.Add(comp.Name2);
                        }
                    }
                    names.Add(string.Join("<->", compNames));
                }
                return (results.Length, string.Join("; ", names));
            }
            catch (Exception ex)
            {
                return (-1, "检测异常: " + ex.Message);
            }
        }

        private (int count, string components) CheckInterference(IAssemblyDoc assy)
        {
            IInterferenceDetectionMgr mgr = null;
            try
            {
                mgr = CreateInterferenceMgr(assy);
                return CheckInterference(mgr);
            }
            finally
            {
                try { mgr?.Done(); } catch { }
            }
        }

        // ----------------------------------------------------------------
        // 平面角度测量 — 模式1：从当前选择集读取（推荐）
        // 用户在 SW 中选好两个平面后点按钮，不需要输入名称
        // ----------------------------------------------------------------
        public double MeasurePlaneAngleFromSelection()
        {
            IModelDoc2 doc = m_SwApp?.ActiveDoc as IModelDoc2;
            if (doc == null) return double.NaN;

            try
            {
                ISelectionMgr selMgr = doc.SelectionManager as ISelectionMgr;
                int count = selMgr.GetSelectedObjectCount2(-1);

                if (count != 2)
                {
                    MessageBox.Show(
                        "请确保只选中且仅选中 2 个对象（基准面或平面面）后再测量。\n" +
                        "当前选中数量: " + count,
                        "选择数量不正确", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    return double.NaN;
                }

                // 优先：使用 SolidWorks 内置 Measure（最可靠，会自动处理装配体坐标系/组件变换/不同对象类型）
                if (TryMeasureAngleUsingSwMeasure(doc, out double deg))
                    return deg;

                int t1 = selMgr.GetSelectedObjectType3(1, -1);
                int t2 = selMgr.GetSelectedObjectType3(2, -1);
                object o1 = selMgr.GetSelectedObject6(1, -1);
                object o2 = selMgr.GetSelectedObject6(2, -1);
                double[] n1 = TryGetPlaneNormalFromSelectionObject(o1, t1);
                double[] n2 = TryGetPlaneNormalFromSelectionObject(o2, t2);

                if (n1 == null || n2 == null)
                {
                    MessageBox.Show("未能提取平面法向量。\n请只选择“基准面”或“平面面”（不要选圆柱/曲面）。");
                    return double.NaN;
                }

                return AnkleInterferenceAddIn.CalcAngleDegForPanel(n1, n2);
            }
            catch (Exception ex)
            {
                MessageBox.Show("测量失败: " + ex.Message);
                return double.NaN;
            }
        }

        // ----------------------------------------------------------------
        // 平面角度测量 — 模式2：按名称查找
        // ----------------------------------------------------------------
        public double MeasurePlaneAngle(string plane1Name, string plane2Name)
        {
            IModelDoc2 doc = m_SwApp?.ActiveDoc as IModelDoc2;
            if (doc == null) return double.NaN;

            try
            {
                double[] n1 = GetRefPlaneNormal(doc, plane1Name);
                if (n1 == null)
                {
                    MessageBox.Show("找不到平面: " + plane1Name + "\n\n" +
                        "名称格式: 平面名@零件实例名@装配体名\n" +
                        "例如: 前视基准面@Part1-1@Assembly1\n\n" +
                        "建议改用[从选择测量]按钮。",
                        "错误", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return double.NaN;
                }

                double[] n2 = GetRefPlaneNormal(doc, plane2Name);
                if (n2 == null)
                {
                    MessageBox.Show("找不到平面: " + plane2Name,
                        "错误", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return double.NaN;
                }

                doc.ClearSelection2(true);
                return CalcAngleDeg(n1, n2);
            }
            catch (Exception ex)
            {
                MessageBox.Show("测量失败: " + ex.Message);
                return double.NaN;
            }
        }

        public bool CaptureLinePlaneRefsFromSelection(out string message)
        {
            message = "";
            try
            {
                IModelDoc2 doc = m_SwApp?.ActiveDoc as IModelDoc2;
                if (doc == null) { message = "未检测到当前文档。"; return false; }

                ISelectionMgr selMgr = doc.SelectionManager as ISelectionMgr;
                int count = selMgr.GetSelectedObjectCount2(-1);
                if (count != 2)
                {
                    message = "请在 SolidWorks 中只选择 2 个对象：2 个子坐标系（Coordinate System）。";
                    return false;
                }

                int t1 = selMgr.GetSelectedObjectType3(1, -1);
                int t2 = selMgr.GetSelectedObjectType3(2, -1);
                object o1 = selMgr.GetSelectedObject6(1, -1);
                object o2 = selMgr.GetSelectedObject6(2, -1);

                bool isCs1 = TryGetCoordAxesFromSelectionObject(o1, t1, out _, out _, out _);
                bool isCs2 = TryGetCoordAxesFromSelectionObject(o2, t2, out _, out _, out _);
                if (!isCs1 || !isCs2)
                {
                    message = "选择无效：请在特征树中选中 2 个“坐标系”特征（如 TARGET_ONE / TARGET_TWO），不要选原点点或其他参考几何。\n" +
                              "当前对象1=" + GetSelDebugName(o1, t1) + "，对象2=" + GetSelDebugName(o2, t2);
                    return false;
                }

                // 复用既有持久引用字段：对象1/对象2
                m_ProjLinePersist = doc.Extension.GetPersistReference3(o1) as byte[];
                m_ProjPlanePersist = doc.Extension.GetPersistReference3(o2) as byte[];
                m_ProjLineType = t1;
                m_ProjPlaneType = t2;
                m_EnableLinePlaneProjection = m_ProjLinePersist != null && m_ProjPlanePersist != null;
                if (!m_EnableLinePlaneProjection)
                {
                    message = "无法获取持久引用（Persist Reference）。";
                    return false;
                }

                message = "双坐标系参考已捕获。";
                return true;
            }
            catch (Exception ex)
            {
                message = "捕获失败: " + ex.Message;
                return false;
            }
        }

        public void EnableLinePlaneProjection(bool enabled)
        {
            m_EnableLinePlaneProjection = enabled;
        }

        public bool HasLinePlaneRefs()
        {
            return m_ProjLinePersist != null && m_ProjLinePersist.Length > 0 &&
                   m_ProjPlanePersist != null && m_ProjPlanePersist.Length > 0;
        }

        public bool CaptureScanPointFromSelection(out string message)
        {
            message = "";
            try
            {
                IModelDoc2 doc = m_SwApp?.ActiveDoc as IModelDoc2;
                if (doc == null) { message = "No active SolidWorks document."; return false; }

                ISelectionMgr selMgr = doc.SelectionManager as ISelectionMgr;
                int count = selMgr?.GetSelectedObjectCount2(-1) ?? 0;
                if (count != 1)
                {
                    message = "Select exactly one point / marker / coordinate system / component for scan tracking.";
                    return false;
                }

                int type = selMgr.GetSelectedObjectType3(1, -1);
                object obj = selMgr.GetSelectedObject6(1, -1);
                if (!TryGetPointCoordinateFromSelectionObject(obj, type, out _))
                {
                    message = "Could not read a coordinate from this selection. Try a vertex, sketch point, datum point, coordinate system, or marker component.";
                    return false;
                }

                m_ScanPointPersist = doc.Extension.GetPersistReference3(obj) as byte[];
                m_ScanPointComponentPersist = TryGetSelectionComponentPersist(doc, selMgr, 1);
                m_ScanPointType = type;
                m_ScanPointName = GetSelDebugName(obj, type);
                if (m_ScanPointPersist == null || m_ScanPointPersist.Length == 0)
                {
                    message = "Could not create a persistent reference for the selected point.";
                    return false;
                }

                message = "Scan point captured: " + m_ScanPointName;
                return true;
            }
            catch (Exception ex)
            {
                message = "Capture scan point failed: " + ex.Message;
                return false;
            }
        }

        public bool HasScanPointRef()
        {
            return m_ScanPointPersist != null && m_ScanPointPersist.Length > 0;
        }

        public bool TryMeasureScanPoint(out double xMm, out double yMm, out double zMm, out string error)
        {
            xMm = yMm = zMm = double.NaN;
            error = "";
            try
            {
                IModelDoc2 doc = m_SwApp?.ActiveDoc as IModelDoc2;
                if (doc == null) { error = "No active SolidWorks document."; return false; }
                if (!HasScanPointRef()) { error = "Scan point is not captured."; return false; }

                object pointObj = ResolvePersistObject(doc, m_ScanPointPersist);
                object compObj = ResolvePersistObject(doc, m_ScanPointComponentPersist);
                if (pointObj == null) { error = "Scan point reference is invalid."; return false; }

                if (!TryGetPointCoordinateFromSelectionObject(pointObj, m_ScanPointType, out var pt))
                {
                    error = "Could not read scan point coordinate.";
                    return false;
                }
                if (!(pointObj is IComponent2) && TryGetComponentTransform(compObj, out var tf))
                    pt = TransformPoint(pt, tf);

                xMm = pt[0] * 1000.0;
                yMm = pt[1] * 1000.0;
                zMm = pt[2] * 1000.0;
                return true;
            }
            catch (Exception ex)
            {
                error = "Measure scan point failed: " + ex.Message;
                return false;
            }
        }

        public bool CaptureScanActualAngleRefsFromSelection(int jointIndex, out string message)
        {
            message = "";
            try
            {
                IModelDoc2 doc = m_SwApp?.ActiveDoc as IModelDoc2;
                if (doc == null) { message = "No active SolidWorks document."; return false; }

                ISelectionMgr selMgr = doc.SelectionManager as ISelectionMgr;
                int count = selMgr?.GetSelectedObjectCount2(-1) ?? 0;
                if (count != 2)
                {
                    message = "Select exactly two planes/axes/edges/coordinate systems for actual angle measurement.";
                    return false;
                }

                int t1 = selMgr.GetSelectedObjectType3(1, -1);
                int t2 = selMgr.GetSelectedObjectType3(2, -1);
                object o1 = selMgr.GetSelectedObject6(1, -1);
                object o2 = selMgr.GetSelectedObject6(2, -1);
                byte[] p1 = doc.Extension.GetPersistReference3(o1) as byte[];
                byte[] p2 = doc.Extension.GetPersistReference3(o2) as byte[];
                byte[] c1 = TryGetSelectionComponentPersist(doc, selMgr, 1);
                byte[] c2 = TryGetSelectionComponentPersist(doc, selMgr, 2);
                if (p1 == null || p2 == null)
                {
                    message = "Could not create persistent references for the selected angle refs.";
                    return false;
                }

                if (jointIndex == 1)
                {
                    m_ScanJ1AnglePersist1 = p1;
                    m_ScanJ1AnglePersist2 = p2;
                    m_ScanJ1AngleComponentPersist1 = c1;
                    m_ScanJ1AngleComponentPersist2 = c2;
                    m_ScanJ1AngleType1 = t1;
                    m_ScanJ1AngleType2 = t2;
                }
                else
                {
                    m_ScanJ2AnglePersist1 = p1;
                    m_ScanJ2AnglePersist2 = p2;
                    m_ScanJ2AngleComponentPersist1 = c1;
                    m_ScanJ2AngleComponentPersist2 = c2;
                    m_ScanJ2AngleType1 = t1;
                    m_ScanJ2AngleType2 = t2;
                }

                double a = double.NaN;
                string err = "";
                TryMeasureActualAngleRefs(doc, p1, p2, c1, c2, t1, t2, out a, out err);
                message = "J" + jointIndex + " actual angle refs captured" +
                          (double.IsNaN(a) ? "." : (": " + a.ToString("F2") + " deg"));
                return true;
            }
            catch (Exception ex)
            {
                message = "Capture actual angle refs failed: " + ex.Message;
                return false;
            }
        }

        public bool HasScanActualAngleRefs(int jointIndex)
        {
            if (jointIndex == 1)
                return m_ScanJ1AnglePersist1 != null && m_ScanJ1AnglePersist2 != null;
            return m_ScanJ2AnglePersist1 != null && m_ScanJ2AnglePersist2 != null;
        }

        private bool TryMeasureScanActualAngle(int jointIndex, out double angleDeg, out string error)
        {
            angleDeg = double.NaN;
            error = "";
            try
            {
                IModelDoc2 doc = m_SwApp?.ActiveDoc as IModelDoc2;
                if (doc == null) { error = "No active SolidWorks document."; return false; }

                if (jointIndex == 1)
                    return TryMeasureActualAngleRefs(doc,
                        m_ScanJ1AnglePersist1, m_ScanJ1AnglePersist2,
                        m_ScanJ1AngleComponentPersist1, m_ScanJ1AngleComponentPersist2,
                        m_ScanJ1AngleType1, m_ScanJ1AngleType2,
                        out angleDeg, out error);

                return TryMeasureActualAngleRefs(doc,
                    m_ScanJ2AnglePersist1, m_ScanJ2AnglePersist2,
                    m_ScanJ2AngleComponentPersist1, m_ScanJ2AngleComponentPersist2,
                    m_ScanJ2AngleType1, m_ScanJ2AngleType2,
                    out angleDeg, out error);
            }
            catch (Exception ex)
            {
                error = ex.Message;
                return false;
            }
        }

        private static bool TryMeasureActualAngleRefs(
            IModelDoc2 doc,
            byte[] persist1, byte[] persist2,
            byte[] compPersist1, byte[] compPersist2,
            int type1, int type2,
            out double angleDeg, out string error)
        {
            angleDeg = double.NaN;
            error = "";
            object o1 = ResolvePersistObject(doc, persist1);
            object o2 = ResolvePersistObject(doc, persist2);
            if (o1 == null || o2 == null) { error = "angle refs invalid"; return false; }

            if (TryMeasureAngleBetweenObjects(doc, o1, o2, out angleDeg, out error))
                return true;

            if (TryGetAngleVectorFromSelectionObject(o1, type1, out var v1) &&
                TryGetAngleVectorFromSelectionObject(o2, type2, out var v2))
            {
                object c1 = ResolvePersistObject(doc, compPersist1);
                object c2 = ResolvePersistObject(doc, compPersist2);
                if (!(o1 is IComponent2) && TryGetComponentTransform(c1, out var tf1))
                    v1 = TransformVector(v1, tf1);
                if (!(o2 is IComponent2) && TryGetComponentTransform(c2, out var tf2))
                    v2 = TransformVector(v2, tf2);
                angleDeg = CalcAngleDegUnsigned(v1, v2);
                return !double.IsNaN(angleDeg) && !double.IsInfinity(angleDeg);
            }

            return false;
        }

        public bool TryMeasureLinePlaneProjection(
            out double aYZ, out double aXZ, out double aXY, out double normalDeg,
            out double dx, out double dy, out double dz,
            out double r11, out double r12, out double r13,
            out double r21, out double r22, out double r23,
            out double r31, out double r32, out double r33,
            out double pitchDeg, out double rollDeg, out double yawDeg,
            out string error)
        {
            aYZ = aXZ = aXY = normalDeg = dx = dy = dz = double.NaN;
            r11 = r12 = r13 = r21 = r22 = r23 = r31 = r32 = r33 = double.NaN;
            pitchDeg = rollDeg = yawDeg = double.NaN;
            error = "";
            if (!m_EnableLinePlaneProjection)
            {
                error = "未启用双坐标系角度计算。";
                return false;
            }

            try
            {
                IModelDoc2 doc = m_SwApp?.ActiveDoc as IModelDoc2;
                if (doc == null) { error = "未检测到当前文档。"; return false; }

                object lineObj = ResolvePersistObject(doc, m_ProjLinePersist);
                object planeObj = ResolvePersistObject(doc, m_ProjPlanePersist);
                if (lineObj == null || planeObj == null)
                {
                    error = "参考对象失效，请重新选择两个坐标系。";
                    return false;
                }

                if (!TryGetCoordAxesFromSelectionObject(lineObj, m_ProjLineType, out var x1, out var y1, out var z1) ||
                    !TryGetCoordAxesFromSelectionObject(planeObj, m_ProjPlaneType, out var x2, out var y2, out var z2))
                {
                    error = "无法解析坐标系三轴方向。";
                    return false;
                }

                dx = CalcAngleDegUnsigned(x1, x2);
                dy = CalcAngleDegUnsigned(y1, y2);
                dz = CalcAngleDegUnsigned(z1, z2);
                if (double.IsNaN(dx) || double.IsNaN(dy) || double.IsNaN(dz))
                {
                    error = "坐标轴夹角计算失败。";
                    return false;
                }
                // 旧字段置空，核心输出转为 ΔX/Y/Z（双坐标系对应轴夹角）
                aYZ = aXZ = aXY = normalDeg = double.NaN;

                BuildRelativeRotation(x1, y1, z1, x2, y2, z2, out var R);
                r11 = R[0, 0]; r12 = R[0, 1]; r13 = R[0, 2];
                r21 = R[1, 0]; r22 = R[1, 1]; r23 = R[1, 2];
                r31 = R[2, 0]; r32 = R[2, 1]; r33 = R[2, 2];
                ExtractPitchRollYawYZX(R, out pitchDeg, out rollDeg, out yawDeg);
                return true;
            }
            catch (Exception ex)
            {
                error = "测量异常: " + ex.Message;
                return false;
            }
        }

        private static bool TryGetCoordAxesFromSelectionObject(object selObj, int objType, out double[] xAxis, out double[] yAxis, out double[] zAxis)
        {
            xAxis = yAxis = zAxis = null;
            if (selObj == null) return false;
            try
            {
                // 1) 常见：坐标系选择返回 IFeature（TypeName2 == CoordSys）
                IFeature feat = selObj as IFeature;
                if (feat != null)
                {
                    string tn = "";
                    try { tn = feat.GetTypeName2() ?? ""; } catch { }
                    if (tn.Equals("CoordSys", StringComparison.OrdinalIgnoreCase) ||
                        tn.IndexOf("Coord", StringComparison.OrdinalIgnoreCase) >= 0)
                    {
                        try
                        {
                            dynamic spec = feat.GetSpecificFeature2();
                            if (TryGetCoordAxesFromDynamic(spec, out xAxis, out yAxis, out zAxis)) return true;
                        }
                        catch { }
                        try
                        {
                            dynamic def = feat.GetDefinition();
                            if (TryGetCoordAxesFromDynamic(def, out xAxis, out yAxis, out zAxis)) return true;
                        }
                        catch { }
                    }
                }

                // 2) 兜底：对选中对象本体尝试 dynamic Transform
                if (TryGetCoordAxesFromDynamic(selObj, out xAxis, out yAxis, out zAxis)) return true;
            }
            catch { }
            return false;
        }

        private static bool TryGetCoordAxesFromDynamic(object obj, out double[] xAxis, out double[] yAxis, out double[] zAxis)
        {
            xAxis = yAxis = zAxis = null;
            if (obj == null) return false;
            try
            {
                dynamic d = obj;
                dynamic tf = null;
                try { tf = d.Transform; } catch { }
                if (tf == null) { try { tf = d.GetTransform(); } catch { } }
                if (tf == null) return false;

                double[] arr = tf.ArrayData as double[];
                if (arr == null || arr.Length < 9) return false;

                // SW MathTransform 常用布局：0..2 X, 3..5 Y, 6..8 Z
                var x = Normalize3(new[] { arr[0], arr[1], arr[2] });
                var y = Normalize3(new[] { arr[3], arr[4], arr[5] });
                var z = Normalize3(new[] { arr[6], arr[7], arr[8] });
                if (x != null && y != null && z != null)
                {
                    xAxis = x; yAxis = y; zAxis = z;
                    return true;
                }
            }
            catch { }
            return false;
        }

        private static string GetSelDebugName(object obj, int type)
        {
            try
            {
                if (obj == null) return "null(type=" + type + ")";
                if (obj is IFeature f)
                {
                    string tn = "";
                    try { tn = f.GetTypeName2() ?? ""; } catch { }
                    return "IFeature(name=" + f.Name + ",typeName2=" + tn + ",selType=" + type + ")";
                }
                return obj.GetType().Name + "(selType=" + type + ")";
            }
            catch { return "unknown(selType=" + type + ")"; }
        }

        private static double Dot3(double[] a, double[] b)
        {
            if (a == null || b == null || a.Length < 3 || b.Length < 3) return double.NaN;
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        }

        // R = A^T * B, A/B 列向量分别是 X/Y/Z 轴方向
        private static void BuildRelativeRotation(double[] ax, double[] ay, double[] az, double[] bx, double[] by, double[] bz, out double[,] R)
        {
            R = new double[3, 3];
            R[0, 0] = Dot3(ax, bx); R[0, 1] = Dot3(ax, by); R[0, 2] = Dot3(ax, bz);
            R[1, 0] = Dot3(ay, bx); R[1, 1] = Dot3(ay, by); R[1, 2] = Dot3(ay, bz);
            R[2, 0] = Dot3(az, bx); R[2, 1] = Dot3(az, by); R[2, 2] = Dot3(az, bz);
        }

        // 坐标约定：前=X，右=Z，上=Y；分解顺序 R = Ry(yaw) * Rz(pitch) * Rx(roll)
        private static void ExtractPitchRollYawYZX(double[,] R, out double pitchDeg, out double rollDeg, out double yawDeg)
        {
            pitchDeg = rollDeg = yawDeg = double.NaN;
            if (R == null || R.GetLength(0) < 3 || R.GetLength(1) < 3) return;

            double Clamp(double v) => Math.Max(-1.0, Math.Min(1.0, v));
            double pitch = Math.Asin(Clamp(R[1, 0]));
            double cp = Math.Cos(pitch);
            double roll, yaw;
            if (Math.Abs(cp) > 1e-8)
            {
                roll = Math.Atan2(-R[1, 2], R[1, 1]);
                yaw = Math.Atan2(-R[2, 0], R[0, 0]);
            }
            else
            {
                roll = 0.0;
                yaw = Math.Atan2(R[0, 2], R[2, 2]);
            }

            const double r2d = 180.0 / Math.PI;
            pitchDeg = pitch * r2d;
            rollDeg = roll * r2d;
            yawDeg = yaw * r2d;
        }

        private double[] GetRefPlaneNormal(IModelDoc2 doc, string planeName)
        {
            doc.ClearSelection2(true);
            // 按名称选取基准面，类型字符串用 "PLANE"
            bool ok = doc.Extension.SelectByID2(
                planeName, "PLANE", 0, 0, 0, false, 0, null, 0);
            if (!ok) return null;

            ISelectionMgr sel = doc.SelectionManager as ISelectionMgr;
            object selObj = sel.GetSelectedObject6(1, -1);

            double[] norm = null;
            norm = TryGetPlaneNormalFromSelectionObject(selObj, (int)swSelectType_e.swSelDATUMPLANES);

            doc.ClearSelection2(true);
            return norm;
        }

        internal static double[] TryGetPlaneNormalFromSelectionObject(object selObj, int objType)
        {
            try
            {
                IRefPlane anyPlane = selObj as IRefPlane;
                if (anyPlane == null && selObj is IFeature anyFeat)
                {
                    try { anyPlane = anyFeat.GetSpecificFeature2() as IRefPlane; } catch { }
                }
                if (anyPlane != null)
                {
                    double[] pAny = anyPlane.GetRefPlaneParams() as double[];
                    if (pAny != null && pAny.Length >= 6)
                        return Normalize3(new[] { pAny[3], pAny[4], pAny[5] });
                }

                IFace2 anyFace = selObj as IFace2;
                if (anyFace != null)
                {
                    ISurface anySurf = anyFace.GetSurface() as ISurface;
                    if (anySurf != null && anySurf.IsPlane())
                    {
                        double[] anyPp = anySurf.PlaneParams as double[];
                        if (anyPp != null && anyPp.Length >= 3)
                            return Normalize3(new[] { anyPp[0], anyPp[1], anyPp[2] });
                    }
                }

                if (objType == (int)swSelectType_e.swSelDATUMPLANES)
                {
                    // 基准面：装配体里可能返回 IRefPlane 或 IFeature
                    IRefPlane plane = selObj as IRefPlane;
                    if (plane == null && selObj is IFeature feat)
                    {
                        try { plane = feat.GetSpecificFeature2() as IRefPlane; } catch { }
                    }
                    if (plane == null) return null;
                    double[] p = plane.GetRefPlaneParams() as double[];
                    if (p == null) return null;
                    // SW2024 实测：GetRefPlaneParams 固定返回 12 项
                    // [0-2 origin, 3-5 normal, 6-8 X dir, 9-11 Y dir]
                    // 因此法向量应固定取 3..5（旧代码取 9..11 会误用 Y 方向）
                    if (p.Length >= 6)
                        return Normalize3(new[] { p[3], p[4], p[5] });
                    return null;
                }

                if (objType == (int)swSelectType_e.swSelFACES)
                {
                    // 面：只接受“平面面”，曲面直接忽略
                    IFace2 face = selObj as IFace2;
                    if (face == null) return null;

                    ISurface surf = face.GetSurface() as ISurface;
                    if (surf == null || !surf.IsPlane()) return null;

                    // PlaneParams: SolidWorks 返回 6 个值
                    // [normal.x, normal.y, normal.z, rootPoint.x, rootPoint.y, rootPoint.z]
                    double[] pp = surf.PlaneParams as double[];
                    if (pp != null && pp.Length >= 3)
                        return Normalize3(new[] { pp[0], pp[1], pp[2] });

                    // 兜底：极少数情况下用 face.Normal
                    double[] n = face.Normal as double[];
                    if (n == null || n.Length < 3) return null;
                    return Normalize3(new[] { n[0], n[1], n[2] });
                }
            }
            catch { }

            return null;
        }

        internal static bool TryMeasureAngleUsingSwMeasure(IModelDoc2 doc, out double angleDeg)
        {
            angleDeg = double.NaN;
            if (doc == null) return false;
            try
            {
                // 不同 SW 版本 interop 对 Measure/IMeasure 命名略有差异：用 dynamic 保底兼容
                dynamic ext = doc.Extension;
                dynamic measure = ext.CreateMeasure();
                if (measure == null) return false;

                try { measure.ArcOption = 0; } catch { }

                bool ok = false;
                try { ok = measure.Calculate(null); } catch { ok = false; }
                if (!ok) return false;

                double rad;
                try { rad = (double)measure.Angle; }
                catch { return false; }

                angleDeg = rad * 180.0 / Math.PI;
                return !double.IsNaN(angleDeg) && !double.IsInfinity(angleDeg);
            }
            catch
            {
                return false;
            }
        }

        private static object ResolvePersistObject(IModelDoc2 doc, byte[] persist)
        {
            if (doc == null || persist == null || persist.Length == 0) return null;
            try
            {
                int err = 0;
                object obj = doc.Extension.GetObjectByPersistReference3(persist, out err);
                return err == 0 ? obj : null;
            }
            catch { return null; }
        }

        private static double[] TryGetLineDirectionFromSelectionObject(object selObj, int objType)
        {
            try
            {
                // 基准轴（Datum Axis）
                if (objType == (int)swSelectType_e.swSelDATUMAXES)
                {
                    IRefAxis axis = selObj as IRefAxis;
                    if (axis == null && selObj is IFeature feat)
                    {
                        try { axis = feat.GetSpecificFeature2() as IRefAxis; } catch { }
                    }
                    if (axis == null) return null;
                    double[] ap = axis.GetRefAxisParams() as double[];
                    if (ap != null && ap.Length >= 6)
                        return Normalize3(new[] { ap[3] - ap[0], ap[4] - ap[1], ap[5] - ap[2] });
                    if (ap != null && ap.Length >= 3)
                        return Normalize3(new[] { ap[0], ap[1], ap[2] });
                    return null;
                }

                // 直线边
                if (objType == (int)swSelectType_e.swSelEDGES)
                {
                    IEdge edge = selObj as IEdge;
                    if (edge == null) return null;
                    ICurve curve = edge.GetCurve() as ICurve;
                    if (curve == null || !curve.IsLine()) return null;
                    double[] lp = curve.LineParams as double[];
                    if (lp != null && lp.Length >= 6)
                        return Normalize3(new[] { lp[3], lp[4], lp[5] });
                }

                // 草图直线（尽量兼容）
                if (objType == (int)swSelectType_e.swSelSKETCHSEGS)
                {
                    ISketchSegment seg = selObj as ISketchSegment;
                    if (seg == null) return null;
                    if ((swSketchSegments_e)seg.GetType() != swSketchSegments_e.swSketchLINE) return null;
                    ISketchLine line = seg as ISketchLine;
                    if (line == null) return null;
                    ISketchPoint sp = line.GetStartPoint2() as ISketchPoint;
                    ISketchPoint ep = line.GetEndPoint2() as ISketchPoint;
                    if (sp == null || ep == null) return null;
                    return Normalize3(new[] { ep.X - sp.X, ep.Y - sp.Y, ep.Z - sp.Z });
                }
            }
            catch { }
            return null;
        }

        private static double[] ProjectVectorToPlane(double[] v, double[] planeNormal)
        {
            if (v == null || planeNormal == null) return null;
            double dot = v[0] * planeNormal[0] + v[1] * planeNormal[1] + v[2] * planeNormal[2];
            return new[]
            {
                v[0] - dot * planeNormal[0],
                v[1] - dot * planeNormal[1],
                v[2] - dot * planeNormal[2]
            };
        }

        private static double CalcProjectedAngleOnPlane(double[] a, double[] b, double[] planeNormal)
        {
            var ap = Normalize3(ProjectVectorToPlane(a, planeNormal));
            var bp = Normalize3(ProjectVectorToPlane(b, planeNormal));
            if (ap == null || bp == null) return double.NaN;
            return CalcAngleDegUnsigned(ap, bp);
        }

        private static double[] Normalize3(double[] v)
        {
            if (v == null || v.Length < 3) return null;
            double m = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            if (m < 1e-12) return null;
            return new[] { v[0] / m, v[1] / m, v[2] / m };
        }

        private static double[] Cross3(double[] a, double[] b)
        {
            if (a == null || b == null || a.Length < 3 || b.Length < 3) return null;
            return new[]
            {
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
            };
        }

        private static double CalcAngleDeg(double[] n1, double[] n2)
        {
            double dot  = n1[0]*n2[0] + n1[1]*n2[1] + n1[2]*n2[2];
            double mag1 = Math.Sqrt(n1[0]*n1[0] + n1[1]*n1[1] + n1[2]*n1[2]);
            double mag2 = Math.Sqrt(n2[0]*n2[0] + n2[1]*n2[1] + n2[2]*n2[2]);
            if (mag1 < 1e-10 || mag2 < 1e-10) return double.NaN;
            double cosA = dot / (mag1 * mag2);
            cosA = Math.Max(-1.0, Math.Min(1.0, cosA));
            double angleDeg = Math.Acos(cosA) * 180.0 / Math.PI;
            return angleDeg > 90.0 ? 180.0 - angleDeg : angleDeg;
        }

        // 0~180 的未折叠夹角，用于线-面投影角/法向角，避免 90 度折返
        private static double CalcAngleDegUnsigned(double[] n1, double[] n2)
        {
            double dot  = n1[0]*n2[0] + n1[1]*n2[1] + n1[2]*n2[2];
            double mag1 = Math.Sqrt(n1[0]*n1[0] + n1[1]*n1[1] + n1[2]*n1[2]);
            double mag2 = Math.Sqrt(n2[0]*n2[0] + n2[1]*n2[1] + n2[2]*n2[2]);
            if (mag1 < 1e-10 || mag2 < 1e-10) return double.NaN;
            double cosA = dot / (mag1 * mag2);
            cosA = Math.Max(-1.0, Math.Min(1.0, cosA));
            return Math.Acos(cosA) * 180.0 / Math.PI;
        }

        // Expose for AnalysisPanel without duplicating logic
        internal static double CalcAngleDegForPanel(double[] n1, double[] n2) => CalcAngleDeg(n1, n2);

        // ----------------------------------------------------------------
        // 遍历扫描（BackgroundWorker 线程内运行）
        // ----------------------------------------------------------------
        public void RunSweepAnalysis(BackgroundWorker worker, DoWorkEventArgs e)
        {
            IModelDoc2   doc  = m_SwApp?.ActiveDoc as IModelDoc2;
            IAssemblyDoc assy = doc as IAssemblyDoc;

            if (assy == null)
            {
                e.Result = "请先打开一个装配体文档。";
                return;
            }

            m_Results.Clear();
            int step = 0;

            Control uiControl = null;
            try
            {
                uiControl = Application.OpenForms.Cast<Form>().FirstOrDefault(f => f != null && !f.IsDisposed);
            }
            catch { }

            T CallOnMainThread<T>(Func<T> fn)
            {
                if (fn == null) return default(T);
                try
                {
                    if (uiControl == null || uiControl.IsDisposed) return fn();
                    if (uiControl.InvokeRequired) return (T)uiControl.Invoke(fn);
                    return fn();
                }
                catch
                {
                    return fn();
                }
            }

            // 批处理模式：SW 抑制 UI 重绘，提升速度
            try { m_SwApp.CommandInProgress = true; } catch { }
            IInterferenceDetectionMgr mgr = null;

            try
            {
                double stepC = Math.Max(0.1, StepSizeCoarse > 0 ? StepSizeCoarse : Math.Max(1.0, StepSize));
                double stepM = Math.Max(0.1, StepSizeMedium > 0 ? StepSizeMedium : Math.Max(0.5, stepC / 3.0));
                double stepF = Math.Max(0.1, StepSizeFine > 0 ? StepSizeFine : Math.Max(0.2, stepM / 3.0));
                if (stepM > stepC) stepM = stepC;
                if (stepF > stepM) stepF = stepM;

                int coarseJ1Steps = FixJoint1 ? 1 : (int)Math.Round((J1Max - J1Min) / stepC) + 1;
                int coarseJ2Steps = FixJoint2 ? 1 : (int)Math.Round((J2Max - J2Min) / stepC) + 1;
                int estimatedTotal = Math.Max(1, coarseJ1Steps * coarseJ2Steps * 3);
                int progressInterval = Math.Max(1, estimatedTotal / 200);
                m_Results.Capacity = Math.Max(m_Results.Capacity, estimatedTotal);

                double Quant(double v) => Math.Round(v, 2);
                string Key(double a, double b) => Quant(a).ToString("F2") + "|" + Quant(b).ToString("F2");
                const double actualAngleToleranceDeg = 0.25;

                bool ReadActualJointAngle(int jointIndex, string mateName, double target, out double actual, out string warn)
                {
                    actual = target;
                    warn = "";

                    if (HasPickedJointMate(jointIndex))
                    {
                        double pickedRead = double.NaN;
                        string pickedReadErr = "";
                        if (CallOnMainThread(() => TryReadJointAngleForJoint(doc, jointIndex, mateName, out pickedRead, out pickedReadErr)))
                        {
                            actual = pickedRead;
                            return true;
                        }

                        warn = "J" + jointIndex + " picked mate readback failed: " + pickedReadErr;
                        return false;
                    }

                    if (EnableFastCoordinateAngleSampling && !EnableInterferenceCheck)
                    {
                        if (HasScanActualAngleRefs(jointIndex))
                        {
                            double measured = double.NaN;
                            string measureErr = "";
                            if (CallOnMainThread(() => TryMeasureScanActualAngle(jointIndex, out measured, out measureErr)))
                            {
                                actual = measured;
                                return true;
                            }
                            warn = "J" + jointIndex + " actual refs measure failed: " + measureErr;
                        }

                        double mateMeasuredFast = double.NaN;
                        string mateMeasureFastErr = "";
                        if (CallOnMainThread(() => TryMeasureMateReferenceAngleDeg(doc, mateName, out mateMeasuredFast, out mateMeasureFastErr)))
                        {
                            actual = mateMeasuredFast;
                            return true;
                        }
                        warn = string.IsNullOrWhiteSpace(warn)
                            ? ("J" + jointIndex + " mate refs measure failed: " + mateMeasureFastErr)
                            : (warn + " | J" + jointIndex + " mate refs measure failed: " + mateMeasureFastErr);

                        double readFast = double.NaN;
                        string readFastErr = "";
                        if (CallOnMainThread(() => TryReadJointAngleForJoint(doc, jointIndex, mateName, out readFast, out readFastErr)))
                        {
                            actual = readFast;
                            return true;
                        }

                        warn = string.IsNullOrWhiteSpace(warn)
                            ? ("J" + jointIndex + " fast readback failed: " + readFastErr)
                            : (warn + " | J" + jointIndex + " fast readback failed: " + readFastErr);
                        return false;
                    }

                    double mateMeasured = double.NaN;
                    string mateMeasureErr = "";
                    if (CallOnMainThread(() => TryMeasureMateReferenceAngleDeg(doc, mateName, out mateMeasured, out mateMeasureErr)))
                    {
                        actual = mateMeasured;
                        return true;
                    }

                    warn = "J" + jointIndex + " mate refs measure failed: " + mateMeasureErr;

                    if (HasScanActualAngleRefs(jointIndex))
                    {
                        double measured = double.NaN;
                        string measureErr = "";
                        if (CallOnMainThread(() => TryMeasureScanActualAngle(jointIndex, out measured, out measureErr)))
                        {
                            actual = measured;
                            return true;
                        }

                        AppendWarn(ref warn, "J" + jointIndex + " manual refs measure failed: " + measureErr);
                    }

                    double read = double.NaN;
                    string readErr = "";
                    if (CallOnMainThread(() => TryReadJointAngleForJoint(doc, jointIndex, mateName, out read, out readErr)))
                    {
                        actual = read;
                        return true;
                    }

                    warn = string.IsNullOrWhiteSpace(warn)
                        ? ("J" + jointIndex + " readback failed: " + readErr)
                        : (warn + " | J" + jointIndex + " readback failed: " + readErr);
                    return false;
                }

                void AppendWarn(ref string warn, string part)
                {
                    if (string.IsNullOrWhiteSpace(part)) return;
                    warn = string.IsNullOrWhiteSpace(warn) ? part : (warn + " | " + part);
                }

                List<(double a, double b)> BuildFullGrid(double st)
                {
                    var pts = new List<(double, double)>();
                    int s1 = FixJoint1 ? 1 : (int)Math.Round((J1Max - J1Min) / st) + 1;
                    int s2 = FixJoint2 ? 1 : (int)Math.Round((J2Max - J2Min) / st) + 1;
                    for (int i2 = 0; i2 < s2; i2++)
                    {
                        double b = FixJoint2 ? FixedJ2Value : Quant(J2Min + i2 * st);
                        for (int i1 = 0; i1 < s1; i1++)
                        {
                            double a = FixJoint1 ? FixedJ1Value : Quant(J1Min + i1 * st);
                            pts.Add((a, b));
                        }
                    }
                    return pts;
                }

                mgr = EnableInterferenceCheck ? CallOnMainThread(() => CreateInterferenceMgr(assy)) : null;

                // 实时落盘：每完成一个网格点立刻写一行 CSV，防止中途崩溃丢数据
                m_LiveCsvPath = BuildLiveCsvPath();
                using (var live = new StreamWriter(m_LiveCsvPath, false, new System.Text.UTF8Encoding(true)))
                {
                    live.WriteLine(GetCsvHeader());
                    live.Flush();
                    int pendingLiveRows = 0;

                    void WriteLiveResult(InterferenceResult result)
                    {
                        live.WriteLine(ToCsvLine(result));
                        pendingLiveRows++;
                        if (!EnableFastCoordinateAngleSampling || EnableInterferenceCheck || pendingLiveRows >= 50)
                        {
                            live.Flush();
                            pendingLiveRows = 0;
                        }
                    }

                    var globalSeen = new HashSet<string>();
                    var stateMap = new Dictionary<string, bool>();
                    var level1 = BuildFullGrid(stepC);

                    // 在一条线段上按 childStep 取内点（不含端点），用于粗/中网格边界的 1 维细分。
                    List<double> Interior1D(double lo, double hi, double childStep)
                    {
                        var list = new List<double>();
                        if (childStep <= 0 || hi - lo <= childStep * 0.5) return list;
                        for (double t = lo + childStep; t < hi - 1e-6; t += childStep)
                            list.Add(Math.Round(t, 2));
                        return list;
                    }

                    // 沿水平边 (j1a,j2)-(j1b,j2) 或竖直边 (j1,j2a)-(j1,j2b) 扫描；从安全侧向干涉侧，首次干涉可提前停止。
                    void ScanEdge1D(bool varyJ1, double j1a, double j1b, double j2a, double j2b, double stepChild, string levelTag, bool earlyStop)
                    {
                        var pts = new List<(double j1, double j2)>();
                        if (varyJ1)
                        {
                            double j2 = j2a;
                            double lo = Math.Min(j1a, j1b);
                            double hi = Math.Max(j1a, j1b);
                            foreach (double j1 in Interior1D(lo, hi, stepChild))
                                pts.Add((j1, j2));
                        }
                        else
                        {
                            double j1 = j1a;
                            double lo = Math.Min(j2a, j2b);
                            double hi = Math.Max(j2a, j2b);
                            foreach (double j2 in Interior1D(lo, hi, stepChild))
                                pts.Add((j1, j2));
                        }

                        if (pts.Count == 0) return;

                        string kLo, kHi;
                        if (varyJ1)
                        {
                            double lo1 = Math.Min(j1a, j1b);
                            double hi1 = Math.Max(j1a, j1b);
                            kLo = Key(lo1, j2a);
                            kHi = Key(hi1, j2a);
                        }
                        else
                        {
                            double lo2 = Math.Min(j2a, j2b);
                            double hi2 = Math.Max(j2a, j2b);
                            kLo = Key(j1a, lo2);
                            kHi = Key(j1a, hi2);
                        }

                        if (!stateMap.TryGetValue(kLo, out bool sLo) || !stateMap.TryGetValue(kHi, out bool sHi))
                            return;
                        if (sLo == sHi) return;

                        // 内点顺序为沿边从低到高；若低端干涉、高端安全则反向，使扫描从安全端开始
                        if (!sLo && sHi)
                            pts.Reverse();

                        foreach (var p in pts)
                        {
                            if (worker.CancellationPending) { e.Cancel = true; return; }
                            string kk = Key(p.j1, p.j2);
                            if (!globalSeen.Add(kk)) continue;

                            bool j1ok = true, j2ok = true;
                            if (!FixJoint1) j1ok = CallOnMainThread(() => SetJointAngleForJoint(doc, assy, 1, Joint1DimName, p.j1));
                            if (!FixJoint2) j2ok = CallOnMainThread(() => SetJointAngleForJoint(doc, assy, 2, Joint2DimName, p.j2));
                            double actualJ1 = p.j1;
                            double actualJ2 = p.j2;
                            string actualWarn = "";
                            string angleWarn = "";
                            bool targetMismatch = false;
                            {
                                ReadActualJointAngle(1, Joint1DimName, p.j1, out actualJ1, out angleWarn);
                                AppendWarn(ref actualWarn, angleWarn);
                                ReadActualJointAngle(2, Joint2DimName, p.j2, out actualJ2, out angleWarn);
                                AppendWarn(ref actualWarn, angleWarn);
                                targetMismatch =
                                    Math.Abs(actualJ1 - p.j1) > actualAngleToleranceDeg ||
                                    Math.Abs(actualJ2 - p.j2) > actualAngleToleranceDeg;
                                if (targetMismatch)
                                {
                                    string miss = "Target not reached: J1 target=" + p.j1.ToString("F2") +
                                                  " actual=" + actualJ1.ToString("F2") +
                                                  "; J2 target=" + p.j2.ToString("F2") +
                                                  " actual=" + actualJ2.ToString("F2");
                                    AppendWarn(ref actualWarn, miss);
                                }
                            }
                            var (count, comps) = EnableInterferenceCheck
                                ? CallOnMainThread(() => CheckInterference(mgr))
                                : (0, "");
                            double pax = double.NaN, pay = double.NaN, paz = double.NaN, pnormal = double.NaN;
                            double dx = double.NaN, dy = double.NaN, dz = double.NaN;
                            double r11 = double.NaN, r12 = double.NaN, r13 = double.NaN;
                            double r21 = double.NaN, r22 = double.NaN, r23 = double.NaN;
                            double r31 = double.NaN, r32 = double.NaN, r33 = double.NaN;
                            double pitch = double.NaN, roll = double.NaN, yaw = double.NaN;
                            double tx = double.NaN, ty = double.NaN, tz = double.NaN;
                            string projWarn = "";
                            string pointWarn = "";
                            if (HasScanPointRef())
                            {
                                string pointErr = "";
                                if (!CallOnMainThread(() => TryMeasureScanPoint(out tx, out ty, out tz, out pointErr)))
                                    pointWarn = pointErr;
                            }
                            if (m_EnableLinePlaneProjection)
                            {
                                string perr = "";
                                if (!CallOnMainThread(() => TryMeasureLinePlaneProjection(
                                    out pax, out pay, out paz, out pnormal,
                                    out dx, out dy, out dz,
                                    out r11, out r12, out r13,
                                    out r21, out r22, out r23,
                                    out r31, out r32, out r33,
                                    out pitch, out roll, out yaw,
                                    out perr)))
                                    projWarn = perr;
                                else if (!string.IsNullOrWhiteSpace(perr))
                                    projWarn = perr;
                            }

                            step++;
                            string warn = "";
                            if (!j1ok || !j2ok) warn = "驱动失败 J1=" + j1ok + " J2=" + j2ok;
                            if (!string.IsNullOrWhiteSpace(projWarn))
                                warn = string.IsNullOrWhiteSpace(warn) ? projWarn : (warn + " | 投影: " + projWarn);

                            if (!string.IsNullOrWhiteSpace(pointWarn))
                                warn = string.IsNullOrWhiteSpace(warn) ? pointWarn : (warn + " | Point: " + pointWarn);
                            if (!string.IsNullOrWhiteSpace(actualWarn))
                                warn = string.IsNullOrWhiteSpace(warn) ? actualWarn : (warn + " | " + actualWarn);

                            bool interf = EnableInterferenceCheck ? (count > 0) : targetMismatch;
                            if (!string.IsNullOrWhiteSpace(pointWarn))
                                warn = string.IsNullOrWhiteSpace(warn) ? pointWarn : (warn + " | Point: " + pointWarn);

                            if (!string.IsNullOrWhiteSpace(pointWarn))
                                warn = string.IsNullOrWhiteSpace(warn) ? pointWarn : (warn + " | Point: " + pointWarn);
                            if (!string.IsNullOrWhiteSpace(actualWarn))
                                warn = string.IsNullOrWhiteSpace(warn) ? actualWarn : (warn + " | " + actualWarn);

                            warn = "";
                            if (!j1ok || !j2ok) warn = "椹卞姩澶辫触 J1=" + j1ok + " J2=" + j2ok;
                            AppendWarn(ref warn, projWarn);
                            AppendWarn(ref warn, pointWarn);
                            AppendWarn(ref warn, actualWarn);

                            warn = "";
                            if (!j1ok || !j2ok) warn = "椹卞姩澶辫触 J1=" + j1ok + " J2=" + j2ok;
                            AppendWarn(ref warn, projWarn);
                            AppendWarn(ref warn, pointWarn);
                            AppendWarn(ref warn, actualWarn);

                            var result = new InterferenceResult
                            {
                                Step = step,
                                Level = levelTag,
                                TargetJoint1Angle = p.j1,
                                TargetJoint2Angle = p.j2,
                                Joint1Angle = Math.Round(actualJ1, 2),
                                Joint2Angle = Math.Round(actualJ2, 2),
                                IsInterference = interf,
                                InterferenceCount = EnableInterferenceCheck ? Math.Max(0, count) : (targetMismatch ? 1 : 0),
                                ComponentNames = EnableInterferenceCheck ? (comps ?? "") : (targetMismatch ? "TargetNotReached" : ""),
                                Warning = warn,
                                ProjAngYZDeg = pax,
                                ProjAngXZDeg = pay,
                                ProjAngXYDeg = paz,
                                LinePlaneNormalDeg = pnormal,
                                DeltaX = dx,
                                DeltaY = dy,
                                DeltaZ = dz,
                                R11 = r11, R12 = r12, R13 = r13,
                                R21 = r21, R22 = r22, R23 = r23,
                                R31 = r31, R32 = r32, R33 = r33,
                                PitchDeg = pitch,
                                RollDeg = roll,
                                YawDeg = yaw,
                                TrackXmm = tx,
                                TrackYmm = ty,
                                TrackZmm = tz,
                                Timestamp = DateTime.Now
                            };
                            m_Results.Add(result);
                            stateMap[kk] = interf;
                            WriteLiveResult(result);
                            if (step % progressInterval == 0)
                                worker.ReportProgress(Math.Min(99, (int)((double)step / estimatedTotal * 100)), result);

                            if (earlyStop && interf)
                                break;
                        }
                    }

                    void RefineCoarseEdges(double childStep, string levelTag)
                    {
                        int n1 = FixJoint1 ? 1 : (int)Math.Round((J1Max - J1Min) / stepC) + 1;
                        int n2 = FixJoint2 ? 1 : (int)Math.Round((J2Max - J2Min) / stepC) + 1;
                        for (int i2 = 0; i2 < n2; i2++)
                        {
                            for (int i1 = 0; i1 < n1; i1++)
                            {
                                double j1 = FixJoint1 ? FixedJ1Value : Quant(J1Min + i1 * stepC);
                                double j2 = FixJoint2 ? FixedJ2Value : Quant(J2Min + i2 * stepC);
                                if (i1 < n1 - 1 && !FixJoint1)
                                {
                                    double j1n = Quant(J1Min + (i1 + 1) * stepC);
                                    string ka = Key(j1, j2);
                                    string kb = Key(j1n, j2);
                                    if (stateMap.TryGetValue(ka, out bool sa) && stateMap.TryGetValue(kb, out bool sb) && sa != sb)
                                        ScanEdge1D(true, j1, j1n, j2, j2, childStep, levelTag, true);
                                    if (e.Cancel) return;
                                }
                                if (i2 < n2 - 1 && !FixJoint2)
                                {
                                    double j2n = Quant(J2Min + (i2 + 1) * stepC);
                                    string ka = Key(j1, j2);
                                    string kb = Key(j1, j2n);
                                    if (stateMap.TryGetValue(ka, out bool sa) && stateMap.TryGetValue(kb, out bool sb) && sa != sb)
                                        ScanEdge1D(false, j1, j1, j2, j2n, childStep, levelTag, true);
                                    if (e.Cancel) return;
                                }
                            }
                        }
                    }

                    // 仅在中步长「规则网格」的相邻格之间找跨界边；不得遍历 stateMap 全部键（L2 会写入大量内点键，会导致 L3 爆炸到上千步）
                    void RefineMediumEdges(double childStep, string levelTag)
                    {
                        int n1m = FixJoint1 ? 1 : (int)Math.Round((J1Max - J1Min) / stepM) + 1;
                        int n2m = FixJoint2 ? 1 : (int)Math.Round((J2Max - J2Min) / stepM) + 1;
                        for (int i2 = 0; i2 < n2m; i2++)
                        {
                            for (int i1 = 0; i1 < n1m; i1++)
                            {
                                double j1 = FixJoint1 ? FixedJ1Value : Quant(J1Min + i1 * stepM);
                                double j2 = FixJoint2 ? FixedJ2Value : Quant(J2Min + i2 * stepM);
                                string ka = Key(j1, j2);
                                if (!stateMap.TryGetValue(ka, out bool sa))
                                    continue;
                                if (i1 < n1m - 1 && !FixJoint1)
                                {
                                    double j1n = Quant(J1Min + (i1 + 1) * stepM);
                                    string kb = Key(j1n, j2);
                                    if (stateMap.TryGetValue(kb, out bool sbH) && sa != sbH)
                                        ScanEdge1D(true, j1, j1n, j2, j2, childStep, levelTag, true);
                                    if (e.Cancel) return;
                                }
                                if (i2 < n2m - 1 && !FixJoint2)
                                {
                                    double j2n = Quant(J2Min + (i2 + 1) * stepM);
                                    string kb = Key(j1, j2n);
                                    if (stateMap.TryGetValue(kb, out bool sbV) && sa != sbV)
                                        ScanEdge1D(false, j1, j1, j2, j2n, childStep, levelTag, true);
                                    if (e.Cancel) return;
                                }
                            }
                        }
                    }

                    void ScanPoints(List<(double a, double b)> pts, string levelTag)
                    {
                        foreach (var p in pts)
                        {
                            if (worker.CancellationPending) { e.Cancel = true; return; }

                            double j1 = p.a;
                            double j2 = p.b;

                            bool j1ok = true;
                            bool j2ok = true;
                            if (!FixJoint1)
                                j1ok = CallOnMainThread(() => SetJointAngleForJoint(doc, assy, 1, Joint1DimName, j1));
                            if (!FixJoint2)
                                j2ok = CallOnMainThread(() => SetJointAngleForJoint(doc, assy, 2, Joint2DimName, j2));
                            double actualJ1 = j1;
                            double actualJ2 = j2;
                            string actualWarn = "";
                            string angleWarn = "";
                            bool targetMismatch = false;
                            {
                                ReadActualJointAngle(1, Joint1DimName, j1, out actualJ1, out angleWarn);
                                AppendWarn(ref actualWarn, angleWarn);
                                ReadActualJointAngle(2, Joint2DimName, j2, out actualJ2, out angleWarn);
                                AppendWarn(ref actualWarn, angleWarn);
                                targetMismatch =
                                    Math.Abs(actualJ1 - j1) > actualAngleToleranceDeg ||
                                    Math.Abs(actualJ2 - j2) > actualAngleToleranceDeg;
                                if (targetMismatch)
                                {
                                    string miss = "Target not reached: J1 target=" + j1.ToString("F2") +
                                                  " actual=" + actualJ1.ToString("F2") +
                                                  "; J2 target=" + j2.ToString("F2") +
                                                  " actual=" + actualJ2.ToString("F2");
                                    AppendWarn(ref actualWarn, miss);
                                }
                            }

                            var (count, comps) = EnableInterferenceCheck
                                ? CallOnMainThread(() => CheckInterference(mgr))
                                : (0, "");
                            double pax = double.NaN, pay = double.NaN, paz = double.NaN, pnormal = double.NaN;
                            double dx = double.NaN, dy = double.NaN, dz = double.NaN;
                            double r11 = double.NaN, r12 = double.NaN, r13 = double.NaN;
                            double r21 = double.NaN, r22 = double.NaN, r23 = double.NaN;
                            double r31 = double.NaN, r32 = double.NaN, r33 = double.NaN;
                            double pitch = double.NaN, roll = double.NaN, yaw = double.NaN;
                            double tx = double.NaN, ty = double.NaN, tz = double.NaN;
                            string projWarn = "";
                            string pointWarn = "";
                            if (HasScanPointRef())
                            {
                                string pointErr = "";
                                if (!CallOnMainThread(() => TryMeasureScanPoint(out tx, out ty, out tz, out pointErr)))
                                    pointWarn = pointErr;
                            }
                            if (m_EnableLinePlaneProjection)
                            {
                                string perr = "";
                                if (!CallOnMainThread(() => TryMeasureLinePlaneProjection(
                                    out pax, out pay, out paz, out pnormal,
                                    out dx, out dy, out dz,
                                    out r11, out r12, out r13,
                                    out r21, out r22, out r23,
                                    out r31, out r32, out r33,
                                    out pitch, out roll, out yaw,
                                    out perr)))
                                    projWarn = perr;
                                else if (!string.IsNullOrWhiteSpace(perr))
                                    projWarn = perr;
                            }

                            step++;
                            string warn = "";
                            if (!j1ok || !j2ok) warn = "驱动失败 J1=" + j1ok + " J2=" + j2ok;
                            if (!string.IsNullOrWhiteSpace(projWarn))
                                warn = string.IsNullOrWhiteSpace(warn) ? projWarn : (warn + " | 投影: " + projWarn);

                            warn = "";
                            if (!j1ok || !j2ok) warn = "Drive failed J1=" + j1ok + " J2=" + j2ok;
                            AppendWarn(ref warn, projWarn);
                            AppendWarn(ref warn, pointWarn);
                            AppendWarn(ref warn, actualWarn);

                            var result = new InterferenceResult
                            {
                                Step               = step,
                                Level              = levelTag,
                                TargetJoint1Angle  = j1,
                                TargetJoint2Angle  = j2,
                                Joint1Angle        = Math.Round(actualJ1, 2),
                                Joint2Angle        = Math.Round(actualJ2, 2),
                                IsInterference     = EnableInterferenceCheck ? (count > 0) : targetMismatch,
                                InterferenceCount  = EnableInterferenceCheck ? Math.Max(0, count) : (targetMismatch ? 1 : 0),
                                ComponentNames     = EnableInterferenceCheck ? (comps ?? "") : (targetMismatch ? "TargetNotReached" : ""),
                                Warning            = warn,
                                ProjAngYZDeg       = pax,
                                ProjAngXZDeg       = pay,
                                ProjAngXYDeg       = paz,
                                LinePlaneNormalDeg = pnormal,
                                DeltaX             = dx,
                                DeltaY             = dy,
                                DeltaZ             = dz,
                                R11                = r11,
                                R12                = r12,
                                R13                = r13,
                                R21                = r21,
                                R22                = r22,
                                R23                = r23,
                                R31                = r31,
                                R32                = r32,
                                R33                = r33,
                                PitchDeg           = pitch,
                                RollDeg            = roll,
                                YawDeg             = yaw,
                                TrackXmm           = tx,
                                TrackYmm           = ty,
                                TrackZmm           = tz,
                                Timestamp = DateTime.Now
                            };
                            m_Results.Add(result);

                            stateMap[Key(j1, j2)] = result.IsInterference;
                            globalSeen.Add(Key(j1, j2));

                            // 关键：每步立即写入并 flush
                            WriteLiveResult(result);

                            if (step % progressInterval == 0)
                            {
                                int pct = Math.Min(99, (int)((double)step / estimatedTotal * 100));
                                worker.ReportProgress(pct, result);
                            }
                        }
                    }

                    ScanPoints(level1, "L1");
                    if (e.Cancel) return;

                    if (!EnableInterferenceCheck)
                    {
                        if (m_Results.Count > 0)
                            worker.ReportProgress(100, m_Results[m_Results.Count - 1]);
                        live.Flush();
                        return;
                    }

                    // L2：仅在粗网格“边”上状态不一致时，沿该边做 1 维中步长细分（非整块矩形铺点）
                    RefineCoarseEdges(stepM, "L2");
                    if (e.Cancel) return;

                    // L3：仅在已有中步长相邻格状态不一致时，沿该边做 1 维细步长细分
                    RefineMediumEdges(stepF, "L3");

                    if (m_Results.Count > 0)
                        worker.ReportProgress(100, m_Results[m_Results.Count - 1]);

                    live.Flush();
                }
            }
            finally
            {
                try { CallOnMainThread(() => { try { mgr?.Done(); } catch { } return 0; }); } catch { }
                try { m_SwApp.CommandInProgress = false; } catch { }
            }

            e.Result = "完成";
        }

        private static List<(int i1, int i2)> BuildSpiralOrder(int j1Steps, int j2Steps)
        {
            var order = new List<(int, int)>(Math.Max(0, j1Steps * j2Steps));
            if (j1Steps <= 0 || j2Steps <= 0) return order;

            int left = 0, right = j1Steps - 1;
            int top = 0, bottom = j2Steps - 1;

            while (left <= right && top <= bottom)
            {
                // 下：左列
                for (int y = top; y <= bottom; y++) order.Add((left, y));
                left++;
                if (left > right) break;

                // 右：下行
                for (int x = left; x <= right; x++) order.Add((x, bottom));
                bottom--;
                if (top > bottom) break;

                // 上：右列
                for (int y = bottom; y >= top; y--) order.Add((right, y));
                right--;
                if (left > right) break;

                // 左：上行
                for (int x = right; x >= left; x--) order.Add((x, top));
                top++;
            }

            return order;
        }

        private static string BuildLiveCsvPath()
        {
            string dir = Path.Combine(
                System.Environment.GetFolderPath(System.Environment.SpecialFolder.MyDocuments),
                "SWAnkleInterference_Autosave");
            Directory.CreateDirectory(dir);
            return Path.Combine(dir, "LiveScan_" + DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".csv");
        }

        private string GetCsvHeader()
        {
            if (EnableFastCoordinateAngleSampling && !EnableInterferenceCheck)
                return "J1_target_deg,J2_target_deg,J1_actual_deg,J2_actual_deg,Reached,X_mm,Y_mm,Z_mm";

            return "Step,Level,J1_deg,J2_deg,Interference," +
                   "Count,ProjAngYZ_deg,ProjAngXZ_deg,ProjAngXY_deg,LinePlaneNormal_deg,DeltaX,DeltaY,DeltaZ," +
                   "R11,R12,R13,R21,R22,R23,R31,R32,R33,Pitch_deg,Roll_deg,Yaw_deg," +
                   "TrackX_mm,TrackY_mm,TrackZ_mm,Components,Warning,Timestamp";
        }

        private string ToCsvLine(InterferenceResult r)
        {
            string NumOrEmpty(double v) => (double.IsNaN(v) || double.IsInfinity(v)) ? "" : v.ToString("F6");
            if (EnableFastCoordinateAngleSampling && !EnableInterferenceCheck)
            {
                return NumOrEmpty(r.TargetJoint1Angle) + "," +
                       NumOrEmpty(r.TargetJoint2Angle) + "," +
                       NumOrEmpty(r.Joint1Angle) + "," +
                       NumOrEmpty(r.Joint2Angle) + "," +
                       (!r.IsInterference) + "," +
                       NumOrEmpty(r.TrackXmm) + "," +
                       NumOrEmpty(r.TrackYmm) + "," +
                       NumOrEmpty(r.TrackZmm);
            }

            string comps = "\"" + (r.ComponentNames ?? "").Replace("\"", "\"\"") + "\"";
            string warn  = "\"" + (r.Warning ?? "").Replace("\"", "\"\"") + "\"";
            return r.Step + "," + (r.Level ?? "") + "," + r.Joint1Angle + "," + r.Joint2Angle +
                   "," + r.IsInterference + "," + r.InterferenceCount +
                   "," + NumOrEmpty(r.ProjAngYZDeg) + "," + NumOrEmpty(r.ProjAngXZDeg) + "," + NumOrEmpty(r.ProjAngXYDeg) +
                   "," + NumOrEmpty(r.LinePlaneNormalDeg) +
                   "," + NumOrEmpty(r.DeltaX) + "," + NumOrEmpty(r.DeltaY) + "," + NumOrEmpty(r.DeltaZ) +
                   "," + NumOrEmpty(r.R11) + "," + NumOrEmpty(r.R12) + "," + NumOrEmpty(r.R13) +
                   "," + NumOrEmpty(r.R21) + "," + NumOrEmpty(r.R22) + "," + NumOrEmpty(r.R23) +
                   "," + NumOrEmpty(r.R31) + "," + NumOrEmpty(r.R32) + "," + NumOrEmpty(r.R33) +
                   "," + NumOrEmpty(r.PitchDeg) + "," + NumOrEmpty(r.RollDeg) + "," + NumOrEmpty(r.YawDeg) +
                   "," + NumOrEmpty(r.TrackXmm) + "," + NumOrEmpty(r.TrackYmm) + "," + NumOrEmpty(r.TrackZmm) +
                   "," + comps + "," + warn + "," +
                   r.Timestamp.ToString("yyyy-MM-dd HH:mm:ss");
        }

        // ----------------------------------------------------------------
        // 导出 CSV
        // ----------------------------------------------------------------
        public void ExportResults(string path = null)
        {
            if (m_Results.Count == 0)
            { MessageBox.Show("没有可导出的结果。"); return; }

            if (path == null)
            {
                using (var dlg = new SaveFileDialog
                {
                    Filter     = "CSV 文件 (*.csv)|*.csv",
                    DefaultExt = "csv",
                    FileName   = "AnkleInterference_" +
                                 DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".csv"
                })
                {
                    if (dlg.ShowDialog() != DialogResult.OK) return;
                    path = dlg.FileName;
                }
            }

            try
            {
                using (var w = new StreamWriter(path, false,
                    new System.Text.UTF8Encoding(true)))
                {
                    w.WriteLine(GetCsvHeader());
                    foreach (var r in m_Results)
                    {
                        w.WriteLine(ToCsvLine(r));
                    }
                }
                MessageBox.Show("已导出到:\n" + path, "导出完成",
                    MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception ex)
            {
                MessageBox.Show("导出失败: " + ex.Message);
            }
        }

        public List<InterferenceResult> GetResults() => m_Results;
        public string GetLiveCsvPath() => m_LiveCsvPath;

        public IModelDoc2 ActiveDocForUi() => m_SwApp?.ActiveDoc as IModelDoc2;

        public bool CaptureRealtimeTrackingSelection(out string message)
        {
            message = "";
            try
            {
                IModelDoc2 doc = m_SwApp?.ActiveDoc as IModelDoc2;
                if (doc == null) { message = "No active SolidWorks document."; return false; }

                ISelectionMgr selMgr = doc.SelectionManager as ISelectionMgr;
                int count = selMgr?.GetSelectedObjectCount2(-1) ?? 0;
                if (count != 3)
                {
                    message = "Select exactly 3 objects: one point/marker, plus two angle references.";
                    return false;
                }

                var candidates = new List<(object obj, int type, string name, byte[] persist, byte[] compPersist, double[] point, double[] angleVec)>();
                for (int i = 1; i <= count; i++)
                {
                    int type = selMgr.GetSelectedObjectType3(i, -1);
                    object obj = selMgr.GetSelectedObject6(i, -1);
                    byte[] persist = null;
                    try { persist = doc.Extension.GetPersistReference3(obj) as byte[]; } catch { }
                    byte[] compPersist = TryGetSelectionComponentPersist(doc, selMgr, i);
                    TryGetPointCoordinateFromSelectionObject(obj, type, out var pt);
                    TryGetAngleVectorFromSelectionObject(obj, type, out var av);
                    candidates.Add((obj, type, GetSelDebugName(obj, type), persist, compPersist, pt, av));
                }

                int pointIndex = candidates.FindIndex(c => c.point != null && c.persist != null);
                if (pointIndex < 0)
                {
                    message = "Could not identify a trackable point. Use a vertex, sketch point, datum point, coordinate system, or component.";
                    return false;
                }

                var angleRefs = candidates
                    .Select((c, idx) => new { c, idx })
                    .Where(x => x.idx != pointIndex && x.c.persist != null)
                    .ToList();
                if (angleRefs.Count != 2)
                {
                    message = "Could not capture two angle references. Select one point/marker plus exactly two angle references.";
                    return false;
                }

                var point = candidates[pointIndex];
                m_TrackPointPersist = point.persist;
                m_TrackPointComponentPersist = point.compPersist;
                m_TrackPointType = point.type;
                m_TrackPointName = point.name;

                m_TrackAnglePersist1 = angleRefs[0].c.persist;
                m_TrackAngleComponentPersist1 = angleRefs[0].c.compPersist;
                m_TrackAngleType1 = angleRefs[0].c.type;
                m_TrackAngleName1 = angleRefs[0].c.name;
                m_TrackAnglePersist2 = angleRefs[1].c.persist;
                m_TrackAngleComponentPersist2 = angleRefs[1].c.compPersist;
                m_TrackAngleType2 = angleRefs[1].c.type;
                m_TrackAngleName2 = angleRefs[1].c.name;

                double angle = double.NaN;
                string angleErr = "";
                TryMeasureAngleBetweenObjects(doc, angleRefs[0].c.obj, angleRefs[1].c.obj, out angle, out angleErr);
                if (double.IsNaN(angle) && angleRefs[0].c.angleVec != null && angleRefs[1].c.angleVec != null)
                    angle = CalcAngleDegUnsigned(angleRefs[0].c.angleVec, angleRefs[1].c.angleVec);
                message = "Tracking refs captured. Point=" + m_TrackPointName +
                          " | Angle refs=" + m_TrackAngleName1 + " / " + m_TrackAngleName2 +
                          " | Angle=" + (double.IsNaN(angle) ? "N/A (" + angleErr + ")" : angle.ToString("F2") + " deg");
                return true;
            }
            catch (Exception ex)
            {
                message = "Capture failed: " + ex.Message;
                return false;
            }
        }

        public bool HasRealtimeTrackingRefs()
        {
            return m_TrackPointPersist != null && m_TrackPointPersist.Length > 0 &&
                   m_TrackAnglePersist1 != null && m_TrackAnglePersist1.Length > 0 &&
                   m_TrackAnglePersist2 != null && m_TrackAnglePersist2.Length > 0;
        }

        public void ClearRealtimeTrackingSamples()
        {
            m_RealtimeTrackSamples.Clear();
        }

        public int GetRealtimeTrackingSampleCount() => m_RealtimeTrackSamples.Count;

        public bool TryAppendRealtimeTrackingSample(DateTime startTime, out RealtimeTrackSample sample, out string error)
        {
            sample = null;
            error = "";
            try
            {
                IModelDoc2 doc = m_SwApp?.ActiveDoc as IModelDoc2;
                if (doc == null) { error = "No active SolidWorks document."; return false; }
                if (!HasRealtimeTrackingRefs()) { error = "Tracking references are not captured."; return false; }

                object pointObj = ResolvePersistObject(doc, m_TrackPointPersist);
                object angleObj1 = ResolvePersistObject(doc, m_TrackAnglePersist1);
                object angleObj2 = ResolvePersistObject(doc, m_TrackAnglePersist2);
                object pointCompObj = ResolvePersistObject(doc, m_TrackPointComponentPersist);
                object angleCompObj1 = ResolvePersistObject(doc, m_TrackAngleComponentPersist1);
                object angleCompObj2 = ResolvePersistObject(doc, m_TrackAngleComponentPersist2);
                if (pointObj == null || angleObj1 == null || angleObj2 == null)
                {
                    error = "One or more tracked references are invalid. Capture them again.";
                    return false;
                }

                if (!TryGetPointCoordinateFromSelectionObject(pointObj, m_TrackPointType, out var pt))
                {
                    error = "Could not read the tracked point coordinate.";
                    return false;
                }
                if (!(pointObj is IComponent2) && TryGetComponentTransform(pointCompObj, out var pointTf))
                    pt = TransformPoint(pt, pointTf);

                string warning = "";
                double angleDeg = double.NaN;
                if (TryMeasureAngleBetweenObjects(doc, angleObj1, angleObj2, out angleDeg, out var angleErr))
                {
                    warning = "";
                }
                else if (TryGetAngleVectorFromSelectionObject(angleObj1, m_TrackAngleType1, out var v1) &&
                         TryGetAngleVectorFromSelectionObject(angleObj2, m_TrackAngleType2, out var v2))
                {
                    if (!(angleObj1 is IComponent2) && TryGetComponentTransform(angleCompObj1, out var tf1))
                        v1 = TransformVector(v1, tf1);
                    if (!(angleObj2 is IComponent2) && TryGetComponentTransform(angleCompObj2, out var tf2))
                        v2 = TransformVector(v2, tf2);
                    angleDeg = CalcAngleDegUnsigned(v1, v2);
                }
                else
                {
                    warning = "Could not read one angle reference. " + angleErr;
                }

                DateTime now = DateTime.Now;
                sample = new RealtimeTrackSample
                {
                    Index = m_RealtimeTrackSamples.Count + 1,
                    Timestamp = now,
                    ElapsedSeconds = (now - startTime).TotalSeconds,
                    Xmm = pt[0] * 1000.0,
                    Ymm = pt[1] * 1000.0,
                    Zmm = pt[2] * 1000.0,
                    AngleDeg = angleDeg,
                    Warning = warning
                };
                m_RealtimeTrackSamples.Add(sample);
                return true;
            }
            catch (Exception ex)
            {
                error = "Tracking sample failed: " + ex.Message;
                return false;
            }
        }

        public void ExportRealtimeTrackingResults(string path = null)
        {
            if (m_RealtimeTrackSamples.Count == 0)
            { MessageBox.Show("No realtime tracking data to export."); return; }

            if (path == null)
            {
                using (var dlg = new SaveFileDialog
                {
                    Filter = "CSV file (*.csv)|*.csv",
                    DefaultExt = "csv",
                    FileName = "RealtimeTrack_" + DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".csv"
                })
                {
                    if (dlg.ShowDialog() != DialogResult.OK) return;
                    path = dlg.FileName;
                }
            }

            try
            {
                using (var w = new StreamWriter(path, false, new System.Text.UTF8Encoding(true)))
                {
                    w.WriteLine("Index,Elapsed_s,X_mm,Y_mm,Z_mm,Angle_deg,Warning,Timestamp");
                    foreach (var r in m_RealtimeTrackSamples)
                    {
                        w.WriteLine(r.Index + "," +
                                    NumOrEmpty(r.ElapsedSeconds) + "," +
                                    NumOrEmpty(r.Xmm) + "," +
                                    NumOrEmpty(r.Ymm) + "," +
                                    NumOrEmpty(r.Zmm) + "," +
                                    NumOrEmpty(r.AngleDeg) + "," +
                                    CsvQuote(r.Warning) + "," +
                                    r.Timestamp.ToString("yyyy-MM-dd HH:mm:ss.fff"));
                    }
                }
                MessageBox.Show("Exported to:\n" + path, "Export complete",
                    MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception ex)
            {
                MessageBox.Show("Export failed: " + ex.Message);
            }
        }

        private static string NumOrEmpty(double v)
        {
            return (double.IsNaN(v) || double.IsInfinity(v)) ? "" : v.ToString("F6");
        }

        private static string CsvQuote(string s)
        {
            return "\"" + (s ?? "").Replace("\"", "\"\"") + "\"";
        }

        private static bool TryMeasureAngleBetweenObjects(IModelDoc2 doc, object obj1, object obj2, out double angleDeg, out string error)
        {
            angleDeg = double.NaN;
            error = "";
            if (doc == null) { error = "No active document."; return false; }
            if (obj1 == null || obj2 == null) { error = "Angle object is null."; return false; }

            try
            {
                doc.ClearSelection2(true);
                bool s1 = TrySelectObjectForMeasure(obj1, false);
                bool s2 = TrySelectObjectForMeasure(obj2, true);
                if (!s1 || !s2)
                {
                    error = "Could not select angle refs for SW Measure.";
                    try { doc.ClearSelection2(true); } catch { }
                    return false;
                }

                bool ok = TryMeasureAngleUsingSwMeasure(doc, out angleDeg);
                try { doc.ClearSelection2(true); } catch { }
                if (!ok || double.IsNaN(angleDeg) || double.IsInfinity(angleDeg))
                {
                    error = "SW Measure did not return an angle.";
                    return false;
                }

                return true;
            }
            catch (Exception ex)
            {
                error = ex.Message;
                try { doc.ClearSelection2(true); } catch { }
                return false;
            }
        }

        private static bool TrySelectObjectForMeasure(object obj, bool append)
        {
            if (obj == null) return false;
            try
            {
                dynamic d = obj;
                try { return (bool)d.Select4(append, null); } catch { }
                try { return (bool)d.Select2(append, -1); } catch { }
                try { return (bool)d.Select(append); } catch { }
                try { return (bool)d.Select(true); } catch { }
            }
            catch { }
            return false;
        }

        private static bool TryGetPointCoordinateFromSelectionObject(object selObj, int objType, out double[] point)
        {
            point = null;
            if (selObj == null) return false;

            try
            {
                if (selObj is IVertex vertex)
                {
                    object pObj = vertex.GetPoint();
                    if (TryReadPointArray(pObj, out point)) return true;
                }

                if (selObj is ISketchPoint sketchPoint)
                {
                    point = new[] { sketchPoint.X, sketchPoint.Y, sketchPoint.Z };
                    return true;
                }

                if (selObj is IEdge edge && TryGetEdgeMidpoint(edge, out point))
                    return true;

                if (selObj is ISketchSegment seg && TryGetSketchSegmentMidpoint(seg, out point))
                    return true;

                if (selObj is IComponent2 comp && TryGetTransformOrigin(comp.Transform2, out point))
                    return true;

                if (selObj is IFeature feat)
                {
                    try
                    {
                        object specific = feat.GetSpecificFeature2();
                        if (TryGetPointCoordinateFromDynamic(specific, out point)) return true;
                    }
                    catch { }

                    try
                    {
                        object def = feat.GetDefinition();
                        if (TryGetPointCoordinateFromDynamic(def, out point)) return true;
                    }
                    catch { }

                    if (TryGetPointCoordinateFromDynamic(feat, out point)) return true;
                }

                if (TryGetPointCoordinateFromDynamic(selObj, out point)) return true;
            }
            catch { }

            return false;
        }

        private static byte[] TryGetSelectionComponentPersist(IModelDoc2 doc, ISelectionMgr selMgr, int index)
        {
            if (doc == null || selMgr == null) return null;
            try
            {
                dynamic d = selMgr;
                object compObj = null;
                try { compObj = d.GetSelectedObjectsComponent4(index, -1); } catch { }
                if (compObj == null) { try { compObj = d.GetSelectedObjectsComponent3(index, -1); } catch { } }
                if (compObj == null) return null;
                return doc.Extension.GetPersistReference3(compObj) as byte[];
            }
            catch { return null; }
        }

        private static bool TryGetEdgeMidpoint(IEdge edge, out double[] point)
        {
            point = null;
            if (edge == null) return false;
            try
            {
                IVertex v1 = null;
                IVertex v2 = null;
                try { v1 = edge.GetStartVertex() as IVertex; } catch { }
                try { v2 = edge.GetEndVertex() as IVertex; } catch { }
                if (v1 == null || v2 == null) return false;

                if (!TryReadPointArray(v1.GetPoint(), out var p1)) return false;
                if (!TryReadPointArray(v2.GetPoint(), out var p2)) return false;
                point = new[]
                {
                    (p1[0] + p2[0]) / 2.0,
                    (p1[1] + p2[1]) / 2.0,
                    (p1[2] + p2[2]) / 2.0
                };
                return true;
            }
            catch { return false; }
        }

        private static bool TryGetSketchSegmentMidpoint(ISketchSegment seg, out double[] point)
        {
            point = null;
            if (seg == null) return false;
            try
            {
                if ((swSketchSegments_e)seg.GetType() != swSketchSegments_e.swSketchLINE) return false;
                ISketchLine line = seg as ISketchLine;
                if (line == null) return false;
                ISketchPoint sp = line.GetStartPoint2() as ISketchPoint;
                ISketchPoint ep = line.GetEndPoint2() as ISketchPoint;
                if (sp == null || ep == null) return false;
                point = new[]
                {
                    (sp.X + ep.X) / 2.0,
                    (sp.Y + ep.Y) / 2.0,
                    (sp.Z + ep.Z) / 2.0
                };
                return true;
            }
            catch { return false; }
        }

        private static bool TryGetComponentTransform(object compObj, out double[] transform)
        {
            transform = null;
            try
            {
                if (compObj is IComponent2 comp)
                {
                    dynamic tf = comp.Transform2;
                    transform = tf?.ArrayData as double[];
                    return transform != null && transform.Length >= 12;
                }

                dynamic d = compObj;
                dynamic tf2 = null;
                try { tf2 = d.Transform2; } catch { }
                if (tf2 == null) { try { tf2 = d.Transform; } catch { } }
                transform = tf2?.ArrayData as double[];
                return transform != null && transform.Length >= 12;
            }
            catch { return false; }
        }

        private static double[] TransformPoint(double[] p, double[] t)
        {
            if (p == null || p.Length < 3 || t == null || t.Length < 12) return p;
            return new[]
            {
                p[0] * t[0] + p[1] * t[3] + p[2] * t[6] + t[9],
                p[0] * t[1] + p[1] * t[4] + p[2] * t[7] + t[10],
                p[0] * t[2] + p[1] * t[5] + p[2] * t[8] + t[11]
            };
        }

        private static double[] TransformVector(double[] v, double[] t)
        {
            if (v == null || v.Length < 3 || t == null || t.Length < 9) return v;
            return Normalize3(new[]
            {
                v[0] * t[0] + v[1] * t[3] + v[2] * t[6],
                v[0] * t[1] + v[1] * t[4] + v[2] * t[7],
                v[0] * t[2] + v[1] * t[5] + v[2] * t[8]
            }) ?? v;
        }

        private static bool TryGetPointCoordinateFromDynamic(object obj, out double[] point)
        {
            point = null;
            if (obj == null) return false;
            try
            {
                dynamic d = obj;

                try
                {
                    object pObj = d.GetPoint();
                    if (TryReadPointArray(pObj, out point)) return true;
                }
                catch { }

                try
                {
                    object pObj = d.Point;
                    if (TryReadPointArray(pObj, out point)) return true;
                }
                catch { }

                try
                {
                    double x = (double)d.X;
                    double y = (double)d.Y;
                    double z = (double)d.Z;
                    point = new[] { x, y, z };
                    return true;
                }
                catch { }

                try
                {
                    object pObj = d.GetRefPointParams();
                    if (TryReadPointArray(pObj, out point)) return true;
                }
                catch { }

                dynamic tf = null;
                try { tf = d.Transform; } catch { }
                if (tf == null) { try { tf = d.Transform2; } catch { } }
                if (tf == null) { try { tf = d.GetTransform(); } catch { } }
                if (TryGetTransformOrigin(tf, out point)) return true;
            }
            catch { }
            return false;
        }

        private static bool TryReadPointArray(object pObj, out double[] point)
        {
            point = null;
            double[] arr = pObj as double[];
            if (arr == null || arr.Length < 3) return false;
            point = new[] { arr[0], arr[1], arr[2] };
            return true;
        }

        private static bool TryGetTransformOrigin(object transformObj, out double[] point)
        {
            point = null;
            if (transformObj == null) return false;
            try
            {
                dynamic tf = transformObj;
                double[] arr = tf.ArrayData as double[];
                if (arr != null && arr.Length >= 12)
                {
                    point = new[] { arr[9], arr[10], arr[11] };
                    return true;
                }
            }
            catch { }
            return false;
        }

        private static bool TryGetAngleVectorFromSelectionObject(object selObj, int objType, out double[] vector)
        {
            vector = null;
            if (selObj == null) return false;

            vector = TryGetPlaneNormalFromSelectionObject(selObj, objType);
            if (vector != null) return true;

            vector = TryGetLineDirectionFromSelectionObject(selObj, objType);
            if (vector != null) return true;

            if (TryGetCoordAxesFromSelectionObject(selObj, objType, out var xAxis, out var yAxis, out var zAxis))
            {
                vector = zAxis ?? xAxis ?? yAxis;
                return vector != null;
            }

            return false;
        }

        public bool PreflightCheckDrive(out string message)
        {
            message = "";
            try
            {
                IModelDoc2 doc = m_SwApp?.ActiveDoc as IModelDoc2;
                IAssemblyDoc assy = doc as IAssemblyDoc;
                if (assy == null)
                {
                    message = "请先打开一个装配体文档（.SLDASM）。";
                    return false;
                }

                double delta = Math.Max(1.0, Math.Min(5.0, StepSize));

                bool CheckOne(int jointIndex, string name, double min, double max, bool fixedMode, double fixedVal, string label, out string failMsg)
                {
                    failMsg = "";
                    if (fixedMode)
                    {
                        bool ok = SetJointAngleForJoint(doc, assy, jointIndex, name, fixedVal);
                        LogStatic($"Preflight {label}: name='{name}' fixed={fixedVal:F3} ok={ok}");
                        if (!ok) failMsg = $"{label} 驱动失败：配合名='{name}' 固定角度={fixedVal}";
                        return ok;
                    }

                    double mid = (min + max) / 2.0;
                    double a1 = mid;
                    double a2 = Math.Min(max, mid + delta);
                    if (Math.Abs(a2 - a1) < 1e-6) a2 = Math.Max(min, mid - delta);

                    bool ok1 = SetJointAngleForJoint(doc, assy, jointIndex, name, a1);
                    bool ok2 = SetJointAngleForJoint(doc, assy, jointIndex, name, a2);
                    LogStatic($"Preflight {label}: name='{name}' a1={a1:F3} ok1={ok1} a2={a2:F3} ok2={ok2}");
                    if (!ok1 || !ok2)
                        failMsg = $"{label} 驱动失败：配合名='{name}' 试驱角度={a1:F2}/{a2:F2} (ok={ok1}/{ok2})";
                    return ok1 && ok2;
                }

                if (!CheckOne(1, Joint1DimName, J1Min, J1Max, FixJoint1, FixedJ1Value, "关节1", out var msg1))
                { message = msg1; return false; }
                if (!CheckOne(2, Joint2DimName, J2Min, J2Max, FixJoint2, FixedJ2Value, "关节2", out var msg2))
                { message = msg2; return false; }

                message = "OK";
                return true;
            }
            catch (Exception ex)
            {
                message = "自检异常: " + ex.Message;
                LogStatic("Preflight exception: " + ex);
                return false;
            }
        }
    }

    // =====================================================================
    // 进度窗体
    // =====================================================================
    public class ProgressForm : Form
    {
        private ProgressBar m_Bar;
        private Label       m_StatusLbl, m_DetailLbl;
        private Button      m_CancelBtn;
        private Button      m_TopMostBtn;
        private RichTextBox m_Log;

        public event EventHandler CancelRequested;

        public ProgressForm()
        {
            Text            = "干涉分析进度  " + AnkleInterferenceAddIn.GetBuildVersionTag();
            // Auto-fit to screen, allow resize
            AutoScaleMode   = AutoScaleMode.Font;
            FormBorderStyle = FormBorderStyle.Sizable;
            StartPosition   = FormStartPosition.CenterScreen;
            MaximizeBox = MinimizeBox = false;
            ControlBox  = false;

            m_StatusLbl = new Label
            {
                Text = "正在初始化...",
                Location = new Point(12, 12), Size = new Size(480, 20),
                Font = new Font("微软雅黑", 9, FontStyle.Bold)
            };
            Controls.Add(m_StatusLbl);

            m_Bar = new ProgressBar
            {
                Location = new Point(12, 38), Size = new Size(480, 22),
                Style = ProgressBarStyle.Continuous, Minimum = 0, Maximum = 100
            };
            Controls.Add(m_Bar);

            m_DetailLbl = new Label
            {
                Location = new Point(12, 66), Size = new Size(480, 20),
                ForeColor = Color.Gray
            };
            Controls.Add(m_DetailLbl);

            m_Log = new RichTextBox
            {
                Location = new Point(12, 92), Size = new Size(480, 180),
                ReadOnly = true, BackColor = Color.Black,
                ForeColor = Color.LimeGreen,
                Font = new Font("Consolas", 8),
                ScrollBars = RichTextBoxScrollBars.Vertical
            };
            Controls.Add(m_Log);

            m_CancelBtn = new Button
            {
                Text = "取消", Location = new Point(215, 278), Size = new Size(90, 30)
            };
            m_CancelBtn.Click += (s, ev) =>
            {
                m_CancelBtn.Enabled = false;
                m_StatusLbl.Text    = "正在取消...";
                CancelRequested?.Invoke(this, EventArgs.Empty);
            };
            Controls.Add(m_CancelBtn);

            m_TopMostBtn = new Button
            {
                Text = "置顶", Location = new Point(320, 278), Size = new Size(90, 30),
                FlatStyle = FlatStyle.Flat
            };
            m_TopMostBtn.Click += (s, ev) =>
            {
                TopMost = !TopMost;
                m_TopMostBtn.Text = TopMost ? "置顶:开" : "置顶";
            };
            Controls.Add(m_TopMostBtn);

            Shown += (s, e) =>
            {
                try
                {
                    var wa = Screen.FromControl(this).WorkingArea;
                    Width  = Math.Min(wa.Width - 40, 820);
                    Height = Math.Min(wa.Height - 80, 520);
                    MinimumSize = new Size(520, 340);
                    // Make main widgets stretch with window
                    m_Bar.Anchor       = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;
                    m_StatusLbl.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;
                    m_DetailLbl.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;
                    m_Log.Anchor       = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right;
                    m_CancelBtn.Anchor = AnchorStyles.Bottom;
                    m_TopMostBtn.Anchor = AnchorStyles.Bottom;
                }
                catch { }
            };
        }

        public void Update(int percent, InterferenceResult r)
        {
            if (IsDisposed) return;
            m_Bar.Value      = Math.Min(100, percent);
            m_StatusLbl.Text = "进度: " + percent + "%   步骤 " + r.Step;
            m_DetailLbl.Text = "J1=" + r.Joint1Angle.ToString("F1") + "  " +
                               "J2=" + r.Joint2Angle.ToString("F1") + "  ->  " +
                               (r.IsInterference ? "干涉" : "正常");

            string line = "[" + r.Step.ToString("D4") + "] " +
                          "J1=" + r.Joint1Angle.ToString("F1").PadLeft(6) + "  " +
                          "J2=" + r.Joint2Angle.ToString("F1").PadLeft(6) + "  " +
                          (r.IsInterference
                              ? "干涉(" + r.InterferenceCount + "处)"
                              : "正常");
            if (!string.IsNullOrEmpty(r.Warning))
                line += "  [" + r.Warning + "]";

            m_Log.SelectionColor = r.IsInterference ? Color.OrangeRed : Color.LimeGreen;
            m_Log.AppendText(line + "\n");
            m_Log.ScrollToCaret();
        }

        public void SetFinished(string summary)
        {
            if (IsDisposed) return;
            m_Bar.Value      = 100;
            m_StatusLbl.Text = "分析完成";
            m_DetailLbl.Text = summary;
            m_CancelBtn.Text    = "关闭";
            m_CancelBtn.Enabled = true;
            m_CancelBtn.Click  += (s, ev) => Close();
        }
    }

    // =====================================================================
    // 分析面板
    // =====================================================================
    public class AnalysisPanel : Form
    {
        private readonly AnkleInterferenceAddIn m_Addin;
        private BackgroundWorker m_Worker;
        private ProgressForm     m_Progress;

        // 关节配置
        private TextBox       txJ1Dim, txJ2Dim;
        private NumericUpDown numJ1Min, numJ1Max, numJ2Min, numJ2Max, numStepCoarse, numStepMedium, numStepFine;
        private CheckBox      ckFixJ1, ckFixJ2;
        private NumericUpDown numFixJ1, numFixJ2;
        private CheckBox      ckInterference;
        private CheckBox      ckFastCoordinateCsv;
        private Button        btnPickScanPoint;
        private Label         lblScanPoint;
        private Button        btnPickJ1Mate, btnPickJ2Mate;
        private Label         lblPickedJ1Mate, lblPickedJ2Mate;
        private Button        btnPickJ1ActualRefs, btnPickJ2ActualRefs;
        private Label         lblJ1ActualRefs, lblJ2ActualRefs;

        // 单关节驱动测试（不记录任何扫描结果）
        private ComboBox      cbTestJoint;
        private NumericUpDown numTestTarget;
        private Button        btnPickRefs, btnTestDrive;
        private Label         lblRefAngle;
        private Label         lblRefNames;
        // 不持久保存 COM 对象，改用 Persist Reference（避免选择集失效/悬空引用）
        private byte[]        m_RefPersist1, m_RefPersist2;
        private int           m_RefType1, m_RefType2;

        // 平面角度
        private TextBox txPlane1, txPlane2;
        private Label   lblAngleResult;
        private CheckBox ckProjEnable;
        private Button   btnPickLinePlane;
        private Label    lblProjResult;

        // 状态
        private Label  lblStatus;
        private Button btnRun, btnExport, btnStop, btnTopMost;
        private Button btnTrackCapture, btnTrackStart, btnTrackStop, btnTrackExport;
        private Label lblTrackStatus, lblTrackLive;
        private NumericUpDown numTrackInterval;
        private Timer m_TrackTimer;
        private DateTime m_TrackStartTime;

        public AnalysisPanel(AnkleInterferenceAddIn addin)
        {
            m_Addin = addin;
            BuildUI();
        }

        protected override void OnFormClosed(FormClosedEventArgs e)
        {
            try
            {
                m_TrackTimer?.Stop();
                m_TrackTimer?.Dispose();
                m_TrackTimer = null;
            }
            catch { }
            base.OnFormClosed(e);
        }

        private void BuildUI()
        {
            Text            = "脚踝干涉分析工具  " + AnkleInterferenceAddIn.GetBuildVersionTag();
            AutoScaleMode   = AutoScaleMode.Font;
            FormBorderStyle = FormBorderStyle.Sizable;
            StartPosition   = FormStartPosition.CenterScreen;
            AutoScroll      = true;

            Shown += (s, e) =>
            {
                try
                {
                    var wa = Screen.FromControl(this).WorkingArea;
                    Width  = Math.Min(wa.Width - 40, 560);
                    Height = Math.Min(wa.Height - 80, 860);
                    MinimumSize = new Size(440, 660);
                }
                catch { }
            };

            int y = 10;

            // ── 单关节驱动测试（强制验证关节是否能动）───────────────────
            Lbl("单关节驱动测试（不记录）", 10, ref y, true);

            Controls.Add(new Label { Text = "测试关节:", Location = new Point(10, y), AutoSize = true });
            cbTestJoint = new ComboBox
            {
                Location = new Point(80, y - 3), Width = 150, DropDownStyle = ComboBoxStyle.DropDownList
            };
            cbTestJoint.Items.Add("关节1（使用关节1配合名）");
            cbTestJoint.Items.Add("关节2（使用关节2配合名）");
            cbTestJoint.SelectedIndex = 0;
            Controls.Add(cbTestJoint);

            btnPickRefs = new Button
            {
                Text = "选两面作参考角",
                Location = new Point(250, y - 4),
                Size = new Size(170, 28)
            };
            btnPickRefs.Click += (s, e) =>
            {
                if (!TryCaptureTwoRefs(out var err))
                {
                    MessageBox.Show(err, "选择失败", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }
                double a = MeasureRefAngle();
                lblRefAngle.Text = double.IsNaN(a) ? "参考角: (无效)" : "参考角: " + a.ToString("F2") + "°";
            };
            Controls.Add(btnPickRefs);
            y += 26;

            lblRefAngle = new Label
            {
                Text = "参考角: 未设置（请先在 SW 里选中两面/两基准面）",
                Location = new Point(10, y),
                Size = new Size(410, 18),
                ForeColor = Color.DimGray,
                Font = new Font("微软雅黑", 7.5f)
            };
            Controls.Add(lblRefAngle);
            y += 22;

            lblRefNames = new Label
            {
                Text = "参考对象: -",
                Location = new Point(10, y),
                Size = new Size(410, 18),
                ForeColor = Color.DimGray,
                Font = new Font("微软雅黑", 7.5f)
            };
            Controls.Add(lblRefNames);
            y += 20;

            Lbl("目标角度(度):", 10, ref y);
            numTestTarget = Num(-360, 360, 0, 160, y - 22);

            btnTestDrive = new Button
            {
                Text = "测试驱动（不记录）",
                Location = new Point(250, y - 24),
                Size = new Size(170, 30),
                BackColor = Color.FromArgb(90, 90, 90),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
            };
            btnTestDrive.Click += (s, e) => RunSingleJointDriveTest();
            Controls.Add(btnTestDrive);
            y += 10;

            Controls.Add(new Label
            {
                Text = "────────────────────────────────────",
                Location = new Point(10, y), AutoSize = true, ForeColor = Color.LightGray
            });
            y += 18;

            // ── 关节配置 ──────────────────────────────────────────────
            Lbl("关节配置", 10, ref y, true);

            Lbl("关节1 配合名称:", 10, ref y);
            txJ1Dim = Txt(m_Addin.Joint1DimName, 160, y - 22, 240);

            Lbl("关节2 配合名称:", 10, ref y);
            txJ2Dim = Txt(m_Addin.Joint2DimName, 160, y - 22, 240);

            btnPickJ1Mate = new Button
            {
                Text = "Pick J1 mate",
                Location = new Point(10, y),
                Size = new Size(120, 28)
            };
            btnPickJ1Mate.Click += (s, e) =>
            {
                if (!m_Addin.CaptureJointMateFromSelection(1, out var msg))
                {
                    MessageBox.Show(msg, "Pick J1 mate failed", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }
                txJ1Dim.Text = m_Addin.Joint1DimName;
                lblPickedJ1Mate.Text = msg;
                lblPickedJ1Mate.ForeColor = Color.DarkGreen;
            };
            Controls.Add(btnPickJ1Mate);

            lblPickedJ1Mate = new Label
            {
                Text = "Optional: select one angle mate, then click Pick J1 mate.",
                Location = new Point(140, y + 4),
                Size = new Size(360, 24),
                ForeColor = Color.DimGray,
                Font = new Font("Microsoft YaHei", 7.5f)
            };
            Controls.Add(lblPickedJ1Mate);
            y += 32;

            btnPickJ2Mate = new Button
            {
                Text = "Pick J2 mate",
                Location = new Point(10, y),
                Size = new Size(120, 28)
            };
            btnPickJ2Mate.Click += (s, e) =>
            {
                if (!m_Addin.CaptureJointMateFromSelection(2, out var msg))
                {
                    MessageBox.Show(msg, "Pick J2 mate failed", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }
                txJ2Dim.Text = m_Addin.Joint2DimName;
                lblPickedJ2Mate.Text = msg;
                lblPickedJ2Mate.ForeColor = Color.DarkGreen;
            };
            Controls.Add(btnPickJ2Mate);

            lblPickedJ2Mate = new Label
            {
                Text = "Optional: select one angle mate, then click Pick J2 mate.",
                Location = new Point(140, y + 4),
                Size = new Size(360, 24),
                ForeColor = Color.DimGray,
                Font = new Font("Microsoft YaHei", 7.5f)
            };
            Controls.Add(lblPickedJ2Mate);
            y += 32;

            // 提示标签
            var hint = new Label
            {
                Text = "配合名称请直接填写特征树中看到的名字，例如 LimitAngle1/LimitAngle2（兼容 D1@配合名 格式）",
                Location = new Point(10, y), Size = new Size(400, 16),
                ForeColor = Color.Gray, Font = new Font("微软雅黑", 7.5f)
            };
            Controls.Add(hint); y += 20;

            // 关节1 范围
            ckFixJ1 = new CheckBox { Text = "固定关节1",
                Location = new Point(10, y), AutoSize = true };
            ckFixJ1.CheckedChanged += (s, e) => {
                numJ1Min.Enabled = numJ1Max.Enabled = !ckFixJ1.Checked;
                numFixJ1.Enabled = ckFixJ1.Checked;
            };
            Controls.Add(ckFixJ1); y += 26;

            Lbl("关节1范围 (度):", 10, ref y);
            numJ1Min = Num(-360, 360, (decimal)m_Addin.J1Min, 160, y - 22);
            Controls.Add(new Label { Text = "~",
                Location = new Point(242, y - 17), AutoSize = true });
            numJ1Max = Num(-360, 360, (decimal)m_Addin.J1Max, 258, y - 22);

            Lbl("固定关节1值 (度):", 10, ref y);
            numFixJ1 = Num(-360, 360, 0, 160, y - 22);
            numFixJ1.Enabled = false;

            // 关节2 范围
            ckFixJ2 = new CheckBox { Text = "固定关节2",
                Location = new Point(10, y), AutoSize = true };
            ckFixJ2.CheckedChanged += (s, e) => {
                numJ2Min.Enabled = numJ2Max.Enabled = !ckFixJ2.Checked;
                numFixJ2.Enabled = ckFixJ2.Checked;
            };
            Controls.Add(ckFixJ2); y += 26;

            Lbl("关节2范围 (度):", 10, ref y);
            numJ2Min = Num(-360, 360, (decimal)m_Addin.J2Min, 160, y - 22);
            Controls.Add(new Label { Text = "~",
                Location = new Point(242, y - 17), AutoSize = true });
            numJ2Max = Num(-360, 360, (decimal)m_Addin.J2Max, 258, y - 22);

            Lbl("固定关节2值 (度):", 10, ref y);
            numFixJ2 = Num(-360, 360, 0, 160, y - 22);
            numFixJ2.Enabled = false;

            Lbl("粗步长 (度):", 10, ref y);
            numStepCoarse = Num(0.01m, 90m, (decimal)m_Addin.StepSizeCoarse, 160, y - 22);
            Lbl("中步长 (度):", 10, ref y);
            numStepMedium = Num(0.01m, 90m, (decimal)m_Addin.StepSizeMedium, 160, y - 22);
            Lbl("细步长 (度):", 10, ref y);
            numStepFine = Num(0.01m, 90m, (decimal)m_Addin.StepSizeFine, 160, y - 22);

            y += 10;

            // ── 操作按钮 ─────────────────────────────────────────────
            ckInterference = new CheckBox
            {
                Text = "Calculate interference during scan",
                Location = new Point(10, y),
                AutoSize = true,
                Checked = m_Addin.EnableInterferenceCheck
            };
            Controls.Add(ckInterference);
            y += 26;

            ckFastCoordinateCsv = new CheckBox
            {
                Text = "Fast CSV: target/actual angles + reached + point XYZ",
                Location = new Point(10, y),
                AutoSize = true,
                Checked = m_Addin.EnableFastCoordinateAngleSampling
            };
            Controls.Add(ckFastCoordinateCsv);
            y += 26;

            btnPickScanPoint = new Button
            {
                Text = "Pick scan point",
                Location = new Point(10, y - 2),
                Size = new Size(130, 28)
            };
            btnPickScanPoint.Click += (s, e) =>
            {
                if (!m_Addin.CaptureScanPointFromSelection(out var msg))
                {
                    MessageBox.Show(msg, "Scan point failed", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }
                lblScanPoint.Text = msg;
                lblScanPoint.ForeColor = Color.DarkGreen;
            };
            Controls.Add(btnPickScanPoint);

            lblScanPoint = new Label
            {
                Text = "Optional: select one point/marker, then click Pick scan point.",
                Location = new Point(150, y + 3),
                Size = new Size(360, 34),
                ForeColor = Color.DimGray,
                Font = new Font("Microsoft YaHei", 7.5f)
            };
            Controls.Add(lblScanPoint);
            y += 40;

            btnPickJ1ActualRefs = new Button
            {
                Text = "Pick J1 actual refs",
                Location = new Point(10, y - 2),
                Size = new Size(150, 28)
            };
            btnPickJ1ActualRefs.Click += (s, e) =>
            {
                if (!m_Addin.CaptureScanActualAngleRefsFromSelection(1, out var msg))
                {
                    MessageBox.Show(msg, "J1 actual angle refs failed", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }
                lblJ1ActualRefs.Text = msg;
                lblJ1ActualRefs.ForeColor = Color.DarkGreen;
            };
            Controls.Add(btnPickJ1ActualRefs);

            lblJ1ActualRefs = new Label
            {
                Text = "Optional for full scan only: select 2 refs for actual J1.",
                Location = new Point(170, y + 3),
                Size = new Size(340, 28),
                ForeColor = Color.DimGray,
                Font = new Font("Microsoft YaHei", 7.5f)
            };
            Controls.Add(lblJ1ActualRefs);
            y += 34;

            btnPickJ2ActualRefs = new Button
            {
                Text = "Pick J2 actual refs",
                Location = new Point(10, y - 2),
                Size = new Size(150, 28)
            };
            btnPickJ2ActualRefs.Click += (s, e) =>
            {
                if (!m_Addin.CaptureScanActualAngleRefsFromSelection(2, out var msg))
                {
                    MessageBox.Show(msg, "J2 actual angle refs failed", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }
                lblJ2ActualRefs.Text = msg;
                lblJ2ActualRefs.ForeColor = Color.DarkGreen;
            };
            Controls.Add(btnPickJ2ActualRefs);

            lblJ2ActualRefs = new Label
            {
                Text = "Optional for full scan only: select 2 refs for actual J2.",
                Location = new Point(170, y + 3),
                Size = new Size(340, 28),
                ForeColor = Color.DimGray,
                Font = new Font("Microsoft YaHei", 7.5f)
            };
            Controls.Add(lblJ2ActualRefs);
            y += 38;

            btnRun = new Button
            {
                Text = "开始扫描", Location = new Point(10, y), Size = new Size(120, 34),
                BackColor = Color.FromArgb(0, 120, 215), ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
            };
            btnRun.Click += OnRunClick;
            Controls.Add(btnRun);

            btnExport = new Button
            {
                Text = "导出 CSV", Location = new Point(140, y),
                Size = new Size(100, 34), Enabled = false
            };
            btnExport.Click += (s, e) => m_Addin.ExportResults();
            Controls.Add(btnExport);

            btnStop = new Button
            {
                Text = "停止", Location = new Point(250, y),
                Size = new Size(90, 34), Enabled = false,
                BackColor = Color.FromArgb(200, 60, 60), ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
            };
            btnStop.Click += (s, e) =>
            {
                try
                {
                    if (m_Worker != null && m_Worker.IsBusy)
                    {
                        btnStop.Enabled = false;
                        lblStatus.Text = "正在停止...";
                        lblStatus.ForeColor = Color.Gray;
                        m_Worker.CancelAsync();
                    }
                }
                catch { }
            };
            Controls.Add(btnStop);

            btnTopMost = new Button
            {
                Text = "置顶", Location = new Point(350, y),
                Size = new Size(80, 34),
                FlatStyle = FlatStyle.Flat
            };
            btnTopMost.Click += (s, e) =>
            {
                TopMost = !TopMost;
                btnTopMost.Text = TopMost ? "置顶:开" : "置顶";
            };
            Controls.Add(btnTopMost);
            y += 44;

            // ── 平面角度测量 ─────────────────────────────────────────
            var sep = new Label
            {
                Text = "────── 平面角度测量 ──────",
                Location = new Point(10, y), AutoSize = true, ForeColor = Color.Gray
            };
            Controls.Add(sep); y += 22;

            // 推荐：从选择集测量
            var btnFromSel = new Button
            {
                Text = "从当前选择测量（推荐）",
                Location = new Point(10, y), Size = new Size(200, 30),
                BackColor = Color.FromArgb(0, 140, 80), ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
            };
            btnFromSel.Click += (s, e) => {
                double a = m_Addin.MeasurePlaneAngleFromSelection();
                lblAngleResult.Text = double.IsNaN(a) ? "测量失败"
                    : "夹角 = " + a.ToString("F2") + " 度";
            };
            Controls.Add(btnFromSel);
            y += 34;

            var hintSel = new Label
            {
                Text = "先在 SW 中选中两个平面，再点上方按钮",
                Location = new Point(10, y), AutoSize = true,
                ForeColor = Color.DimGray, Font = new Font("微软雅黑", 7.5f)
            };
            Controls.Add(hintSel); y += 20;

            // 按名称测量
            Lbl("平面1名称:", 10, ref y);
            txPlane1 = Txt("前视基准面", 160, y - 22, 220);

            Lbl("平面2名称:", 10, ref y);
            txPlane2 = Txt("上视基准面", 160, y - 22, 220);

            var btnByName = new Button
            {
                Text = "按名称测量", Location = new Point(10, y), Size = new Size(110, 28)
            };
            btnByName.Click += (s, e) => {
                double a = m_Addin.MeasurePlaneAngle(txPlane1.Text, txPlane2.Text);
                lblAngleResult.Text = double.IsNaN(a) ? "测量失败"
                    : "夹角 = " + a.ToString("F2") + " 度";
            };
            Controls.Add(btnByName);

            lblAngleResult = new Label
            {
                Text = "—", Location = new Point(130, y + 5), AutoSize = true,
                Font = new Font("微软雅黑", 11, FontStyle.Bold), ForeColor = Color.DarkBlue
            };
            Controls.Add(lblAngleResult);
            y += 38;

            // ── 线-面投影角度 / ΔXΔYΔZ ─────────────────────────────
            var sep2 = new Label
            {
                Text = "────── 线-面投影参数 ──────",
                Location = new Point(10, y), AutoSize = true, ForeColor = Color.Gray
            };
            Controls.Add(sep2); y += 22;

            ckProjEnable = new CheckBox
            {
                Text = "启用双坐标系 ΔX/ΔY/ΔZ 并写入 CSV",
                Location = new Point(10, y), AutoSize = true
            };
            ckProjEnable.CheckedChanged += (s, e) => m_Addin.EnableLinePlaneProjection(ckProjEnable.Checked);
            Controls.Add(ckProjEnable); y += 26;

            btnPickLinePlane = new Button
            {
                Text = "选 2 个子坐标系",
                Location = new Point(10, y - 2), Size = new Size(170, 28)
            };
            btnPickLinePlane.Click += (s, e) =>
            {
                if (!m_Addin.CaptureLinePlaneRefsFromSelection(out var msg))
                {
                    MessageBox.Show(msg, "选择失败", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }

                if (m_Addin.TryMeasureLinePlaneProjection(
                    out double ax, out double ay, out double az, out double an,
                    out double dx, out double dy, out double dz,
                    out double r11, out double r12, out double r13,
                    out double r21, out double r22, out double r23,
                    out double r31, out double r32, out double r33,
                    out double pitch, out double roll, out double yaw,
                    out var err))
                {
                    string warn = string.IsNullOrWhiteSpace(err) ? "" : ("  |  " + err);
                    lblProjResult.Text = $"ΔX/Y/Z={dx:F2}/{dy:F2}/{dz:F2}°  Pitch/Roll={pitch:F2}/{roll:F2}°{warn}";
                }
                else
                {
                    lblProjResult.Text = "双坐标系测量失败: " + err;
                }
            };
            Controls.Add(btnPickLinePlane);
            y += 32;

            lblProjResult = new Label
            {
                Text = "未设置双坐标系参考（先在 SW 里选中 2 个子坐标系）",
                Location = new Point(10, y),
                Size = new Size(520, 36),
                ForeColor = Color.DimGray,
                Font = new Font("微软雅黑", 7.5f)
            };
            Controls.Add(lblProjResult);
            y += 40;

            var sepTrack = new Label
            {
                Text = "------ Realtime point + angle tracking ------",
                Location = new Point(10, y), AutoSize = true, ForeColor = Color.Gray
            };
            Controls.Add(sepTrack); y += 22;

            btnTrackCapture = new Button
            {
                Text = "Confirm selection",
                Location = new Point(10, y),
                Size = new Size(130, 30)
            };
            btnTrackCapture.Click += (s, e) =>
            {
                if (!m_Addin.CaptureRealtimeTrackingSelection(out var msg))
                {
                    MessageBox.Show(msg, "Selection failed", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }
                lblTrackStatus.Text = msg;
                lblTrackStatus.ForeColor = Color.DarkGreen;
                btnTrackStart.Enabled = true;
                btnTrackExport.Enabled = m_Addin.GetRealtimeTrackingSampleCount() > 0;
            };
            Controls.Add(btnTrackCapture);

            Controls.Add(new Label
            {
                Text = "Interval(ms):",
                Location = new Point(150, y + 6),
                AutoSize = true
            });
            numTrackInterval = new NumericUpDown
            {
                Minimum = 50,
                Maximum = 5000,
                Value = 200,
                Increment = 50,
                Location = new Point(230, y + 3),
                Width = 80
            };
            Controls.Add(numTrackInterval);
            y += 36;

            btnTrackStart = new Button
            {
                Text = "Start tracking",
                Location = new Point(10, y),
                Size = new Size(120, 32),
                Enabled = false,
                BackColor = Color.FromArgb(0, 140, 80),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
            };
            btnTrackStart.Click += (s, e) => StartRealtimeTracking();
            Controls.Add(btnTrackStart);

            btnTrackStop = new Button
            {
                Text = "Stop",
                Location = new Point(140, y),
                Size = new Size(80, 32),
                Enabled = false,
                BackColor = Color.FromArgb(200, 60, 60),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat
            };
            btnTrackStop.Click += (s, e) => StopRealtimeTracking();
            Controls.Add(btnTrackStop);

            btnTrackExport = new Button
            {
                Text = "Export track CSV",
                Location = new Point(230, y),
                Size = new Size(130, 32),
                Enabled = false
            };
            btnTrackExport.Click += (s, e) => m_Addin.ExportRealtimeTrackingResults();
            Controls.Add(btnTrackExport);
            y += 38;

            lblTrackStatus = new Label
            {
                Text = "Select 1 point/marker and 2 angle references in SolidWorks, then confirm.",
                Location = new Point(10, y),
                Size = new Size(520, 34),
                ForeColor = Color.DimGray,
                Font = new Font("Microsoft YaHei", 7.5f)
            };
            Controls.Add(lblTrackStatus);
            y += 36;

            lblTrackLive = new Label
            {
                Text = "No realtime samples.",
                Location = new Point(10, y),
                Size = new Size(520, 38),
                ForeColor = Color.DarkBlue,
                Font = new Font("Microsoft YaHei", 8.5f, FontStyle.Bold)
            };
            Controls.Add(lblTrackLive);
            y += 44;

            // ── 状态栏 ───────────────────────────────────────────────
            string swPath = SwPathResolver.FindInstallPath() ?? "未检测到";
            lblStatus = new Label
            {
                Text = "就绪  |  " + swPath,
                Location = new Point(10, y), Size = new Size(400, 36),
                ForeColor = Color.DarkGreen, Font = new Font("微软雅黑", 7.5f)
            };
            Controls.Add(lblStatus);
        }

        private void OnRunClick(object sender, EventArgs e)
        {
            // 写回配置
            m_Addin.Joint1DimName  = txJ1Dim.Text.Trim();
            m_Addin.Joint2DimName  = txJ2Dim.Text.Trim();
            m_Addin.J1Min          = (double)numJ1Min.Value;
            m_Addin.J1Max          = (double)numJ1Max.Value;
            m_Addin.J2Min          = (double)numJ2Min.Value;
            m_Addin.J2Max          = (double)numJ2Max.Value;
            m_Addin.StepSizeCoarse = (double)numStepCoarse.Value;
            m_Addin.StepSizeMedium = (double)numStepMedium.Value;
            m_Addin.StepSizeFine   = (double)numStepFine.Value;
            m_Addin.StepSize       = m_Addin.StepSizeCoarse;
            m_Addin.FixJoint1      = ckFixJ1.Checked;
            m_Addin.FixJoint2      = ckFixJ2.Checked;
            m_Addin.FixedJ1Value   = (double)numFixJ1.Value;
            m_Addin.FixedJ2Value   = (double)numFixJ2.Value;
            m_Addin.EnableInterferenceCheck = ckInterference == null || ckInterference.Checked;
            m_Addin.EnableFastCoordinateAngleSampling = ckFastCoordinateCsv == null || ckFastCoordinateCsv.Checked;
            m_Addin.EnableLinePlaneProjection(ckProjEnable != null && ckProjEnable.Checked);

            if (string.IsNullOrWhiteSpace(m_Addin.Joint1DimName) ||
                string.IsNullOrWhiteSpace(m_Addin.Joint2DimName))
            {
                MessageBox.Show("请填写关节配合名称。", "输入错误",
                    MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }
            if (!m_Addin.FixJoint1 && m_Addin.J1Min >= m_Addin.J1Max)
            {
                MessageBox.Show("关节1: 最小值不能大于等于最大值。", "输入错误",
                    MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }
            if (!m_Addin.FixJoint2 && m_Addin.J2Min >= m_Addin.J2Max)
            {
                MessageBox.Show("关节2: 最小值不能大于等于最大值。", "输入错误",
                    MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }
            if (!(m_Addin.StepSizeCoarse >= m_Addin.StepSizeMedium && m_Addin.StepSizeMedium >= m_Addin.StepSizeFine))
            {
                MessageBox.Show("步长需满足：粗步长 >= 中步长 >= 细步长。", "输入错误",
                    MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }
            if (ckProjEnable != null && ckProjEnable.Checked && !m_Addin.HasLinePlaneRefs())
            {
                MessageBox.Show("已启用双坐标系参数，但尚未选择“2 个子坐标系”。\n请先点击“选 2 个子坐标系”。",
                    "输入错误", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            if (m_Addin.EnableFastCoordinateAngleSampling && !m_Addin.EnableInterferenceCheck)
            {
                if (!m_Addin.HasScanPointRef())
                {
                    MessageBox.Show("Fast coordinate-angle sampling needs a scan point.\nSelect the point/marker and click Pick scan point first.",
                        "Missing scan point", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }
            }

            // Preflight: verify joints can be driven before starting a long sweep
            if (!m_Addin.PreflightCheckDrive(out var preflightMsg))
            {
                var logPath = Path.Combine(Path.GetTempPath(), "SWAnkleInterference.log");
                MessageBox.Show(
                    "自检未通过，扫描已停止。\n\n" +
                    preflightMsg + "\n\n" +
                    "请查看日志：\n" + logPath,
                    "驱动自检失败",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Warning);
                return;
            }

            int s1 = m_Addin.FixJoint1 ? 1 :
                     (int)Math.Round((m_Addin.J1Max - m_Addin.J1Min) / m_Addin.StepSizeCoarse) + 1;
            int s2 = m_Addin.FixJoint2 ? 1 :
                     (int)Math.Round((m_Addin.J2Max - m_Addin.J2Min) / m_Addin.StepSizeCoarse) + 1;
            int total = s1 * s2;

            if (MessageBox.Show(
                "即将扫描 " + total + " 种配置。\n" +
                "关节1: " + s1 + " 步  关节2: " + s2 + " 步\n\n继续？",
                "确认", MessageBoxButtons.YesNo,
                MessageBoxIcon.Question) != DialogResult.Yes)
                return;

            btnRun.Enabled    = false;
            btnExport.Enabled = false;
            btnStop.Enabled   = true;
            lblStatus.Text    = "运行中...";
            lblStatus.ForeColor = Color.DarkOrange;

            m_Progress = new ProgressForm();
            m_Progress.CancelRequested += (ps, pe) =>
            {
                try { btnStop.Enabled = false; } catch { }
                m_Worker?.CancelAsync();
            };
            m_Progress.Show(this);

            m_Worker = new BackgroundWorker
            {
                WorkerReportsProgress      = true,
                WorkerSupportsCancellation = true
            };
            m_Worker.DoWork += (ws, we) =>
                m_Addin.RunSweepAnalysis(m_Worker, we);

            m_Worker.ProgressChanged += (ws, we) =>
            {
                var r = we.UserState as InterferenceResult;
                if (r != null && m_Progress != null && !m_Progress.IsDisposed)
                    m_Progress.Update(we.ProgressPercentage, r);
            };

            m_Worker.RunWorkerCompleted += (ws, we) =>
            {
                string summary;
                if (we.Cancelled)
                {
                    summary = "已取消。";
                    lblStatus.ForeColor = Color.Gray;
                }
                else if (we.Error != null)
                {
                    summary = "出错: " + we.Error.Message;
                    lblStatus.ForeColor = Color.Red;
                    MessageBox.Show(summary, "错误");
                }
                else
                {
                    var res = m_Addin.GetResults();
                    int ifr = res.Count(r2 => r2.IsInterference);
                    summary = "完成 " + res.Count + " 步  |  干涉 " + ifr +
                              "  |  正常 " + (res.Count - ifr);
                    string liveCsv = m_Addin.GetLiveCsvPath();
                    if (!string.IsNullOrWhiteSpace(liveCsv))
                        summary += "\n自动保存: " + liveCsv;
                    lblStatus.ForeColor = ifr > 0 ? Color.DarkRed : Color.DarkGreen;
                    btnExport.Enabled   = true;
                }
                lblStatus.Text = summary;
                m_Progress?.SetFinished(summary);
                btnRun.Enabled = true;
                try { btnStop.Enabled = false; } catch { }
            };

            m_Worker.RunWorkerAsync();
        }

        private void StartRealtimeTracking()
        {
            if (!m_Addin.HasRealtimeTrackingRefs())
            {
                MessageBox.Show("Please confirm the tracking selection first.", "Missing selection",
                    MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            m_Addin.ClearRealtimeTrackingSamples();
            m_TrackStartTime = DateTime.Now;

            if (m_TrackTimer == null)
            {
                m_TrackTimer = new Timer();
                m_TrackTimer.Tick += (s, e) => AppendRealtimeTrackingSample();
            }
            m_TrackTimer.Interval = Math.Max(50, (int)numTrackInterval.Value);
            m_TrackTimer.Start();

            btnTrackStart.Enabled = false;
            btnTrackStop.Enabled = true;
            btnTrackExport.Enabled = false;
            btnTrackCapture.Enabled = false;
            numTrackInterval.Enabled = false;
            lblTrackStatus.Text = "Realtime tracking...";
            lblTrackStatus.ForeColor = Color.DarkOrange;

            AppendRealtimeTrackingSample();
        }

        private void StopRealtimeTracking()
        {
            try { m_TrackTimer?.Stop(); } catch { }

            btnTrackStart.Enabled = m_Addin.HasRealtimeTrackingRefs();
            btnTrackStop.Enabled = false;
            btnTrackExport.Enabled = m_Addin.GetRealtimeTrackingSampleCount() > 0;
            btnTrackCapture.Enabled = true;
            numTrackInterval.Enabled = true;
            lblTrackStatus.Text = "Tracking stopped. Samples: " + m_Addin.GetRealtimeTrackingSampleCount();
            lblTrackStatus.ForeColor = Color.DarkGreen;
        }

        private void AppendRealtimeTrackingSample()
        {
            try
            {
                if (m_Addin.TryAppendRealtimeTrackingSample(m_TrackStartTime, out var sample, out var err))
                {
                    lblTrackLive.Text =
                        "#" + sample.Index + "  t=" + sample.ElapsedSeconds.ToString("F2") + "s  " +
                        "XYZ=" + sample.Xmm.ToString("F3") + ", " +
                        sample.Ymm.ToString("F3") + ", " +
                        sample.Zmm.ToString("F3") + " mm  " +
                        "Angle=" + sample.AngleDeg.ToString("F3") + " deg";
                    btnTrackExport.Enabled = false;
                    return;
                }

                lblTrackLive.Text = "Tracking read failed: " + err;
                lblTrackLive.ForeColor = Color.DarkRed;
            }
            catch (Exception ex)
            {
                lblTrackLive.Text = "Tracking exception: " + ex.Message;
                lblTrackLive.ForeColor = Color.DarkRed;
            }
        }

        private bool TryCaptureTwoRefs(out string error)
        {
            error = "";
            try
            {
                IModelDoc2 doc = m_Addin?.ActiveDocForUi();
                if (doc == null) { error = "未检测到当前文档。"; return false; }

                ISelectionMgr selMgr = doc.SelectionManager as ISelectionMgr;
                int count = selMgr.GetSelectedObjectCount2(-1);
                if (count != 2)
                {
                    error = "请先在 SolidWorks 中只选中 2 个对象（基准面或平面面）。当前数量: " + count;
                    return false;
                }

                m_RefType1 = selMgr.GetSelectedObjectType3(1, -1);
                m_RefType2 = selMgr.GetSelectedObjectType3(2, -1);
                object o1 = selMgr.GetSelectedObject6(1, -1);
                object o2 = selMgr.GetSelectedObject6(2, -1);

                double[] n1 = GetNormalFromRef(o1, m_RefType1);
                double[] n2 = GetNormalFromRef(o2, m_RefType2);
                if (n1 == null || n2 == null)
                {
                    error = "你选中的对象里至少有一个不是平面（可能是圆柱/曲面），请重新选择两个平面。";
                    return false;
                }

                // Persist references
                try { m_RefPersist1 = doc.Extension.GetPersistReference3(o1) as byte[]; } catch { m_RefPersist1 = null; }
                try { m_RefPersist2 = doc.Extension.GetPersistReference3(o2) as byte[]; } catch { m_RefPersist2 = null; }
                if (m_RefPersist1 == null || m_RefPersist2 == null)
                {
                    error = "无法获取持久引用（Persist Reference）。请改选基准面/平面面后再试。";
                    return false;
                }

                string n1Name = GetRefDisplayName(o1, m_RefType1);
                string n2Name = GetRefDisplayName(o2, m_RefType2);
                lblRefNames.Text = "参考对象: 1) " + n1Name + "   2) " + n2Name;

                return true;
            }
            catch (Exception ex)
            {
                error = "捕获参考对象失败: " + ex.Message;
                return false;
            }
        }

        private double MeasureRefAngle()
        {
            try
            {
                IModelDoc2 doc = m_Addin?.ActiveDocForUi();
                if (doc == null) return double.NaN;

                // 优先用 SW Measure：与右下角“测量”结果一致（自动处理装配体变换）
                try
                {
                    object o1m = ResolvePersist(doc, m_RefPersist1);
                    object o2m = ResolvePersist(doc, m_RefPersist2);
                    if (o1m != null && o2m != null)
                    {
                        doc.ClearSelection2(true);
                        if (TrySelectObjectForMeasure(o1m) && TrySelectObjectForMeasure(o2m))
                        {
                            if (AnkleInterferenceAddIn.TryMeasureAngleUsingSwMeasure(doc, out double deg))
                            {
                                doc.ClearSelection2(true);
                                return deg;
                            }
                        }
                        doc.ClearSelection2(true);
                    }
                }
                catch { try { doc.ClearSelection2(true); } catch { } }

                object o1 = ResolvePersist(doc, m_RefPersist1);
                object o2 = ResolvePersist(doc, m_RefPersist2);
                var n1 = GetNormalFromRef(o1, m_RefType1);
                var n2 = GetNormalFromRef(o2, m_RefType2);
                if (n1 == null || n2 == null) return double.NaN;
                return AnkleInterferenceAddIn.CalcAngleDegForPanel(n1, n2);
            }
            catch { return double.NaN; }
        }

        private static bool TrySelectObjectForMeasure(object obj)
        {
            if (obj == null) return false;
            try
            {
                // 常见：IFeature / IFace2 / IRefPlane 等都带 Select2/Select4
                dynamic d = obj;
                try { return (bool)d.Select4(true, null); } catch { }
                try { return (bool)d.Select2(true, -1); } catch { }
                try { return (bool)d.Select(true); } catch { }
            }
            catch { }
            return false;
        }

        private static object ResolvePersist(IModelDoc2 doc, byte[] persist)
        {
            if (doc == null || persist == null || persist.Length == 0) return null;
            try
            {
                int err = 0;
                object obj = doc.Extension.GetObjectByPersistReference3(persist, out err);
                return err == 0 ? obj : null;
            }
            catch { return null; }
        }

        private static double[] GetNormalFromRef(object obj, int type)
        {
            return AnkleInterferenceAddIn.TryGetPlaneNormalFromSelectionObject(obj, type);
        }

        private static string GetRefDisplayName(object obj, int type)
        {
            try
            {
                if (obj == null) return "(null)";
                if (type == (int)swSelectType_e.swSelDATUMPLANES)
                {
                    if (obj is IFeature f && !string.IsNullOrWhiteSpace(f.Name)) return f.Name;
                    return "基准面";
                }
                else if (type == (int)swSelectType_e.swSelFACES)
                {
                    return "平面面";
                }
            }
            catch { }
            return "(对象)";
        }

        private void RunSingleJointDriveTest()
        {
            try
            {
                if (m_RefPersist1 == null || m_RefPersist2 == null)
                {
                    int jointIndexNoRefs = cbTestJoint.SelectedIndex == 0 ? 1 : 2;
                    string mateNameNoRefs = cbTestJoint.SelectedIndex == 0
                        ? txJ1Dim.Text.Trim()
                        : txJ2Dim.Text.Trim();

                    double targetNoRefs = (double)numTestTarget.Value;
                    if (string.IsNullOrWhiteSpace(mateNameNoRefs))
                    {
                        MessageBox.Show("Please fill or pick the joint mate first.", "Input error",
                            MessageBoxButtons.OK, MessageBoxIcon.Warning);
                        return;
                    }

                    if (!m_Addin.TryDriveJointAngle(jointIndexNoRefs, mateNameNoRefs, targetNoRefs, out var msgNoRefs))
                    {
                        AnkleInterferenceAddIn.LogStatic($"SingleJointTest FAILED(no refs): mate='{mateNameNoRefs}' target={targetNoRefs:F3} msg={msgNoRefs}");
                        MessageBox.Show("Drive failed.\n\n" + msgNoRefs + "\n\nLog: %TEMP%\\SWAnkleInterference.log",
                            "Single joint drive failed", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        return;
                    }

                    MessageBox.Show("Drive command succeeded.\n\nNo reference planes were selected, so only the picked/name mate write was tested.",
                        "Single joint drive succeeded", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    return;
                }

                if (m_RefPersist1 == null || m_RefPersist2 == null)
                {
                    MessageBox.Show("请先点击“选两面作参考角”，并在 SW 里选中两面/两基准面。", "缺少参考角",
                        MessageBoxButtons.OK, MessageBoxIcon.Information);
                    return;
                }

                double before = MeasureRefAngle();
                if (double.IsNaN(before))
                {
                    MessageBox.Show("参考角无效，请重新选择两面/两基准面。", "测量失败",
                        MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }

                int jointIndex = cbTestJoint.SelectedIndex == 0 ? 1 : 2;
                string mateName = cbTestJoint.SelectedIndex == 0
                    ? txJ1Dim.Text.Trim()
                    : txJ2Dim.Text.Trim();

                double target = (double)numTestTarget.Value;
                if (string.IsNullOrWhiteSpace(mateName))
                {
                    MessageBox.Show("请先填写要测试的配合名称。", "输入错误",
                        MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }

                if (!m_Addin.TryDriveJointAngle(jointIndex, mateName, target, out var msg))
                {
                    AnkleInterferenceAddIn.LogStatic($"SingleJointTest FAILED: mate='{mateName}' target={target:F3} msg={msg}");
                    MessageBox.Show("驱动失败，已停止。\n\n" + msg + "\n\n日志：%TEMP%\\SWAnkleInterference.log",
                        "单关节驱动失败", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }

                double after = MeasureRefAngle();
                double delta = Math.Abs(after - before);
                const double minChangeDeg = 0.2;

                if (double.IsNaN(after) || delta < minChangeDeg)
                {
                    AnkleInterferenceAddIn.LogStatic(
                        $"SingleJointTest NO_MOTION: mate='{mateName}' target={target:F3} before={before:F3} after={after:F3} delta={delta:F3}");
                    MessageBox.Show(
                        "驱动未生效：角度变化过小（模型没有动），已停止。\n\n" +
                        $"Before={before:F2}°, After={after:F2}°, Δ={delta:F2}°\n\n" +
                        "日志：%TEMP%\\SWAnkleInterference.log",
                        "单关节驱动失败", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }

                MessageBox.Show(
                    $"驱动成功。\n\nBefore={before:F2}°\nAfter={after:F2}°\nΔ={delta:F2}°",
                    "单关节驱动成功", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception ex)
            {
                AnkleInterferenceAddIn.LogStatic("SingleJointTest exception: " + ex);
                MessageBox.Show("测试异常: " + ex.Message, "错误",
                    MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        // ── UI 辅助方法 ───────────────────────────────────────────────
        private void Lbl(string text, int x, ref int y, bool bold = false)
        {
            Controls.Add(new Label
            {
                Text = text, Location = new Point(x, y), AutoSize = true,
                Font = bold ? new Font("微软雅黑", 9, FontStyle.Bold)
                            : new Font("微软雅黑", 9)
            });
            y += 22;
        }

        private TextBox Txt(string text, int x, int y, int w = 200)
        {
            var tb = new TextBox { Text = text, Location = new Point(x, y), Width = w };
            Controls.Add(tb);
            return tb;
        }

        private NumericUpDown Num(decimal min, decimal max, decimal val, int x, int y)
        {
            var n = new NumericUpDown
            {
                Minimum = min, Maximum = max, Value = val,
                DecimalPlaces = 2, Increment = 0.01m,
                Location = new Point(x, y), Width = 75
            };
            Controls.Add(n);
            return n;
        }
    }
}
