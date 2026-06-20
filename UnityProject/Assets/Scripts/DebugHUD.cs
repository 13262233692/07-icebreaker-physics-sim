using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace IcePhysicsUnity
{
    public class DebugHUD : MonoBehaviour
    {
        [Header("UI References")]
        [SerializeField] private Canvas m_debugCanvas;
        [SerializeField] private Text m_statsText;
        [SerializeField] private Text m_logText;
        [SerializeField] private ScrollRect m_logScrollRect;

        [Header("Settings")]
        [SerializeField] private bool m_showHUD = true;
        [SerializeField] private bool m_showStats = true;
        [SerializeField] private bool m_showLog = true;
        [SerializeField] private int m_maxLogEntries = 100;
        [SerializeField] private KeyCode m_toggleKey = KeyCode.F12;

        [Header("Display Options")]
        [SerializeField] private bool m_showPhysicsStats = true;
        [SerializeField] private bool m_showHydrodynamics = true;
        [SerializeField] private bool m_showIceInfo = true;

        private Queue<string> m_logEntries = new Queue<string>();
        private float m_updateInterval = 0.25f;
        private float m_lastUpdateTime;

        private IcePhysicsManager m_physicsManager;
        private IcebreakerShip m_icebreakerShip;
        private IceSheet[] m_iceSheets;

        private void Awake()
        {
            if (m_debugCanvas == null)
            {
                CreateDebugCanvas();
            }
        }

        private void Start()
        {
            m_physicsManager = IcePhysicsManager.Instance;
            m_icebreakerShip = FindObjectOfType<IcebreakerShip>();
            m_iceSheets = FindObjectsOfType<IceSheet>();

            if (m_physicsManager != null)
            {
                m_physicsManager.OnDebugLog += OnDebugLogHandler;
            }
        }

        private void OnDestroy()
        {
            if (m_physicsManager != null)
            {
                m_physicsManager.OnDebugLog -= OnDebugLogHandler;
            }
        }

        private void Update()
        {
            if (Input.GetKeyDown(m_toggleKey))
            {
                m_showHUD = !m_showHUD;
                if (m_debugCanvas != null)
                {
                    m_debugCanvas.enabled = m_showHUD;
                }
            }

            if (m_showHUD && Time.time - m_lastUpdateTime >= m_updateInterval)
            {
                UpdateStatsDisplay();
                m_lastUpdateTime = Time.time;
            }
        }

        private void CreateDebugCanvas()
        {
            GameObject canvasObj = new GameObject("DebugHUDCanvas");
            m_debugCanvas = canvasObj.AddComponent<Canvas>();
            m_debugCanvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvasObj.AddComponent<CanvasScaler>();
            canvasObj.AddComponent<GraphicRaycaster>();

            GameObject statsObj = new GameObject("StatsText");
            statsObj.transform.SetParent(canvasObj.transform, false);
            m_statsText = statsObj.AddComponent<Text>();
            m_statsText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            m_statsText.fontSize = 14;
            m_statsText.color = Color.white;
            m_statsText.alignment = TextAnchor.UpperLeft;

            RectTransform statsRect = m_statsText.GetComponent<RectTransform>();
            statsRect.anchorMin = new Vector2(0, 1);
            statsRect.anchorMax = new Vector2(0.5f, 1);
            statsRect.pivot = new Vector2(0, 1);
            statsRect.offsetMin = new Vector2(10, -10);
            statsRect.offsetMax = new Vector2(-10, -300);

            GameObject logPanelObj = new GameObject("LogPanel");
            logPanelObj.transform.SetParent(canvasObj.transform, false);
            Image logPanelImage = logPanelObj.AddComponent<Image>();
            logPanelImage.color = new Color(0, 0, 0, 0.5f);

            RectTransform logPanelRect = logPanelObj.GetComponent<RectTransform>();
            logPanelRect.anchorMin = new Vector2(0, 0);
            logPanelRect.anchorMax = new Vector2(1, 0.3f);
            logPanelRect.offsetMin = new Vector2(10, 10);
            logPanelRect.offsetMax = new Vector2(-10, 10);

            m_logScrollRect = logPanelObj.AddComponent<ScrollRect>();
            m_logScrollRect.vertical = true;
            m_logScrollRect.horizontal = false;

            GameObject logContentObj = new GameObject("LogContent");
            logContentObj.transform.SetParent(logPanelObj.transform, false);
            RectTransform logContentRect = logContentObj.AddComponent<RectTransform>();
            logContentRect.anchorMin = new Vector2(0, 1);
            logContentRect.anchorMax = new Vector2(1, 1);
            logContentRect.pivot = new Vector2(0.5f, 1);
            logContentRect.sizeDelta = new Vector2(0, 1000);

            m_logText = logContentObj.AddComponent<Text>();
            m_logText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            m_logText.fontSize = 12;
            m_logText.color = Color.white;
            m_logText.alignment = TextAnchor.UpperLeft;
            m_logText.horizontalOverflow = HorizontalWrapMode.Wrap;
            m_logText.verticalOverflow = VerticalWrapMode.Overflow;

            RectTransform logTextRect = m_logText.GetComponent<RectTransform>();
            logTextRect.anchorMin = new Vector2(0, 1);
            logTextRect.anchorMax = new Vector2(1, 1);
            logTextRect.pivot = new Vector2(0.5f, 1);
            logTextRect.offsetMin = new Vector2(5, -5);
            logTextRect.offsetMax = new Vector2(-5, -5);

            m_logScrollRect.content = logContentRect;
            m_logScrollRect.viewport = logPanelRect;
        }

        private void UpdateStatsDisplay()
        {
            if (m_statsText == null || !m_showStats)
                return;

            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.AppendLine("=== ICE PHYSICS SIMULATION DEBUG ===");
            sb.AppendLine($"Time: {Time.time:F2}s | Delta: {Time.deltaTime * 1000:F1}ms | FPS: {1.0f / Time.deltaTime:F0}");
            sb.AppendLine();

            if (m_showPhysicsStats && m_physicsManager != null)
            {
                sb.AppendLine("--- PHYSICS ENGINE ---");
                sb.AppendLine($"Simulation Time: {m_physicsManager.SimulationTime:F2}s");
                sb.AppendLine($"Frame Count: {m_physicsManager.FrameCount}");
                sb.AppendLine($"Rigid Bodies: {m_physicsManager.ActiveRigidBodyCount}");
                sb.AppendLine($"Ice Sheets: {m_physicsManager.ActiveIceSheetCount}");
                sb.AppendLine($"Active Collisions: {m_physicsManager.ActiveCollisionCount}");
                sb.AppendLine();
            }

            if (m_showHydrodynamics && m_icebreakerShip != null)
            {
                sb.AppendLine("--- ICEBREAKER SHIP ---");
                sb.AppendLine($"Speed: {m_icebreakerShip.CurrentSpeedKnots:F2} knots");
                sb.AppendLine($"Throttle: {m_icebreakerShip.CurrentThrottle * 100:F0}%");
                sb.AppendLine($"Rudder: {m_icebreakerShip.CurrentRudderAngle:F1}°");
                sb.AppendLine($"Heading: {m_icebreakerShip.HeadingDegrees:F1}°");

                if (m_physicsManager != null)
                {
                    float resistance = m_physicsManager.CalculateShipResistance(m_icebreakerShip.CurrentSpeedKnots);
                    sb.AppendLine($"Total Resistance: {resistance * 0.001f:F2} kN");

                    if (resistance > 0)
                    {
                        float effectivePower = resistance * m_icebreakerShip.CurrentSpeedKnots * 0.514444f * 0.001f;
                        sb.AppendLine($"Effective Power: {effectivePower:F2} kW");
                    }
                }
                sb.AppendLine();
            }

            if (m_showIceInfo && m_iceSheets != null && m_iceSheets.Length > 0)
            {
                sb.AppendLine("--- ICE ENVIRONMENT ---");
                foreach (var iceSheet in m_iceSheets)
                {
                    if (iceSheet != null && iceSheet.gameObject.activeSelf)
                    {
                        sb.AppendLine($"Ice Sheet {iceSheet.IceSheetId}:");
                        sb.AppendLine($"  Size: {iceSheet.Size.x:F0}x{iceSheet.Size.y:F0}m");
                        sb.AppendLine($"  Thickness: {iceSheet.Thickness:F2}m");
                        sb.AppendLine($"  Yield Strength: {iceSheet.YieldStrength * 0.000001f:F2} MPa");
                        sb.AppendLine($"  Fragments: {iceSheet.FragmentCount}");

                        PressurePoint[] pressurePoints = iceSheet.GetPressurePoints();
                        if (pressurePoints != null && pressurePoints.Length > 0)
                        {
                            float maxPressure = 0f;
                            foreach (var pp in pressurePoints)
                            {
                                if (pp.pressure > maxPressure) maxPressure = pp.pressure;
                            }
                            sb.AppendLine($"  Max Pressure: {maxPressure * 0.000001f:F3} MPa");
                            sb.AppendLine($"  Pressure Points: {pressurePoints.Length}");
                        }
                    }
                }

                if (m_physicsManager != null)
                {
                    sb.AppendLine();
                    sb.AppendLine("--- ENVIRONMENT ---");
                    sb.AppendLine($"Water Density: {m_physicsManager.WaterDensity:F1} kg/m³");
                    sb.AppendLine($"Water Temp: {m_physicsManager.WaterTemperature:F1} °C");
                    sb.AppendLine($"Ice Concentration: {m_physicsManager.IceConcentration * 100:F0}%");
                }
            }

            m_statsText.text = sb.ToString();
        }

        private void OnDebugLogHandler(DebugLogEntry entry)
        {
            if (!m_showLog || m_logText == null)
                return;

            string timestamp = TimeSpan.FromMilliseconds(entry.timestamp).ToString(@"hh\:mm\:ss\.fff");
            string levelStr = entry.level.ToString().ToUpper();
            string colorTag = GetColorTag(entry.level);

            string logLine = $"<color={colorTag}>[{timestamp}] [{levelStr}] [{entry.subsystem}]</color> {entry.message}";

            m_logEntries.Enqueue(logLine);

            while (m_logEntries.Count > m_maxLogEntries)
            {
                m_logEntries.Dequeue();
            }

            m_logText.text = string.Join("\n", m_logEntries.ToArray());

            Canvas.ForceUpdateCanvases();
            if (m_logScrollRect != null)
            {
                m_logScrollRect.verticalNormalizedPosition = 0f;
            }
        }

        private string GetColorTag(DebugLevel level)
        {
            switch (level)
            {
                case DebugLevel.Error:
                    return "#FF4444";
                case DebugLevel.Warning:
                    return "#FFFF44";
                case DebugLevel.Info:
                    return "#FFFFFF";
                case DebugLevel.Verbose:
                    return "#88FF88";
                case DebugLevel.Debug:
                    return "#8888FF";
                default:
                    return "#FFFFFF";
            }
        }

        public void AddLogEntry(string message, DebugLevel level = DebugLevel.Info, string subsystem = "Unity")
        {
            DebugLogEntry entry = new DebugLogEntry
            {
                level = level,
                timestamp = (ulong)(Time.time * 1000),
                message = message,
                subsystem = subsystem
            };
            OnDebugLogHandler(entry);
        }

        public void ToggleHUD()
        {
            m_showHUD = !m_showHUD;
            if (m_debugCanvas != null)
            {
                m_debugCanvas.enabled = m_showHUD;
            }
        }

        public void ClearLog()
        {
            m_logEntries.Clear();
            if (m_logText != null)
            {
                m_logText.text = string.Empty;
            }
        }
    }
}
