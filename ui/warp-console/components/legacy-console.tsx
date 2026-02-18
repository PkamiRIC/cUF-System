"use client"

import { useEffect, useMemo, useState } from "react"
import { getApiBase } from "@/lib/api-base"

type DeviceStatus = {
  state?: string
  current_sequence?: string | null
  sequence_step?: string | null
  last_error?: string | null
  relay_states?: Record<string, boolean>
  peristaltic_enabled?: boolean
  peristaltic_direction_cw?: boolean
  peristaltic_low_speed?: boolean
  pid_enabled?: boolean
  pid_setpoint?: number
  pid_hall?: number | null
  flow_ml_min?: number
  total_ml?: number
  tmp_mbar?: number | null
  logs?: string[]
}

type ValveConfig = {
  label: string
  relay: number
  offColor: string
}

const apiBase = getApiBase()

const valves: ValveConfig[] = [
  { label: "V1", relay: 1, offColor: "#3b82f6" },
  { label: "V2", relay: 2, offColor: "#3b82f6" },
  { label: "V3", relay: 3, offColor: "#3b82f6" },
  { label: "V8", relay: 4, offColor: "#3b82f6" },
  { label: "V9", relay: 5, offColor: "#3b82f6" },
  { label: "V10", relay: 6, offColor: "#3b82f6" },
  { label: "Collect\nElution", relay: 7, offColor: "#6366f1" },
]

const legacySequenceMap: Record<string, string> = {
  Deaeration: "deaeration",
  Concentration: "concentration",
  Elution: "elution",
  "Clean 1": "clean1",
  "Clean 2": "clean2",
}

async function post(path: string, body?: unknown) {
  const res = await fetch(`${apiBase}${path}`, {
    method: "POST",
    headers: body ? { "Content-Type": "application/json" } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  })

  if (!res.ok) {
    let detail = ""
    try {
      const data = await res.json()
      if (data?.detail) detail = String(data.detail)
    } catch {
      // ignore parse errors
    }
    throw new Error(detail || `POST ${path} failed (${res.status})`)
  }
}

async function fetchStatus(): Promise<DeviceStatus> {
  const res = await fetch(`${apiBase}/status`, { cache: "no-store" })
  if (!res.ok) throw new Error(`GET /status failed (${res.status})`)
  return res.json()
}

function fmtTimer(totalSeconds: number): string {
  const mins = Math.floor(totalSeconds / 60)
  const secs = totalSeconds % 60
  return `${String(mins).padStart(2, "0")}:${String(secs).padStart(2, "0")}`
}

function withTimestamp(message: string, context = "System"): string {
  const now = new Date()
  const hh = String(now.getHours()).padStart(2, "0")
  const mm = String(now.getMinutes()).padStart(2, "0")
  const ss = String(now.getSeconds()).padStart(2, "0")
  return `[${hh}:${mm}:${ss}] (${context}) ${message}`
}

export default function LegacyConsole() {
  const [status, setStatus] = useState<DeviceStatus>({})
  const [tmpHistory, setTmpHistory] = useState<number[]>([])
  const [elapsedSeconds, setElapsedSeconds] = useState(0)
  const [setpointDraft, setSetpointDraft] = useState("80.0")
  const [targetLitersDraft, setTargetLitersDraft] = useState("1.0")
  const [cleanTimeDraft, setCleanTimeDraft] = useState("20.0")
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const [localLogs, setLocalLogs] = useState<string[]>([])
  const [stopFlash, setStopFlash] = useState(false)
  const [resetFlash, setResetFlash] = useState(false)

  const flowLpm = ((status.flow_ml_min ?? 0) / 1000).toFixed(2)
  const totalLiters = ((status.total_ml ?? 0) / 1000).toFixed(2)
  const hallHigh = Boolean(status.pid_hall)

  const eventLog = useMemo(() => {
    const remote = Array.isArray(status.logs) ? status.logs : []
    const merged = [...remote, ...localLogs]
    return merged.slice(-500)
  }, [status.logs, localLogs])

  const addLog = (message: string, context = "System") => {
    setLocalLogs((prev) => [...prev, withTimestamp(message, context)].slice(-500))
  }

  const refreshStatus = async () => {
    try {
      const data = await fetchStatus()
      setStatus(data)
      if (!busy && typeof data.pid_setpoint === "number") {
        setSetpointDraft(data.pid_setpoint.toFixed(1))
      }
      setError(null)
    } catch (err: any) {
      setError(err?.message || "Status refresh failed")
    }
  }

  useEffect(() => {
    addLog("Control console ready.")
    refreshStatus()
    const poll = setInterval(refreshStatus, 1000)
    const tick = setInterval(() => setElapsedSeconds((s) => s + 1), 1000)
    return () => {
      clearInterval(poll)
      clearInterval(tick)
    }
  }, [])

  useEffect(() => {
    const tmp = Number(status.tmp_mbar)
    if (!Number.isFinite(tmp)) return
    setTmpHistory((prev) => [...prev, tmp].slice(-100))
  }, [status.tmp_mbar])

  const waitUntilIdle = async (timeoutMs = 240000) => {
    const started = Date.now()
    while (Date.now() - started < timeoutMs) {
      const s = await fetchStatus()
      setStatus(s)
      const running = String(s.state || "").toUpperCase() === "RUNNING"
      if (!running && !s.current_sequence) return
      await new Promise((resolve) => setTimeout(resolve, 1000))
    }
    throw new Error("Timed out waiting for sequence to finish")
  }

  const tmpGraph = useMemo(() => {
    if (tmpHistory.length < 2) return null
    const width = 600
    const height = 180
    const min = Math.min(...tmpHistory)
    const max = Math.max(...tmpHistory)
    const range = Math.max(1, max - min)
    const points = tmpHistory
      .map((v, i) => {
        const x = (i / (tmpHistory.length - 1)) * (width - 1)
        const y = height - 1 - ((v - min) / range) * (height - 1)
        return `${x.toFixed(2)},${y.toFixed(2)}`
      })
      .join(" ")
    return { width, height, points, min, max }
  }, [tmpHistory])

  const toggleValve = async (relay: number, enabled: boolean) => {
    try {
      await post(`/relays/${relay}/${enabled ? "on" : "off"}`)
      addLog(`Valve ${relay} ${enabled ? "OPEN" : "CLOSED"}`)
      await refreshStatus()
    } catch (err: any) {
      setError(err?.message || "Valve toggle failed")
    }
  }

  const togglePumpEnable = async () => {
    try {
      await post("/peristaltic/enable", { enabled: !status.peristaltic_enabled })
      addLog(`Pump ${status.peristaltic_enabled ? "DISABLED" : "ENABLED"}`)
      await refreshStatus()
    } catch (err: any) {
      setError(err?.message || "Pump enable failed")
    }
  }

  const togglePumpDirection = async () => {
    try {
      const next = !status.peristaltic_direction_cw
      await post("/peristaltic/direction", { forward: next })
      addLog(`Pump direction set to ${next ? "CW" : "CCW"}`)
      await refreshStatus()
    } catch (err: any) {
      setError(err?.message || "Pump direction failed")
    }
  }

  const togglePumpSpeed = async () => {
    try {
      const next = !status.peristaltic_low_speed
      await post("/peristaltic/speed", { low_speed: next })
      addLog(`Pump speed set to ${next ? "LOW" : "HIGH"}`)
      await refreshStatus()
    } catch (err: any) {
      setError(err?.message || "Pump speed failed")
    }
  }

  const togglePid = async () => {
    try {
      const enabled = !status.pid_enabled
      await post("/pid/enable", { enabled })
      addLog(`PID ${enabled ? "enabled" : "disabled"}`, "PID")
      await refreshStatus()
    } catch (err: any) {
      setError(err?.message || "PID toggle failed")
    }
  }

  const applyPidSetpoint = async () => {
    const value = Number.parseFloat(setpointDraft)
    if (!Number.isFinite(value)) {
      setError("Invalid setpoint value")
      return
    }

    try {
      await post("/pid/setpoint", { value })
      addLog(`PID setpoint applied: ${value.toFixed(1)} mBar`, "PID")
      await refreshStatus()
    } catch (err: any) {
      setError(err?.message || "PID setpoint failed")
    }
  }

  const homePid = async () => {
    try {
      await post("/pid/home")
      addLog("PID homing requested", "PID")
      await refreshStatus()
    } catch (err: any) {
      setError(err?.message || "PID home failed")
    }
  }

  const runLegacySequence = async (legacyName: string) => {
    const mapped = legacySequenceMap[legacyName]
    if (!mapped) return

    const liters = Number.parseFloat(targetLitersDraft)
    const payload = Number.isFinite(liters) ? { target_volume_ml: liters * 1000 } : undefined

    setBusy(true)
    setError(null)
    try {
      addLog(`${legacyName} sequence started.`, legacyName)
      await post(`/command/start/${mapped}`, payload)
      await waitUntilIdle()
      addLog(`${legacyName} sequence completed.`, legacyName)
    } catch (err: any) {
      setError(err?.message || `${legacyName} failed`)
      addLog(`${legacyName} sequence error: ${err?.message || "unknown"}`, legacyName)
    } finally {
      setBusy(false)
      await refreshStatus()
    }
  }

  const runFullSequence = async () => {
    setBusy(true)
    setError(null)
    try {
      const liters = Number.parseFloat(targetLitersDraft)
      const payload = Number.isFinite(liters) ? { target_volume_ml: liters * 1000 } : undefined
      addLog("Full Sequence started.", "Full Sequence")
      await post("/command/start/full_sequence", payload)
      await waitUntilIdle()
      addLog("Full Sequence completed.", "Full Sequence")
    } catch (err: any) {
      setError(err?.message || "Full Sequence failed")
      addLog(`Full Sequence error: ${err?.message || "unknown"}`, "Full Sequence")
    } finally {
      setBusy(false)
      await refreshStatus()
    }
  }

  const handleStop = async () => {
    setStopFlash(true)
    setTimeout(() => setStopFlash(false), 1000)
    try {
      await post("/command/emergency_stop")
      addLog("Stop requested for active sequence.")
      await refreshStatus()
    } catch (err: any) {
      setError(err?.message || "Stop failed")
    }
  }

  const resetMetrics = async () => {
    setResetFlash(true)
    setTimeout(() => setResetFlash(false), 1000)
    setElapsedSeconds(0)
    try {
      await post("/flow/reset")
      addLog("Flow meter and timer cleared.")
      await refreshStatus()
    } catch (err: any) {
      setError(err?.message || "Reset failed")
    }
  }

  return (
    <div className="legacy-shell">
      <main className="legacy-main">
        <div className="legacy-top-row">
          <section className="legacy-panel">
            <h2 className="legacy-panel-title">Valves</h2>
            <div className="legacy-valve-grid">
              {valves.map((valve) => {
                const isOpen = Boolean(status.relay_states?.[String(valve.relay)])
                return (
                  <button
                    key={valve.relay}
                    type="button"
                    className="legacy-valve-btn"
                    style={{ backgroundColor: isOpen ? "#f97316" : valve.offColor }}
                    onClick={() => toggleValve(valve.relay, !isOpen)}
                    disabled={busy}
                  >
                    {valve.label}
                  </button>
                )
              })}
            </div>
          </section>

          <section className="legacy-panel legacy-metrics-panel">
            <h2 className="legacy-panel-title">Process Metrics</h2>
            <div className="legacy-metrics-grid">
              <p>Pressure IN: -- mBar</p>
              <p>Pressure OUT: -- mBar</p>
              <p>Filter Pressure: -- mBar</p>
              <p>TMP: -- mBar</p>
            </div>
            <div className="legacy-hall-row">
              <span>Hall Sensor: {status.pid_hall == null ? "--" : hallHigh ? "HIGH" : "LOW"}</span>
              <span className={`legacy-led ${hallHigh ? "on" : ""}`} />
            </div>
            <div className="legacy-flow-row">
              <span>Flow Rate: {flowLpm} L/min</span>
              <span>Total Volume: {totalLiters.padStart(5, "0")} L</span>
            </div>
            <div className="legacy-plot-placeholder">
              {tmpGraph ? (
                <svg viewBox={`0 0 ${tmpGraph.width} ${tmpGraph.height}`} width="100%" height="100%">
                  <polyline
                    fill="none"
                    stroke="#38bdf8"
                    strokeWidth="2"
                    points={tmpGraph.points}
                  />
                </svg>
              ) : (
                "Waiting for TMP data..."
              )}
            </div>
          </section>

          <section className="legacy-panel">
            <h2 className="legacy-panel-title">Controls</h2>
            <div className="legacy-btn-row">
              <button
                className={`legacy-btn ${status.peristaltic_enabled ? "active" : ""}`}
                onClick={togglePumpEnable}
                disabled={busy}
              >
                {status.peristaltic_enabled ? "Pump ON" : "Pump OFF"}
              </button>
              <button className={`legacy-btn ${status.peristaltic_direction_cw ? "active" : ""}`} onClick={togglePumpDirection} disabled={busy}>
                {status.peristaltic_direction_cw ? "Dir: CW" : "Dir CCW"}
              </button>
              <button className={`legacy-btn ${status.peristaltic_low_speed ? "active" : ""}`} onClick={togglePumpSpeed} disabled={busy}>
                {status.peristaltic_low_speed ? "Low Speed" : "High Speed"}
              </button>
            </div>

            <div className="legacy-btn-row">
              <button
                className={`legacy-btn ${status.pid_enabled ? "active-green" : ""}`}
                onClick={togglePid}
                disabled={busy}
              >
                {status.pid_enabled ? "PID Enabled" : "Enable PID"}
              </button>
              <button className="legacy-btn" onClick={homePid} disabled={busy}>
                Home Valve
              </button>
            </div>

            <div className="legacy-form-grid">
              <label htmlFor="setpoint">Setpoint (mBar)</label>
              <input
                id="setpoint"
                value={setpointDraft}
                onChange={(e) => setSetpointDraft(e.target.value)}
                onBlur={applyPidSetpoint}
                onKeyDown={(e) => {
                  if (e.key === "Enter") void applyPidSetpoint()
                }}
              />

              <label htmlFor="target-liters">Target Volume (L)</label>
              <input
                id="target-liters"
                value={targetLitersDraft}
                onChange={(e) => setTargetLitersDraft(e.target.value)}
              />

              <label htmlFor="clean-time">Clean Time (min)</label>
              <input
                id="clean-time"
                value={cleanTimeDraft}
                onChange={(e) => setCleanTimeDraft(e.target.value)}
              />
            </div>
          </section>
        </div>

        <section className="legacy-panel legacy-sequences-panel">
          <h2 className="legacy-panel-title">Sequences</h2>
          <div className="legacy-sequence-grid">
            <button className="legacy-seq-btn" onClick={runFullSequence} disabled={busy}>
              Full Sequence
            </button>
            <button className="legacy-seq-btn" onClick={() => runLegacySequence("Deaeration")} disabled={busy}>
              Deaeration
            </button>
            <button className="legacy-seq-btn" onClick={() => runLegacySequence("Concentration")} disabled={busy}>
              Concentration
            </button>
            <button className="legacy-seq-btn" onClick={() => runLegacySequence("Elution")} disabled={busy}>
              Elution
            </button>
            <button className="legacy-seq-btn" onClick={() => runLegacySequence("Clean 1")} disabled={busy}>
              Clean 1
            </button>
            <button className="legacy-seq-btn" onClick={() => runLegacySequence("Clean 2")} disabled={busy}>
              Clean 2
            </button>
          </div>

          <div className="legacy-bottom-row">
            <div className="legacy-level-row">
              <span>H2O: --</span>
              <span className="legacy-led" />
              <span>NaOH: --</span>
              <span className="legacy-led" />
              <span>Drain Sample: --</span>
              <span className="legacy-led" />
              <span>Drain Cleaning: --</span>
              <span className="legacy-led" />
            </div>

            <div className="legacy-actions-row">
              <span className="legacy-timer">Timer: {fmtTimer(elapsedSeconds)}</span>
              <button className={`legacy-reset-btn ${resetFlash ? "flash" : ""}`} onClick={resetMetrics} disabled={busy}>
                {resetFlash ? "Reset ?" : "Reset Metrics"}
              </button>
              <button className={`legacy-stop-btn ${stopFlash ? "flash" : ""}`} onClick={handleStop}>
                STOP
              </button>
            </div>
          </div>
        </section>

        <section className="legacy-panel legacy-log-panel">
          <h2 className="legacy-panel-title">Event Log</h2>
          <div className="legacy-log-box">
            {eventLog.length === 0 && <div>Control console ready.</div>}
            {eventLog.map((line, idx) => (
              <div key={`${idx}-${line.slice(0, 32)}`}>{line}</div>
            ))}
          </div>
          {error && <p className="legacy-error">{error}</p>}
          {!!status.last_error && <p className="legacy-error">Backend: {status.last_error}</p>}
        </section>
      </main>
    </div>
  )
}
