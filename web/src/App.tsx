import { useEffect, useMemo, useState } from "react";
import { ApiClient } from "./api";
import type { AdviceItem, Apiary, ApiaryTask, Device, DeviceCommand, DeviceEvent, Hive, Organization, SensorReading, User } from "./api";
import {
  AlertsPanel,
  ApiariesScreen,
  AuthView,
  ComparisonPanel,
  DevicesPanel,
  EmptyState,
  EventsPanel,
  HiveDetail,
  HivesTable,
  Sidebar,
  SummaryPanel,
  TipsCard,
  Topbar,
  WeatherCard,
  formatDate,
  indexReadings,
  refreshAll,
  type HiveHistory,
  type HiveSnapshot,
  type HiveState,
  type ChartPeriod,
  type ApiarySummary,
  type AppView
} from "./components/DashboardParts";

const tokenKey = "hivemonitor.accessToken";

export function App() {
  const [token, setToken] = useState(() => localStorage.getItem(tokenKey));
  const client = useMemo(() => new ApiClient(token), [token]);
  const [bootstrapping, setBootstrapping] = useState(Boolean(token));
  const [user, setUser] = useState<User | null>(null);
  const [organizations, setOrganizations] = useState<Organization[]>([]);
  const [apiaries, setApiaries] = useState<Apiary[]>([]);
  const [hives, setHives] = useState<Hive[]>([]);
  const [devices, setDevices] = useState<Device[]>([]);
  const [apiaryEvents, setApiaryEvents] = useState<DeviceEvent[]>([]);
  const [deviceCommands, setDeviceCommands] = useState<DeviceCommand[]>([]);
  const [selectedDevice, setSelectedDevice] = useState<Device | null>(null);
  const [apiaryAdvice, setApiaryAdvice] = useState<AdviceItem[]>([]);
  const [apiaryTasks, setApiaryTasks] = useState<ApiaryTask[]>([]);
  const [includeHiddenAdvice, setIncludeHiddenAdvice] = useState(false);
  const [selectedOrgId, setSelectedOrgId] = useState("");
  const [selectedApiaryId, setSelectedApiaryId] = useState("");
  const [selectedApiariesScreenId, setSelectedApiariesScreenId] = useState("");
  const [selectedHiveId, setSelectedHiveId] = useState("");
  const [activeView, setActiveView] = useState<AppView>("apiary-dashboard");
  const [hiveState, setHiveState] = useState<HiveState>({ latest: [], history: [], events: [], loading: false });
  const [hiveSnapshots, setHiveSnapshots] = useState<HiveSnapshot>({});
  const [hiveHistories, setHiveHistories] = useState<HiveHistory>({});
  const [apiarySummaries, setApiarySummaries] = useState<Record<string, ApiarySummary>>({});
  const [comparisonPeriod, setComparisonPeriod] = useState<ChartPeriod>("1d");
  const [message, setMessage] = useState("");
  const [busy, setBusy] = useState(false);

  const selectedOrg = organizations.find((item) => item.id === selectedOrgId);
  const selectedApiary = apiaries.find((item) => item.id === selectedApiaryId);
  const selectedHive = hives.find((item) => item.id === selectedHiveId);
  const latestByMetric = useMemo(() => indexReadings(hiveState.latest), [hiveState.latest]);
  const snapshotsByHive = useMemo(() => {
    const next = { ...hiveSnapshots };
    if (selectedHiveId) next[selectedHiveId] = hiveState.latest;
    return next;
  }, [hiveSnapshots, hiveState.latest, selectedHiveId]);

  useEffect(() => {
    client.setToken(token);
    if (!token) {
      setUser(null);
      setBootstrapping(false);
      return;
    }
    setBootstrapping(true);
    void refreshAll(client, { setUser, setOrganizations, setMessage }).finally(() => setBootstrapping(false));
  }, [client, token]);

  useEffect(() => {
    if (organizations.length > 0 && !selectedOrgId) setSelectedOrgId(organizations[0].id);
  }, [organizations, selectedOrgId]);

  useEffect(() => {
    if (!token || !selectedOrgId) return;
    void loadApiaries();
  }, [selectedOrgId, token]);

  useEffect(() => {
    if (apiaries.length > 0 && !apiaries.some((item) => item.id === selectedApiaryId)) {
      setSelectedApiaryId(apiaries[0].id);
    }
    if (apiaries.length === 0) setSelectedApiaryId("");
  }, [apiaries, selectedApiaryId]);

  useEffect(() => {
    setSelectedApiariesScreenId("");
    if (apiaries.length === 0) {
      setApiarySummaries({});
      return;
    }
    void loadApiarySummaries(apiaries);
  }, [apiaries]);

  useEffect(() => {
    if (!selectedApiaryId) {
      setHives([]);
      setDevices([]);
      setApiaryEvents([]);
      setApiaryAdvice([]);
      setApiaryTasks([]);
      setHiveSnapshots({});
      setHiveHistories({});
      setDeviceCommands([]);
      return;
    }
    void loadApiaryData(selectedApiaryId, comparisonPeriod);
  }, [selectedApiaryId, comparisonPeriod, includeHiddenAdvice]);

  useEffect(() => {
    if (selectedHiveId && !hives.some((item) => item.id === selectedHiveId)) {
      setSelectedHiveId("");
    }
  }, [hives, selectedHiveId]);

  useEffect(() => {
    setDeviceCommands([]);
    setSelectedDevice(null);
    if (!selectedApiaryId || !selectedHive?.assigned_device_id) return;
    void Promise.all([
      client.deviceCommands(selectedApiaryId, selectedHive.assigned_device_id),
      client.device(selectedApiaryId, selectedHive.assigned_device_id)
    ])
      .then(([commands, device]) => {
        setDeviceCommands(commands);
        setSelectedDevice(device);
      })
      .catch((error) => setMessage(error instanceof Error ? error.message : "Не удалось загрузить команды устройства"));
  }, [client, selectedApiaryId, selectedHive?.assigned_device_id]);

  useEffect(() => {
    if (!selectedHiveId) {
      setHiveState({ latest: [], history: [], events: [], loading: false });
      return;
    }
    void loadHiveData(selectedHiveId, "1d");
  }, [selectedHiveId]);

  async function loadApiaries() {
    if (!selectedOrgId) return;
    await run("Пасеки обновлены", async () => {
      const next = await client.apiaries(selectedOrgId);
      setApiaries(next);
    }, false);
  }

  async function loadApiaryData(apiaryId: string, period: ChartPeriod = comparisonPeriod) {
    await run("Данные пасеки обновлены", async () => {
      const calendarRange = monthRange();
      const [nextHives, nextDevices, nextEvents, nextAdvice, nextTasks] = await Promise.all([
        client.hives(apiaryId),
        client.unassignedDevices(apiaryId),
        client.apiaryEvents(apiaryId),
        client.apiaryAdvice(apiaryId, new Date(), includeHiddenAdvice).catch(() => []),
        client.apiaryTasks(apiaryId, calendarRange.from, calendarRange.to).catch(() => [])
      ]);
      setHives(nextHives);
      setDevices(nextDevices);
      setApiaryEvents(nextEvents);
      setApiaryAdvice(nextAdvice);
      setApiaryTasks(nextTasks);
      const pairs = await Promise.all(
        nextHives.map(async (hive) => [hive.id, await client.latestTelemetry(hive.id)] as const)
      );
      setHiveSnapshots(Object.fromEntries(pairs));
      const historyPairs = await Promise.all(
        nextHives.map(async (hive) => [hive.id, await loadTelemetryHistory(hive.id, period)] as const)
      );
      setHiveHistories(Object.fromEntries(historyPairs));
    }, false);
  }

  async function loadHiveData(hiveId: string, period: ChartPeriod) {
    setHiveState((state) => ({ ...state, loading: true }));
    try {
      const [latest, history, events] = await Promise.all([
        client.latestTelemetry(hiveId),
        loadTelemetryHistory(hiveId, period),
        client.hiveEvents(hiveId)
      ]);
      setHiveState({ latest, history, events, loading: false });
      setHiveSnapshots((state) => ({ ...state, [hiveId]: latest }));
      setHiveHistories((state) => ({ ...state, [hiveId]: history }));
    } catch (error) {
      setHiveState((state) => ({ ...state, loading: false }));
      setMessage(error instanceof Error ? error.message : "Не удалось загрузить улей");
    }
  }

  async function loadHiveHistory(hiveId: string, period: ChartPeriod) {
    setHiveState((state) => ({ ...state, loading: true }));
    try {
      const history = await loadTelemetryHistory(hiveId, period);
      setHiveState((state) => ({ ...state, history, loading: false }));
      setHiveHistories((state) => ({ ...state, [hiveId]: history }));
    } catch (error) {
      setHiveState((state) => ({ ...state, loading: false }));
      setMessage(error instanceof Error ? error.message : "Не удалось загрузить историю улья");
    }
  }

  async function updateTaskStatus(taskId: string, status: ApiaryTask["status"]) {
    if (!selectedApiaryId) return;
    await run("Задача обновлена", async () => {
      await client.updateApiaryTask(selectedApiaryId, taskId, status);
      const range = monthRange();
      setApiaryTasks(await client.apiaryTasks(selectedApiaryId, range.from, range.to));
    });
  }

  async function updateAdviceState(code: string, status: "dismissed" | "snoozed") {
    if (!selectedApiaryId) return;
    await run("Совет обновлен", async () => {
      const snoozedUntil = status === "snoozed" ? new Date(Date.now() + 24 * 60 * 60 * 1000) : undefined;
      await client.updateAdviceState(selectedApiaryId, code, status, snoozedUntil);
      setApiaryAdvice(await client.apiaryAdvice(selectedApiaryId, new Date(), includeHiddenAdvice));
    });
  }

  async function toggleAllAdvice() {
    if (!selectedApiaryId) return;
    const next = !includeHiddenAdvice;
    setIncludeHiddenAdvice(next);
    await run(next ? "Показаны все советы" : "Показаны актуальные советы", async () => {
      setApiaryAdvice(await client.apiaryAdvice(selectedApiaryId, new Date(), next));
    }, false);
  }

  function loadTelemetryHistory(hiveId: string, period: ChartPeriod) {
    const range = chartRange(period);
    return client.telemetryHistory(hiveId, "weight", range);
  }

  async function run(success: string, action: () => Promise<void>, showSuccess = true) {
    setBusy(true);
    setMessage("");
    try {
      await action();
      if (showSuccess) setMessage(success);
    } catch (error) {
      setMessage(error instanceof Error ? error.message : "Ошибка запроса");
    } finally {
      setBusy(false);
    }
  }

  function setSession(nextToken: string, nextUser: User) {
    localStorage.setItem(tokenKey, nextToken);
    setToken(nextToken);
    setUser(nextUser);
  }

  function logout() {
    localStorage.removeItem(tokenKey);
    setToken(null);
    setUser(null);
    setOrganizations([]);
    setApiaries([]);
    setHives([]);
    setDevices([]);
    setSelectedOrgId("");
    setSelectedApiaryId("");
    setSelectedApiariesScreenId("");
    setSelectedHiveId("");
  }

  async function createApiary(input: Partial<Apiary> & { name: string }) {
    if (!selectedOrgId) return;
    const apiary = await client.createApiary({ ...input, organization_id: selectedOrgId });
    const next = await client.apiaries(selectedOrgId);
    setApiaries(next);
    setSelectedApiaryId(apiary.id);
    setSelectedApiariesScreenId(apiary.id);
    return apiary;
  }

  async function deleteApiary(apiary: Apiary) {
    if (!window.confirm(`Удалить пасеку "${apiary.name}"? Это удалит ее ульи, устройства и историю.`)) return;
    await run("Пасека удалена", async () => {
      await client.deleteApiary(apiary.id, apiary.name);
      setSelectedApiaryId((current) => current === apiary.id ? "" : current);
      setSelectedApiariesScreenId("");
      setSelectedHiveId("");
      const next = selectedOrgId ? await client.apiaries(selectedOrgId) : [];
      setApiaries(next);
    });
  }

  async function deleteHive(hive: Hive) {
    if (!selectedApiaryId) return;
    if (!window.confirm(`Удалить улей "${hive.name}"? История улья и активные привязки будут удалены.`)) return;
    await run("Улей удален", async () => {
      await client.deleteHive(hive.id);
      setSelectedHiveId("");
      await loadApiaryData(selectedApiaryId, comparisonPeriod);
    });
  }

  async function deleteDevice(deviceId: string) {
    if (!selectedApiaryId) return;
    if (!window.confirm("Удалить устройство? История устройства и его привязки будут удалены.")) return;
    await run("Устройство удалено", async () => {
      await client.deleteDevice(selectedApiaryId, deviceId);
      await loadApiaryData(selectedApiaryId, comparisonPeriod);
    });
  }

  async function sendDeviceCommand(deviceId: string, command: string, payload: Record<string, unknown> = {}) {
    if (!selectedApiaryId) return;
    setBusy(true);
    setMessage("");
    try {
      const created = await client.createDeviceCommand(selectedApiaryId, deviceId, command, payload);
      setDeviceCommands((items) => [created, ...items.filter((item) => item.id !== created.id)].slice(0, 20));
      setMessage(created.status === "failed" ? `Команда не отправлена: ${created.error_message}` : "Команда отправлена устройству");
      return created;
    } catch (error) {
      setMessage(error instanceof Error ? error.message : "Не удалось отправить команду устройству");
    } finally {
      setBusy(false);
    }
  }

  async function loadDevice(deviceId: string) {
    if (!selectedApiaryId) throw new Error("Пасека не выбрана");
    const device = await client.device(selectedApiaryId, deviceId);
    if (device.id === selectedHive?.assigned_device_id) setSelectedDevice(device);
    return device;
  }

  async function loadApiarySummaries(items: Apiary[]) {
    try {
      const pairs = await Promise.all(
        items.map(async (apiary) => {
          const [apiaryHives, unassignedDevices, events] = await Promise.all([
            client.hives(apiary.id),
            client.unassignedDevices(apiary.id),
            client.apiaryEvents(apiary.id)
          ]);
          return [apiary.id, buildApiarySummary(apiary.id, apiaryHives, unassignedDevices, events)] as const;
        })
      );
      setApiarySummaries(Object.fromEntries(pairs));
    } catch (error) {
      setMessage(error instanceof Error ? error.message : "Не удалось загрузить сводку пасек");
    }
  }

  if (bootstrapping) {
    return <main className="auth-screen"><div className="auth-panel"><p className="eyebrow">HiveMonitor</p><h1>Загрузка</h1></div></main>;
  }

  if (!token || !user) {
    return <AuthView client={client} onAuth={setSession} message={message} setMessage={setMessage} />;
  }

  return (
    <div className={`dashboard-shell ${selectedHive && activeView === "apiary-dashboard" ? "has-detail" : ""}`}>
      <Sidebar
        organizations={organizations}
        apiaries={apiaries}
        selectedOrgId={selectedOrgId}
        selectedApiaryId={selectedApiaryId}
        activeView={activeView}
        onSelectOrg={setSelectedOrgId}
        onSelectApiary={setSelectedApiaryId}
        onSelectView={setActiveView}
        onCreateOrg={(name) => run("Организация создана", async () => {
          const org = await client.createOrganization(name);
          const next = await client.organizations();
          setOrganizations(next);
          setSelectedOrgId(org.id);
        })}
        onCreateApiary={(input) => run("Пасека создана", async () => {
          await createApiary(input);
        })}
        busy={busy}
      />

      <main className="dashboard-main">
        <Topbar
          user={user}
          selectedOrg={selectedOrg}
          selectedApiary={selectedApiary}
          apiaries={apiaries}
          selectedApiaryId={selectedApiaryId}
          onSelectApiary={setSelectedApiaryId}
          onLogout={logout}
        />

        <div className="dashboard-grid">
          <section className="dashboard-content">
            {message && <div className="notice">{message}</div>}
            {activeView === "apiaries" ? (
              <ApiariesScreen
                organizationName={selectedOrg?.name ?? ""}
                apiaries={apiaries}
                summaries={apiarySummaries}
                selectedApiaryId={selectedApiariesScreenId}
                onSelectApiary={setSelectedApiariesScreenId}
                onOpenApiary={(apiaryId) => {
                  setSelectedApiaryId(apiaryId);
                  setActiveView("apiary-dashboard");
                }}
                onDeleteApiary={deleteApiary}
                onCreateApiary={(input) => run("Пасека создана", async () => {
                  await createApiary(input);
                })}
                busy={busy}
              />
            ) : selectedApiary ? (
              <>
                <SummaryPanel hives={hives} devices={devices} events={apiaryEvents} snapshots={snapshotsByHive} />
                <div className="content-row">
                  <AlertsPanel events={apiaryEvents} />
                  <EventsPanel events={apiaryEvents} tasks={apiaryTasks} advice={apiaryAdvice} onTaskStatus={updateTaskStatus} />
                  <DevicesPanel
                    devices={devices}
                    hives={hives}
                    busy={busy}
                    onAssign={(deviceId, hiveId, importMode, replaceExisting) => run("Устройство привязано", async () => {
                      await client.assignDevice(selectedApiaryId, deviceId, hiveId, importMode, replaceExisting);
                      await loadApiaryData(selectedApiaryId);
                      setSelectedHiveId(hiveId);
                    })}
                    onDelete={deleteDevice}
                  />
                </div>
                <div className="lower-dashboard">
                  <div className="lower-main">
                    <HivesTable
                      hives={hives}
                      selectedHiveId={selectedHiveId}
                      snapshots={snapshotsByHive}
                      onSelectHive={setSelectedHiveId}
                      onCreateHive={(input) => run("Улей создан", async () => {
                        const hive = await client.createHive(selectedApiaryId, input);
                        await loadApiaryData(selectedApiaryId, comparisonPeriod);
                        setSelectedHiveId(hive.id);
                      })}
                      busy={busy}
                    />
                    <ComparisonPanel
                      hives={hives}
                      histories={hiveHistories}
                      snapshots={snapshotsByHive}
                      selectedHiveId={selectedHiveId}
                      period={comparisonPeriod}
                      onPeriodChange={setComparisonPeriod}
                      onSelectHive={setSelectedHiveId}
                    />
                  </div>
                  <aside className="insights-column">
                    <WeatherCard apiary={selectedApiary} />
                    <TipsCard
                      advice={apiaryAdvice}
                      events={apiaryEvents}
                      showAll={includeHiddenAdvice}
                      onAdviceState={updateAdviceState}
                      onToggleAll={toggleAllAdvice}
                    />
                  </aside>
                </div>
              </>
            ) : (
              <EmptyState title="Пасека не выбрана" text="Создайте организацию и пасеку, чтобы подключать устройства и смотреть телеметрию." />
            )}
          </section>

        </div>
      </main>

      {selectedHive && activeView === "apiary-dashboard" && (
        <HiveDetail
          hive={selectedHive}
          state={hiveState}
          latestByMetric={latestByMetric}
          commands={deviceCommands}
          device={selectedDevice}
          onLoadHistory={(period) => selectedHiveId ? loadHiveHistory(selectedHiveId, period) : Promise.resolve()}
          onDeleteHive={() => deleteHive(selectedHive)}
          onDeleteDevice={selectedHive.assigned_device_id ? () => deleteDevice(selectedHive.assigned_device_id!) : undefined}
          onSendCommand={selectedHive.assigned_device_id ? (command, payload) => sendDeviceCommand(selectedHive.assigned_device_id!, command, payload) : undefined}
          onLoadDevice={selectedHive.assigned_device_id ? () => loadDevice(selectedHive.assigned_device_id!) : undefined}
          onClose={() => setSelectedHiveId("")}
        />
      )}
    </div>
  );
}

function buildApiarySummary(apiaryId: string, hives: Hive[], devices: Device[], events: DeviceEvent[]): ApiarySummary {
  const activeHiveCount = hives.filter((hive) => hive.status !== "inactive").length;
  const alertCount = events.filter((event) => event.ok === false || event.event_type === "alert").length + hives.filter((hive) => hive.status === "attention").length;
  const telemetryCount = hives.length;
  const status: ApiarySummary["status"] = hives.length === 0 || telemetryCount === 0 ? "no_data" : alertCount > 0 || devices.length > 0 ? "attention" : "normal";
  return {
    apiaryId,
    hiveCount: hives.length,
    activeHiveCount,
    alertCount,
    eventCount: events.length,
    unassignedDeviceCount: devices.length,
    telemetryCount,
    status
  };
}

function chartRange(period: ChartPeriod) {
  const to = new Date();
  const days: Record<ChartPeriod, number> = { "1d": 1, "10d": 10, "30d": 30 };
  const limits: Record<ChartPeriod, number> = { "1d": 96, "10d": 360, "30d": 900 };
  return {
    from: new Date(to.getTime() - days[period] * 24 * 60 * 60 * 1000),
    to,
    limit: limits[period]
  };
}

function monthRange() {
  const now = new Date();
  const from = new Date(now.getFullYear(), now.getMonth(), 1);
  const to = new Date(now.getFullYear(), now.getMonth() + 1, 0, 23, 59, 59, 999);
  return { from, to };
}
