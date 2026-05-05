import { useEffect, useMemo, useState } from "react";
import type { FormEvent, ReactNode } from "react";
import {
  AlertTriangle,
  ArrowUpRight,
  BarChart3,
  Bell,
  Box,
  CalendarDays,
  CheckCircle2,
  ChevronDown,
  CloudSun,
  Filter,
  Gauge,
  HelpCircle,
  Home,
  LayoutDashboard,
  Link2,
  ListChecks,
  LogOut,
  Map,
  PackagePlus,
  Radio,
  Settings,
  ShieldCheck,
  Thermometer,
  Users,
  WifiOff,
  Zap
} from "lucide-react";
import {
  ApiClient,
  Apiary,
  Device,
  DeviceEvent,
  Hive,
  Organization,
  SensorReading,
  User
} from "./api";

const tokenKey = "hivemonitor.accessToken";

const metricLabels: Record<string, string> = {
  weight: "Вес",
  weight_change: "Изменение",
  temperature: "Температура",
  humidity: "Влажность",
  hive_opened: "Открытие",
  battery_percent: "Батарея",
  battery_voltage: "Напряжение",
  rssi: "RSSI",
  free_heap: "Память"
};

type AuthMode = "login" | "register";

type HiveState = {
  latest: SensorReading[];
  history: SensorReading[];
  events: DeviceEvent[];
  loading: boolean;
};

type HiveSnapshot = Record<string, SensorReading[]>;
type HiveHistory = Record<string, SensorReading[]>;

const sidebarNav = [
  { label: "Обзор пасеки", icon: LayoutDashboard, active: true },
  { label: "Ульи", icon: Home },
  { label: "Карта пасеки", icon: Map },
  { label: "Устройства", icon: Radio },
  { label: "События", icon: CalendarDays },
  { label: "Погодные условия", icon: CloudSun },
  { label: "Отчеты", icon: BarChart3 },
  { label: "Советы", icon: HelpCircle }
];

const settingsNav = [
  { label: "Пасека и устройства", icon: Settings },
  { label: "Уведомления", icon: Bell },
  { label: "Пользователи", icon: Users },
  { label: "Интеграции", icon: Link2 }
];

const chartColors = ["#2563eb", "#f5a600", "#16a34a", "#ef4444", "#7c3aed", "#06b6d4", "#f97316", "#64748b"];

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
  const [selectedOrgId, setSelectedOrgId] = useState("");
  const [selectedApiaryId, setSelectedApiaryId] = useState("");
  const [selectedHiveId, setSelectedHiveId] = useState("");
  const [hiveState, setHiveState] = useState<HiveState>({ latest: [], history: [], events: [], loading: false });
  const [hiveSnapshots, setHiveSnapshots] = useState<HiveSnapshot>({});
  const [hiveHistories, setHiveHistories] = useState<HiveHistory>({});
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
    if (!selectedApiaryId) {
      setHives([]);
      setDevices([]);
      setApiaryEvents([]);
      setHiveSnapshots({});
      setHiveHistories({});
      return;
    }
    void loadApiaryData(selectedApiaryId);
  }, [selectedApiaryId]);

  useEffect(() => {
    if (selectedHiveId && !hives.some((item) => item.id === selectedHiveId)) {
      setSelectedHiveId("");
    }
  }, [hives, selectedHiveId]);

  useEffect(() => {
    if (!selectedHiveId) {
      setHiveState({ latest: [], history: [], events: [], loading: false });
      return;
    }
    void loadHiveData(selectedHiveId);
  }, [selectedHiveId]);

  async function loadApiaries() {
    if (!selectedOrgId) return;
    await run("Пасеки обновлены", async () => {
      const next = await client.apiaries(selectedOrgId);
      setApiaries(next);
    }, false);
  }

  async function loadApiaryData(apiaryId: string) {
    await run("Данные пасеки обновлены", async () => {
      const [nextHives, nextDevices, nextEvents] = await Promise.all([
        client.hives(apiaryId),
        client.unassignedDevices(apiaryId),
        client.apiaryEvents(apiaryId)
      ]);
      setHives(nextHives);
      setDevices(nextDevices);
      setApiaryEvents(nextEvents);
      const pairs = await Promise.all(
        nextHives.map(async (hive) => [hive.id, await client.latestTelemetry(hive.id)] as const)
      );
      setHiveSnapshots(Object.fromEntries(pairs));
      const historyPairs = await Promise.all(
        nextHives.map(async (hive) => [hive.id, await client.telemetryHistory(hive.id, "weight", 96)] as const)
      );
      setHiveHistories(Object.fromEntries(historyPairs));
    }, false);
  }

  async function loadHiveData(hiveId: string) {
    setHiveState((state) => ({ ...state, loading: true }));
    try {
      const [latest, history, events] = await Promise.all([
        client.latestTelemetry(hiveId),
        client.telemetryHistory(hiveId, "weight"),
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
    setSelectedHiveId("");
  }

  if (bootstrapping) {
    return <main className="auth-screen"><div className="auth-panel"><p className="eyebrow">HiveMonitor</p><h1>Загрузка</h1></div></main>;
  }

  if (!token || !user) {
    return <AuthView client={client} onAuth={setSession} message={message} setMessage={setMessage} />;
  }

  return (
    <div className={`dashboard-shell ${selectedHive ? "has-detail" : ""}`}>
      <Sidebar
        organizations={organizations}
        apiaries={apiaries}
        selectedOrgId={selectedOrgId}
        selectedApiaryId={selectedApiaryId}
        onSelectOrg={setSelectedOrgId}
        onSelectApiary={setSelectedApiaryId}
        onCreateOrg={(name) => run("Организация создана", async () => {
          const org = await client.createOrganization(name);
          const next = await client.organizations();
          setOrganizations(next);
          setSelectedOrgId(org.id);
        })}
        onCreateApiary={(input) => run("Пасека создана", async () => {
          if (!selectedOrgId) return;
          const apiary = await client.createApiary({ ...input, organization_id: selectedOrgId });
          const next = await client.apiaries(selectedOrgId);
          setApiaries(next);
          setSelectedApiaryId(apiary.id);
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
            {selectedApiary ? (
              <>
                <SummaryPanel hives={hives} devices={devices} events={apiaryEvents} snapshots={snapshotsByHive} />
                <div className="content-row">
                  <AlertsPanel events={apiaryEvents} />
                  <EventsPanel events={apiaryEvents} />
                  <DevicesPanel
                    devices={devices}
                    hives={hives}
                    busy={busy}
                    onAssign={(deviceId, hiveId, importMode) => run("Устройство привязано", async () => {
                      await client.assignDevice(selectedApiaryId, deviceId, hiveId, importMode);
                      await loadApiaryData(selectedApiaryId);
                      setSelectedHiveId(hiveId);
                    })}
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
                        await loadApiaryData(selectedApiaryId);
                        setSelectedHiveId(hive.id);
                      })}
                      busy={busy}
                    />
                    <ComparisonPanel hives={hives} histories={hiveHistories} />
                  </div>
                  <aside className="insights-column">
                    <WeatherCard apiary={selectedApiary} />
                    <TipsCard events={apiaryEvents} />
                  </aside>
                </div>
              </>
            ) : (
              <EmptyState title="Пасека не выбрана" text="Создайте организацию и пасеку, чтобы подключать устройства и смотреть телеметрию." />
            )}
          </section>

        </div>
      </main>

      {selectedHive && (
        <HiveDetail
          hive={selectedHive}
          state={hiveState}
          latestByMetric={latestByMetric}
          onReload={() => selectedHiveId && loadHiveData(selectedHiveId)}
          onClose={() => setSelectedHiveId("")}
        />
      )}
    </div>
  );
}

function AuthView({ client, onAuth, message, setMessage }: { client: ApiClient; onAuth: (token: string, user: User) => void; message: string; setMessage: (value: string) => void }) {
  const [mode, setMode] = useState<AuthMode>("login");
  const [email, setEmail] = useState("demo@hivemonitor.local");
  const [name, setName] = useState("");
  const [password, setPassword] = useState("password123");
  const [busy, setBusy] = useState(false);

  async function submit(event: FormEvent) {
    event.preventDefault();
    setBusy(true);
    setMessage("");
    try {
      const response = mode === "login"
        ? await client.login(email, password)
        : await client.register(email, name, password);
      onAuth(response.access_token, response.user);
    } catch (error) {
      setMessage(error instanceof Error ? error.message : "Ошибка входа");
    } finally {
      setBusy(false);
    }
  }

  return (
    <main className="auth-screen">
      <form className="auth-panel" onSubmit={submit}>
        <div className="brand auth-brand"><LogoMark /><span>HiveMonitor</span></div>
        <h1>{mode === "login" ? "Вход" : "Регистрация"}</h1>
        <div className="segmented">
          <button type="button" className={mode === "login" ? "active" : ""} onClick={() => setMode("login")}>Вход</button>
          <button type="button" className={mode === "register" ? "active" : ""} onClick={() => setMode("register")}>Регистрация</button>
        </div>
        {mode === "register" && <label>Имя<input value={name} onChange={(event) => setName(event.target.value)} /></label>}
        <label>Email<input type="email" value={email} onChange={(event) => setEmail(event.target.value)} required /></label>
        <label>Пароль<input type="password" value={password} onChange={(event) => setPassword(event.target.value)} minLength={8} required /></label>
        {message && <div className="notice error">{message}</div>}
        <button className="primary-action" disabled={busy}>{busy ? "..." : mode === "login" ? "Войти" : "Создать аккаунт"}</button>
      </form>
    </main>
  );
}

function Sidebar({
  organizations,
  apiaries,
  selectedOrgId,
  selectedApiaryId,
  onSelectOrg,
  onSelectApiary,
  onCreateOrg,
  onCreateApiary,
  busy
}: {
  organizations: Organization[];
  apiaries: Apiary[];
  selectedOrgId: string;
  selectedApiaryId: string;
  onSelectOrg: (id: string) => void;
  onSelectApiary: (id: string) => void;
  onCreateOrg: (name: string) => void;
  onCreateApiary: (input: Partial<Apiary> & { name: string }) => void;
  busy: boolean;
}) {
  return (
    <aside className="sidebar">
      <div className="brand"><LogoMark /><span>HiveMonitor</span></div>

      <div className="sidebar-section">
        <span className="sidebar-label">Организация</span>
        <SelectBox value={selectedOrgId} onChange={onSelectOrg} items={organizations} fallback="Нет организаций" />
        <SmallForm placeholder="Новая организация" button="+" onSubmit={onCreateOrg} />
      </div>

      <div className="sidebar-section">
        <span className="sidebar-label">Навигация</span>
        <nav className="side-nav">
          {sidebarNav.map((item) => <button key={item.label} className={item.active ? "active" : ""}><item.icon size={18} />{item.label}</button>)}
        </nav>
      </div>

      <div className="sidebar-section">
        <span className="sidebar-label">Пасека</span>
        <SelectBox value={selectedApiaryId} onChange={onSelectApiary} items={apiaries} fallback="Нет пасек" />
        <CreateApiaryForm disabled={!selectedOrgId || busy} onSubmit={onCreateApiary} />
      </div>

      <div className="sidebar-section">
        <span className="sidebar-label">Настройки</span>
        <nav className="side-nav muted-nav">
          {settingsNav.map((item) => <button key={item.label}><item.icon size={18} />{item.label}</button>)}
        </nav>
      </div>

      <div className="sidebar-art">
        <img src="/images/apiary-meadow.svg" alt="" />
        <button className="support-button"><HelpCircle size={16} />Поддержка</button>
      </div>
    </aside>
  );
}

function Topbar({
  user,
  selectedOrg,
  selectedApiary,
  apiaries,
  selectedApiaryId,
  onSelectApiary,
  onLogout
}: {
  user: User;
  selectedOrg?: Organization;
  selectedApiary?: Apiary;
  apiaries: Apiary[];
  selectedApiaryId: string;
  onSelectApiary: (id: string) => void;
  onLogout: () => void;
}) {
  return (
    <header className="topbar">
      <div className="title-block">
        <div className="title-row"><span className="bee-dot">✽</span><h1>{selectedApiary?.name ?? "Пасека"}</h1></div>
        <div className="selectors-line">
          <span>Организация: <strong>{selectedOrg?.name ?? "не выбрана"}</strong></span>
          <label className="top-select">Пасека:
            <select value={selectedApiaryId} onChange={(event) => onSelectApiary(event.target.value)}>
              {apiaries.map((apiary) => <option key={apiary.id} value={apiary.id}>{apiary.name}</option>)}
            </select>
          </label>
        </div>
      </div>

      <div className="top-actions">
        <span className="system-pill"><ShieldCheck size={15} />Система в норме</span>
        <button className="icon-button" aria-label="Уведомления"><Bell size={19} /><span className="badge-dot">1</span></button>
        <div className="user-card">
          <div className="avatar">{initials(user.name || user.email)}</div>
          <div><strong>{user.name || "Пасечник"}</strong><span>{user.email}</span></div>
          <ChevronDown size={16} />
        </div>
        <button className="icon-button" onClick={onLogout} aria-label="Выйти"><LogOut size={18} /></button>
      </div>
    </header>
  );
}

function SummaryPanel({ hives, devices, events, snapshots }: { hives: Hive[]; devices: Device[]; events: DeviceEvent[]; snapshots: HiveSnapshot }) {
  const active = hives.filter((hive) => hive.status !== "inactive").length;
  const alarms = events.filter((event) => event.ok === false || event.event_type === "alert").length;
  const averageWeightDelta = average(hives.map((hive) => readingValue(snapshots[hive.id], "weight_change")).filter(isNumber));
  const todayEvents = events.filter((event) => isToday(event.occurred_at)).length;

  return (
    <section className="summary-panel panel-card">
      <SectionHeader title="Панель сводок пасеки" info />
      <div className="summary-grid">
        <SummaryCard icon={<Box />} tone="blue" value={hives.length} label="Всего ульев" hint="+1 за неделю" />
        <SummaryCard icon={<CheckCircle2 />} tone="green" value={active} label="Активные ульи" hint={`${percent(active, hives.length)}% от всех`} />
        <SummaryCard icon={<Radio />} tone="gray" value={devices.length} label="Нераспознанные устройства" hint="Ожидают привязки" />
        <SummaryCard icon={<AlertTriangle />} tone="red" value={alarms} label="Всего алармов" hint="Требуют внимания" />
        <SummaryCard icon={<Gauge />} tone="amber" value={formatSigned(averageWeightDelta, " кг")} label="Средний прирост" hint="За сутки" />
        <SummaryCard icon={<CalendarDays />} tone="purple" value={todayEvents} label="Событий сегодня" hint="+3 запланировано" />
      </div>
    </section>
  );
}

function SummaryCard({ icon, tone, value, label, hint }: { icon: ReactNode; tone: string; value: string | number; label: string; hint: string }) {
  return (
    <div className={`summary-card ${tone}`}>
      <span className="summary-icon">{icon}</span>
      <div><strong>{value}</strong><span>{label}</span><small>{hint}</small></div>
    </div>
  );
}

function AlertsPanel({ events }: { events: DeviceEvent[] }) {
  const alerts = events.filter((event) => event.ok === false || event.event_type === "alert" || event.event_type === "hive_opened").slice(0, 4);
  return (
    <section className="panel-card list-panel">
      <SectionHeader title="Предупреждения / Алерты" action="Все алерты" />
      {alerts.length === 0 ? <EmptyInline text="Предупреждений нет" /> : alerts.map((event) => (
        <EventLine key={event.id} event={event} compact />
      ))}
      <button className="text-link">Показать все</button>
    </section>
  );
}

function EventsPanel({ events }: { events: DeviceEvent[] }) {
  return (
    <section className="panel-card list-panel">
      <SectionHeader title="События" />
      <div className="tabs"><span className="active">Важные <b>{Math.min(events.length, 5)}</b></span><span>Запланированные <b>3</b></span><span>Просроченные <b>2</b></span></div>
      {events.slice(0, 5).map((event) => <EventLine key={event.id} event={event} />)}
      {events.length === 0 && <EmptyInline text="Событий пока нет" />}
      <button className="text-link">Все события</button>
    </section>
  );
}

function DevicesPanel({ devices, hives, busy, onAssign }: {
  devices: Device[];
  hives: Hive[];
  busy: boolean;
  onAssign: (deviceId: string, hiveId: string, importMode: string) => void;
}) {
  return (
    <section className="panel-card devices-panel">
      <SectionHeader title="Нераспознанные устройства" />
      <div className="device-list">
        {devices.slice(0, 3).map((device) => (
          <div className="device-row" key={device.id}>
            <Radio size={20} />
            <div><strong>{device.device_id}</strong><span>{device.device_type}</span></div>
            <QuickAssign device={device} hives={hives} busy={busy} onAssign={onAssign} />
          </div>
        ))}
      </div>
      {devices.length === 0 && <EmptyInline text="Новых устройств нет" />}
      <button className="text-link">Все устройства</button>
    </section>
  );
}

function QuickAssign({ device, hives, busy, onAssign }: { device: Device; hives: Hive[]; busy: boolean; onAssign: (deviceId: string, hiveId: string, importMode: string) => void }) {
  return (
    <button className="small-outline" disabled={busy || hives.length === 0} onClick={() => onAssign(device.id, hives[0].id, "attach_to_hive")}>
      Привязать
    </button>
  );
}

function HivesTable({
  hives,
  selectedHiveId,
  snapshots,
  onSelectHive,
  onCreateHive,
  busy
}: {
  hives: Hive[];
  selectedHiveId: string;
  snapshots: HiveSnapshot;
  onSelectHive: (id: string) => void;
  onCreateHive: (input: Partial<Hive> & { name: string }) => void;
  busy: boolean;
}) {
  return (
    <section className="panel-card hives-panel">
      <div className="table-toolbar">
        <SectionHeader title={`Ульи (${hives.length})`} />
        <div className="table-actions"><HiveTabs hives={hives} /><CreateHiveForm disabled={busy} onSubmit={onCreateHive} /><button className="small-outline"><Filter size={15} />Фильтры</button></div>
      </div>
      <div className="hive-table">
        <div className="hive-table-head">
          <span>№ улья</span><span>Тип улья</span><span>Статус</span><span>Изменение веса за сутки</span><span>Вес, кг</span><span>Температура</span><span>Алармы</span><span>Устройство</span><span>Обновлено</span>
        </div>
        {hives.map((hive) => {
          const readings = snapshots[hive.id] ?? [];
          const indexed = indexReadings(readings);
          const weight = indexed.weight;
          const temp = indexed.temperature;
          const delta = indexed.weight_change;
          return (
            <button key={hive.id} className={`hive-table-row ${hive.id === selectedHiveId ? "selected" : ""}`} onClick={() => onSelectHive(hive.id)}>
              <span><ChevronDown size={14} /><b>{hive.number || shortId(hive.id)}</b></span>
              <span>{hive.type || "Дадан"}{hive.super_count ? " с магазином" : ""}</span>
              <StatusPill status={hive.status} />
              <WeightDelta reading={delta} />
              <span>{weight ? valueOnly(weight) : "-"}</span>
              <span>{temp ? formatValue(temp) : "-"}</span>
              <span>{hive.status === "attention" ? <span className="alarm-pill">Слабый прирост</span> : "-"}</span>
              <span className="device-signal"><Radio size={14} />{deviceName(readings)}</span>
              <span>{formatDate(weight?.measured_at)}</span>
            </button>
          );
        })}
      </div>
      {hives.length === 0 && <EmptyState title="Ульев пока нет" text="Создайте первый улей и затем привяжите к нему устройство." />}
      <button className="text-link table-more">Показать все ульи ({hives.length})</button>
    </section>
  );
}

function HiveTabs({ hives }: { hives: Hive[] }) {
  const active = hives.filter((hive) => hive.status !== "inactive").length;
  const attention = hives.filter((hive) => hive.status === "attention").length;
  return <div className="hive-tabs"><span>Все ({hives.length})</span><span>Активные ({active})</span><span>С алармами ({attention})</span></div>;
}

function CreateHiveForm({ disabled, onSubmit }: { disabled: boolean; onSubmit: (input: Partial<Hive> & { name: string }) => void }) {
  const [open, setOpen] = useState(false);
  const [form, setForm] = useState({ name: "", number: "", type: "Дадан", frame_count: "", super_count: "" });
  if (!open) return <button className="small-outline" disabled={disabled} onClick={() => setOpen(true)}><PackagePlus size={15} />Улей</button>;
  return (
    <form className="popover-form" onSubmit={(event) => {
      event.preventDefault();
      if (!form.name.trim()) return;
      onSubmit({
        name: form.name.trim(),
        number: form.number,
        type: form.type,
        frame_count: optionalInt(form.frame_count),
        super_count: optionalInt(form.super_count)
      });
      setForm({ name: "", number: "", type: "Дадан", frame_count: "", super_count: "" });
      setOpen(false);
    }}>
      <input placeholder="Название" value={form.name} onChange={(event) => setForm({ ...form, name: event.target.value })} />
      <input placeholder="Номер" value={form.number} onChange={(event) => setForm({ ...form, number: event.target.value })} />
      <input placeholder="Тип" value={form.type} onChange={(event) => setForm({ ...form, type: event.target.value })} />
      <input placeholder="Рамки" inputMode="numeric" value={form.frame_count} onChange={(event) => setForm({ ...form, frame_count: event.target.value })} />
      <input placeholder="Магазины" inputMode="numeric" value={form.super_count} onChange={(event) => setForm({ ...form, super_count: event.target.value })} />
      <div className="form-actions"><button>Создать</button><button type="button" className="ghost" onClick={() => setOpen(false)}>Отмена</button></div>
    </form>
  );
}

function ComparisonPanel({ hives, histories }: { hives: Hive[]; histories: HiveHistory }) {
  return (
    <section className="panel-card comparison-panel">
      <div className="chart-heading">
        <SectionHeader title="Сравнительный график изменения веса ульев по пасеке" info />
        <div className="range-buttons"><button className="active">1 день</button><button>10 дней</button><button>30 дней</button></div>
      </div>
      <ComparisonChart hives={hives} histories={histories} />
    </section>
  );
}

function HiveDetail({ hive, state, latestByMetric, onReload, onClose }: {
  hive: Hive;
  state: HiveState;
  latestByMetric: Record<string, SensorReading>;
  onReload: () => void;
  onClose: () => void;
}) {
  return (
    <aside className="hive-detail">
      <div className="detail-head">
        <div><span className="back-arrow">←</span><h2>{hive.name}</h2><StatusPill status={hive.status} /></div>
        <button className="icon-button" onClick={onClose} aria-label="Закрыть карточку улья"><span>×</span></button>
      </div>
      <div className="detail-tabs"><span className="active">Обзор</span><span>Графики</span><span>История</span></div>
      <div className="warning-box"><AlertTriangle size={17} />Есть предупреждения <a>Все аларты (1)</a></div>
      <div className="hive-media">
        <img src="/images/hive-stack.svg" alt="Иллюстрация улья" />
        <div>
          <span>Тип улья</span><strong>{hive.type || "Дадан"}{hive.super_count ? " с магазином" : ""}</strong>
          <span>Устройство</span><strong>{deviceName(state.latest)}</strong>
          <span>Установлен</span><strong>{formatDate(hive.created_at)}</strong>
          <span>Местоположение</span><strong>Участок {hive.number || "1"}</strong>
        </div>
      </div>
      <div className="detail-section-title"><h3>Показатели</h3><span>Обновлено: {formatDate(latestByMetric.weight?.measured_at)}</span></div>
      <div className="detail-metrics">
        <ReadingMini title="Вес" reading={latestByMetric.weight} />
        <ReadingMini title="Изменение за 24ч" reading={latestByMetric.weight_change} accent />
        <ReadingMini title="Температура" reading={latestByMetric.temperature} />
        <ReadingMini title="Влажность" reading={latestByMetric.humidity} />
        <ReadingMini title="Батарея" reading={latestByMetric.battery_percent} />
        <ReadingMini title="RSSI" reading={latestByMetric.rssi} />
      </div>
      <section className="detail-chart">
        <SectionHeader title="Изменение веса за последние 24 часа" />
        <TelemetryChart readings={state.history} compact label={hive.name} hives={[hive]} />
      </section>
      <section className="detail-events">
        <SectionHeader title="Последние события" action="Все события" />
        {state.events.slice(0, 5).map((event) => <EventLine key={event.id} event={event} />)}
        {state.events.length === 0 && <EmptyInline text="Событий пока нет" />}
      </section>
      <div className="detail-actions"><button className="ghost" onClick={onReload} disabled={state.loading}><BarChart3 size={17} />Открыть графики</button><button className="primary-action">Перейти к улью <ArrowUpRight size={17} /></button></div>
    </aside>
  );
}

function ReadingMini({ title, reading, accent }: { title: string; reading?: SensorReading; accent?: boolean }) {
  return (
    <div className="reading-mini">
      <span>{title}</span>
      <strong className={accent ? "positive" : ""}>{reading ? formatValue(reading) : "-"}</strong>
      {reading?.metric_type === "rssi" && <small>Отличный</small>}
      {reading?.metric_type === "battery_percent" && <small>{readingValue([reading], "battery_voltage")?.toFixed(1) ?? "3.7"} V</small>}
    </div>
  );
}

function WeatherCard({ apiary }: { apiary?: Apiary }) {
  return (
    <section className="panel-card weather-card">
      <SectionHeader title="Погодные условия" info />
      <div className="weather-main"><CloudSun size={44} /><div><strong>22 °C</strong><span>Переменная облачность</span></div></div>
      <div className="weather-stats"><span>Ветер <b>6 м/с</b></span><span>Влажность <b>58%</b></span><span>Давление <b>1015 гПа</b></span></div>
      <h3>Предупреждения погоды</h3>
      <ul>
        <li><AlertTriangle size={14} />Похолодание ночью до 6°C</li>
        <li><CloudSun size={14} />Ожидается дождь вечером</li>
        <li><Zap size={14} />Порывы ветра до 12 м/с</li>
      </ul>
      <small>{apiary ? `${apiary.locality || apiary.region || "Локация"} · ${apiary.timezone}` : "Локация не выбрана"}</small>
    </section>
  );
}

function TipsCard({ events }: { events: DeviceEvent[] }) {
  const attention = events.find((event) => event.ok === false);
  return (
    <section className="panel-card tips-card">
      <SectionHeader title="Советы / Подсказки" />
      <TipLine icon={<CloudSun />} text="Скоро цветение подсолнуха - не забудьте добавить магазины" />
      <TipLine icon={<Thermometer />} text="Ожидается резкое похолодание ночью до 6°C" />
      <TipLine icon={<AlertTriangle />} text={attention?.message ?? "Проверьте ульи с низким зарядом устройства"} />
      <button className="text-link">Все советы</button>
    </section>
  );
}

function TipLine({ icon, text }: { icon: ReactNode; text: string }) {
  return <div className="tip-line"><span>{icon}</span><p>{text}</p><ChevronDown size={16} /></div>;
}

function TelemetryChart({ readings, label, hives, compact = false }: { readings: SensorReading[]; label: string; hives: Hive[]; compact?: boolean }) {
  if (readings.length === 0) return <div className="chart-empty">История за 24 часа пока пуста</div>;
  const values = readings.map((item) => item.value);
  const min = Math.min(...values);
  const max = Math.max(...values);
  const range = max - min || 1;
  const points = readings.map((item, index) => {
    const x = readings.length === 1 ? 50 : (index / (readings.length - 1)) * 100;
    const y = 88 - ((item.value - min) / range) * 72;
    return `${x},${y}`;
  }).join(" ");
  const ghostLines = hives.slice(0, compact ? 3 : 12);
  return (
    <div className={`chart ${compact ? "compact-chart" : ""}`}>
      <svg viewBox="0 0 100 100" preserveAspectRatio="none" role="img" aria-label="График телеметрии">
        {[20, 40, 60, 80].map((y) => <line key={y} x1="0" x2="100" y1={y} y2={y} className="grid-line" />)}
        {ghostLines.map((hive, index) => (
          <polyline key={hive.id} points={offsetPoints(points, index)} className="ghost-line" vectorEffect="non-scaling-stroke" />
        ))}
        <polyline points={points} className="main-line" vectorEffect="non-scaling-stroke" />
      </svg>
      <div className="chart-legend"><span><i />{label}</span><strong>{formatValue(readings[readings.length - 1])}</strong></div>
    </div>
  );
}

function ComparisonChart({ hives, histories }: { hives: Hive[]; histories: HiveHistory }) {
  const series = hives
    .map((hive, index) => ({ hive, readings: histories[hive.id] ?? [], color: chartColors[index % chartColors.length] }))
    .filter((item) => item.readings.length > 0);

  if (series.length === 0) return <div className="chart-empty">История за 24 часа пока пуста</div>;

  const allValues = series.flatMap((item) => item.readings.map((reading) => reading.value));
  const min = Math.min(...allValues);
  const max = Math.max(...allValues);
  const range = max - min || 1;

  return (
    <div className="chart comparison-chart">
      <svg viewBox="0 0 100 100" preserveAspectRatio="none" role="img" aria-label="Сравнительный график веса ульев">
        {[20, 40, 60, 80].map((y) => <line key={y} x1="0" x2="100" y1={y} y2={y} className="grid-line" />)}
        {series.map((item) => (
          <polyline
            key={item.hive.id}
            points={linePoints(item.readings, min, range)}
            fill="none"
            stroke={item.color}
            strokeWidth="1.8"
            vectorEffect="non-scaling-stroke"
          />
        ))}
      </svg>
      <div className="multi-legend">
        {series.map((item) => (
          <span key={item.hive.id}><i style={{ background: item.color }} />{item.hive.name}</span>
        ))}
      </div>
    </div>
  );
}

function SectionHeader({ title, action, info }: { title: string; action?: string; info?: boolean }) {
  return (
    <div className="section-header">
      <h3>{title}{info && <HelpCircle size={14} />}</h3>
      {action && <a>{action}</a>}
    </div>
  );
}

function EventLine({ event, compact = false }: { event: DeviceEvent; compact?: boolean }) {
  return (
    <div className={`event-line ${compact ? "compact" : ""}`}>
      <EventIcon event={event} />
      <div><strong>{eventTitle(event)}</strong><span>{event.message || event.command || shortId(event.id)}</span></div>
      <time>{formatDate(event.occurred_at)}</time>
    </div>
  );
}

function EventIcon({ event }: { event: DeviceEvent }) {
  if (event.ok === false || event.event_type === "alert") return <span className="event-icon red"><AlertTriangle size={15} /></span>;
  if (event.event_type === "hive_opened") return <span className="event-icon amber"><Thermometer size={15} /></span>;
  return <span className="event-icon green"><CheckCircle2 size={15} /></span>;
}

function StatusPill({ status }: { status: string }) {
  const offline = status === "offline" || status === "no_connection";
  const attention = status === "attention";
  return <span className={`status-pill ${offline ? "offline" : attention ? "attention" : ""}`}>{offline ? "Нет связи" : attention ? "Внимание" : "Активен"}</span>;
}

function WeightDelta({ reading }: { reading?: SensorReading }) {
  if (!reading) return <span>-</span>;
  const positive = reading.value >= 0;
  return <span className={`weight-delta ${positive ? "positive" : "negative"}`}>{formatSigned(reading.value, " кг")} <ArrowUpRight size={14} /></span>;
}

function SelectBox<T extends { id: string; name: string }>({ value, onChange, items, fallback }: { value: string; onChange: (id: string) => void; items: T[]; fallback: string }) {
  return (
    <label className="select-box">
      <select value={value} onChange={(event) => onChange(event.target.value)} disabled={items.length === 0}>
        {items.length === 0 ? <option>{fallback}</option> : items.map((item) => <option key={item.id} value={item.id}>{item.name}</option>)}
      </select>
      <ChevronDown size={15} />
    </label>
  );
}

function SmallForm({ placeholder, button, onSubmit }: { placeholder: string; button: string; onSubmit: (value: string) => void }) {
  const [value, setValue] = useState("");
  return (
    <form className="inline-form" onSubmit={(event) => {
      event.preventDefault();
      if (!value.trim()) return;
      onSubmit(value.trim());
      setValue("");
    }}>
      <input aria-label={placeholder} placeholder={placeholder} value={value} onChange={(event) => setValue(event.target.value)} />
      <button>{button}</button>
    </form>
  );
}

function CreateApiaryForm({ disabled, onSubmit }: { disabled: boolean; onSubmit: (input: Partial<Apiary> & { name: string }) => void }) {
  const [open, setOpen] = useState(false);
  const [form, setForm] = useState({ name: "", country: "Украина", region: "", locality: "", address: "", timezone: "Europe/Kyiv" });
  if (!open) return <button className="sidebar-create" disabled={disabled} onClick={() => setOpen(true)}>Новая пасека</button>;
  return (
    <form className="stack-form" onSubmit={(event) => {
      event.preventDefault();
      if (!form.name.trim()) return;
      onSubmit({ ...form, name: form.name.trim() });
      setForm({ name: "", country: "Украина", region: "", locality: "", address: "", timezone: "Europe/Kyiv" });
      setOpen(false);
    }}>
      <input placeholder="Название пасеки" value={form.name} onChange={(event) => setForm({ ...form, name: event.target.value })} />
      <input placeholder="Регион" value={form.region} onChange={(event) => setForm({ ...form, region: event.target.value })} />
      <input placeholder="Населенный пункт" value={form.locality} onChange={(event) => setForm({ ...form, locality: event.target.value })} />
      <div className="form-actions"><button>Создать</button><button type="button" className="ghost" onClick={() => setOpen(false)}>Отмена</button></div>
    </form>
  );
}

function LogoMark() {
  return <span className="logo-mark">⌘</span>;
}

function EmptyState({ title, text }: { title: string; text: string }) {
  return <div className="empty"><strong>{title}</strong><span>{text}</span></div>;
}

function EmptyInline({ text }: { text: string }) {
  return <p className="empty-inline">{text}</p>;
}

async function refreshAll(client: ApiClient, setters: {
  setUser: (user: User) => void;
  setOrganizations: (items: Organization[]) => void;
  setMessage: (value: string) => void;
}) {
  try {
    const [user, organizations] = await Promise.all([client.me(), client.organizations()]);
    setters.setUser(user);
    setters.setOrganizations(organizations);
  } catch (error) {
    setters.setMessage(error instanceof Error ? error.message : "Сессия недоступна");
  }
}

function indexReadings(readings: SensorReading[]) {
  return readings.reduce<Record<string, SensorReading>>((acc, reading) => {
    acc[reading.metric_type] = reading;
    return acc;
  }, {});
}

function readingValue(readings: SensorReading[] | undefined, metric: string) {
  const reading = readings?.find((item) => item.metric_type === metric);
  return reading?.value;
}

function isNumber(value: unknown): value is number {
  return typeof value === "number" && Number.isFinite(value);
}

function average(values: number[]) {
  if (values.length === 0) return 0;
  return values.reduce((sum, value) => sum + value, 0) / values.length;
}

function percent(value: number, total: number) {
  if (total === 0) return 0;
  return Math.round((value / total) * 100);
}

function optionalInt(value: string) {
  const parsed = Number.parseInt(value, 10);
  return Number.isFinite(parsed) ? parsed : undefined;
}

function formatValue(reading: SensorReading) {
  if (reading.metric_type === "hive_opened") return reading.value > 0 ? "открыт" : "закрыт";
  const value = valueOnly(reading);
  return `${value}${reading.unit ? ` ${reading.unit}` : ""}`;
}

function valueOnly(reading: SensorReading) {
  return Math.abs(reading.value) >= 100 ? reading.value.toFixed(0) : reading.value.toFixed(2).replace(".", ",");
}

function formatSigned(value: number, unit: string) {
  const sign = value > 0 ? "+" : "";
  return `${sign}${value.toFixed(2).replace(".", ",")}${unit}`;
}

function formatDate(value?: string) {
  if (!value) return "";
  return new Intl.DateTimeFormat("ru", { hour: "2-digit", minute: "2-digit" }).format(new Date(value));
}

function isToday(value?: string) {
  if (!value) return false;
  const date = new Date(value);
  const now = new Date();
  return date.toDateString() === now.toDateString();
}

function shortId(value: string) {
  return value.slice(0, 8);
}

function initials(value: string) {
  return value.split(/[ @._-]+/).filter(Boolean).slice(0, 2).map((part) => part[0]?.toUpperCase()).join("") || "П";
}

function deviceName(readings: SensorReading[]) {
  const id = readings[0]?.device_id;
  return id ? `HM-${shortId(id).toUpperCase()}` : "HM-ESP32";
}

function eventTitle(event: DeviceEvent) {
  const labels: Record<string, string> = {
    alert: "Алерт",
    hive_opened: "Открытие улья",
    measurement: "Измерение",
    comment: "Комментарий",
    device_seen: "Новое устройство"
  };
  return labels[event.event_type] ?? event.event_type ?? "Событие";
}

function offsetPoints(points: string, index: number) {
  return points.split(" ").map((pair) => {
    const [x, y] = pair.split(",").map(Number);
    const offset = Math.sin((x + index * 13) / 12) * 4 + index * 1.2 - 4;
    return `${x},${Math.min(94, Math.max(6, y + offset))}`;
  }).join(" ");
}

function linePoints(readings: SensorReading[], min: number, range: number) {
  return readings.map((reading, index) => {
    const x = readings.length === 1 ? 50 : (index / (readings.length - 1)) * 100;
    const y = 88 - ((reading.value - min) / range) * 72;
    return `${x},${y}`;
  }).join(" ");
}
