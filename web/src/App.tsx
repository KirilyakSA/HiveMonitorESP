import { useEffect, useMemo, useState } from "react";
import type { FormEvent, ReactNode } from "react";
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

const metricOrder = [
  "weight",
  "weight_change",
  "temperature",
  "humidity",
  "hive_opened",
  "battery_percent",
  "battery_voltage",
  "rssi"
];

type AuthMode = "login" | "register";

type HiveState = {
  latest: SensorReading[];
  history: SensorReading[];
  events: DeviceEvent[];
  loading: boolean;
};

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
  const [message, setMessage] = useState("");
  const [busy, setBusy] = useState(false);

  const selectedOrg = organizations.find((item) => item.id === selectedOrgId);
  const selectedApiary = apiaries.find((item) => item.id === selectedApiaryId);
  const selectedHive = hives.find((item) => item.id === selectedHiveId);
  const latestByMetric = useMemo(() => indexReadings(hiveState.latest), [hiveState.latest]);

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
    if (organizations.length > 0 && !selectedOrgId) {
      setSelectedOrgId(organizations[0].id);
    }
  }, [organizations, selectedOrgId]);

  useEffect(() => {
    if (!token) return;
    void loadApiaries();
  }, [selectedOrgId, token]);

  useEffect(() => {
    if (apiaries.length > 0 && !apiaries.some((item) => item.id === selectedApiaryId)) {
      setSelectedApiaryId(apiaries[0].id);
    }
    if (apiaries.length === 0) {
      setSelectedApiaryId("");
    }
  }, [apiaries, selectedApiaryId]);

  useEffect(() => {
    if (!selectedApiaryId) {
      setHives([]);
      setDevices([]);
      setApiaryEvents([]);
      return;
    }
    void loadApiaryData(selectedApiaryId);
  }, [selectedApiaryId]);

  useEffect(() => {
    if (hives.length > 0 && !hives.some((item) => item.id === selectedHiveId)) {
      setSelectedHiveId(hives[0].id);
    }
    if (hives.length === 0) {
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
    <div className="app-shell">
      <header className="topbar">
        <div>
          <p className="eyebrow">HiveMonitor</p>
          <h1>Панель пасеки</h1>
        </div>
        <div className="account">
          <span>{user.email}</span>
          <button className="ghost" onClick={logout}>Выйти</button>
        </div>
      </header>

      <main className="workspace">
        <aside className="sidebar">
          <SectionTitle title="Организации" action={<SmallForm placeholder="Название" button="+" onSubmit={(name) => run("Организация создана", async () => {
            const org = await client.createOrganization(name);
            const next = await client.organizations();
            setOrganizations(next);
            setSelectedOrgId(org.id);
          })} />} />
          <NavList
            items={organizations}
            activeId={selectedOrgId}
            label={(item) => item.name}
            meta={(item) => shortId(item.id)}
            onSelect={setSelectedOrgId}
          />

          <SectionTitle title="Пасеки" />
          <CreateApiaryForm disabled={!selectedOrgId || busy} onSubmit={(input) => run("Пасека создана", async () => {
            const apiary = await client.createApiary({ ...input, organization_id: selectedOrgId });
            const next = await client.apiaries(selectedOrgId);
            setApiaries(next);
            setSelectedApiaryId(apiary.id);
          })} />
          <NavList
            items={apiaries}
            activeId={selectedApiaryId}
            label={(item) => item.name}
            meta={(item) => [item.locality, item.region].filter(Boolean).join(", ") || item.timezone}
            onSelect={setSelectedApiaryId}
          />
        </aside>

        <section className="content">
          {message && <div className="notice">{message}</div>}

          <div className="toolbar">
            <div>
              <p className="eyebrow">{selectedOrg?.name ?? "Организация"}</p>
              <h2>{selectedApiary?.name ?? "Выберите пасеку"}</h2>
            </div>
            <button onClick={() => selectedApiaryId && loadApiaryData(selectedApiaryId)} disabled={!selectedApiaryId || busy}>
              Обновить
            </button>
          </div>

          {selectedApiary ? (
            <>
              <ApiarySummary apiary={selectedApiary} hives={hives} devices={devices} events={apiaryEvents} />

              <div className="split">
                <section className="panel">
                  <SectionTitle title="Ульи" />
                  <CreateHiveForm disabled={!selectedApiaryId || busy} onSubmit={(input) => run("Улей создан", async () => {
                    const hive = await client.createHive(selectedApiaryId, input);
                    const next = await client.hives(selectedApiaryId);
                    setHives(next);
                    setSelectedHiveId(hive.id);
                  })} />
                  <HiveList hives={hives} activeId={selectedHiveId} onSelect={setSelectedHiveId} />
                </section>

                <section className="panel">
                  <SectionTitle title="Непривязанные устройства" />
                  <DeviceAssignment devices={devices} hives={hives} disabled={busy} onAssign={(deviceId, hiveId, importMode) => run("Устройство привязано", async () => {
                    await client.assignDevice(selectedApiaryId, deviceId, hiveId, importMode);
                    await loadApiaryData(selectedApiaryId);
                    setSelectedHiveId(hiveId);
                  })} />
                </section>
              </div>

              <section className="panel wide">
                <HiveDashboard hive={selectedHive} state={hiveState} latestByMetric={latestByMetric} onReload={() => selectedHiveId && loadHiveData(selectedHiveId)} />
              </section>

              <div className="split">
                <EventJournal title="Журнал пасеки" events={apiaryEvents} />
                <EventJournal title="Журнал улья" events={hiveState.events} />
              </div>
            </>
          ) : (
            <EmptyState title="Пасека не выбрана" text="Создайте организацию и пасеку, чтобы подключать устройства и смотреть телеметрию." />
          )}
        </section>
      </main>
    </div>
  );
}

function AuthView({ client, onAuth, message, setMessage }: { client: ApiClient; onAuth: (token: string, user: User) => void; message: string; setMessage: (value: string) => void }) {
  const [mode, setMode] = useState<AuthMode>("login");
  const [email, setEmail] = useState("");
  const [name, setName] = useState("");
  const [password, setPassword] = useState("");
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
        <p className="eyebrow">HiveMonitor</p>
        <h1>{mode === "login" ? "Вход" : "Регистрация"}</h1>
        <div className="segmented">
          <button type="button" className={mode === "login" ? "active" : ""} onClick={() => setMode("login")}>Вход</button>
          <button type="button" className={mode === "register" ? "active" : ""} onClick={() => setMode("register")}>Регистрация</button>
        </div>
        {mode === "register" && <label>Имя<input value={name} onChange={(event) => setName(event.target.value)} /></label>}
        <label>Email<input type="email" value={email} onChange={(event) => setEmail(event.target.value)} required /></label>
        <label>Пароль<input type="password" value={password} onChange={(event) => setPassword(event.target.value)} minLength={8} required /></label>
        {message && <div className="notice error">{message}</div>}
        <button disabled={busy}>{busy ? "..." : mode === "login" ? "Войти" : "Создать аккаунт"}</button>
      </form>
    </main>
  );
}

function SectionTitle({ title, action }: { title: string; action?: ReactNode }) {
  return (
    <div className="section-title">
      <h3>{title}</h3>
      {action}
    </div>
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
  const [form, setForm] = useState({ name: "", country: "", region: "", locality: "", address: "", timezone: "UTC" });
  if (!open) return <button className="secondary full" disabled={disabled} onClick={() => setOpen(true)}>Новая пасека</button>;
  return (
    <form className="stack-form" onSubmit={(event) => {
      event.preventDefault();
      if (!form.name.trim()) return;
      onSubmit({ ...form, name: form.name.trim() });
      setForm({ name: "", country: "", region: "", locality: "", address: "", timezone: "UTC" });
      setOpen(false);
    }}>
      <input placeholder="Название пасеки" value={form.name} onChange={(event) => setForm({ ...form, name: event.target.value })} />
      <input placeholder="Страна" value={form.country} onChange={(event) => setForm({ ...form, country: event.target.value })} />
      <input placeholder="Регион" value={form.region} onChange={(event) => setForm({ ...form, region: event.target.value })} />
      <input placeholder="Населенный пункт" value={form.locality} onChange={(event) => setForm({ ...form, locality: event.target.value })} />
      <input placeholder="Адрес" value={form.address} onChange={(event) => setForm({ ...form, address: event.target.value })} />
      <input placeholder="Часовой пояс" value={form.timezone} onChange={(event) => setForm({ ...form, timezone: event.target.value })} />
      <div className="form-actions"><button>Создать</button><button type="button" className="ghost" onClick={() => setOpen(false)}>Отмена</button></div>
    </form>
  );
}

function CreateHiveForm({ disabled, onSubmit }: { disabled: boolean; onSubmit: (input: Partial<Hive> & { name: string }) => void }) {
  const [form, setForm] = useState({ name: "", number: "", type: "Дадан", frame_count: "", super_count: "", bee_breed: "", notes: "" });
  return (
    <form className="hive-form" onSubmit={(event) => {
      event.preventDefault();
      if (!form.name.trim()) return;
      onSubmit({
        name: form.name.trim(),
        number: form.number,
        type: form.type,
        frame_count: optionalInt(form.frame_count),
        super_count: optionalInt(form.super_count),
        bee_breed: form.bee_breed,
        notes: form.notes
      });
      setForm({ name: "", number: "", type: "Дадан", frame_count: "", super_count: "", bee_breed: "", notes: "" });
    }}>
      <input placeholder="Название улья" value={form.name} onChange={(event) => setForm({ ...form, name: event.target.value })} disabled={disabled} />
      <input placeholder="Номер" value={form.number} onChange={(event) => setForm({ ...form, number: event.target.value })} disabled={disabled} />
      <input placeholder="Тип" value={form.type} onChange={(event) => setForm({ ...form, type: event.target.value })} disabled={disabled} />
      <input placeholder="Рамки" inputMode="numeric" value={form.frame_count} onChange={(event) => setForm({ ...form, frame_count: event.target.value })} disabled={disabled} />
      <input placeholder="Магазины" inputMode="numeric" value={form.super_count} onChange={(event) => setForm({ ...form, super_count: event.target.value })} disabled={disabled} />
      <button disabled={disabled}>Добавить улей</button>
    </form>
  );
}

function NavList<T extends { id: string }>({ items, activeId, label, meta, onSelect }: {
  items: T[];
  activeId: string;
  label: (item: T) => string;
  meta: (item: T) => string;
  onSelect: (id: string) => void;
}) {
  if (items.length === 0) return <p className="muted">Нет данных</p>;
  return (
    <div className="nav-list">
      {items.map((item) => (
        <button key={item.id} className={item.id === activeId ? "active" : ""} onClick={() => onSelect(item.id)}>
          <span>{label(item)}</span>
          <small>{meta(item)}</small>
        </button>
      ))}
    </div>
  );
}

function ApiarySummary({ apiary, hives, devices, events }: { apiary: Apiary; hives: Hive[]; devices: Device[]; events: DeviceEvent[] }) {
  return (
    <section className="summary-band">
      <Metric label="Ульи" value={hives.length} />
      <Metric label="Новые устройства" value={devices.length} />
      <Metric label="События" value={events.length} />
      <Metric label="Локация" value={[apiary.locality, apiary.region].filter(Boolean).join(", ") || "Не задана"} />
    </section>
  );
}

function Metric({ label, value }: { label: string; value: string | number }) {
  return <div className="metric"><span>{label}</span><strong>{value}</strong></div>;
}

function HiveList({ hives, activeId, onSelect }: { hives: Hive[]; activeId: string; onSelect: (id: string) => void }) {
  if (hives.length === 0) return <EmptyState title="Ульев пока нет" text="Создайте первый улей и затем привяжите к нему устройство." />;
  return (
    <div className="hive-list">
      {hives.map((hive) => (
        <button key={hive.id} className={hive.id === activeId ? "active" : ""} onClick={() => onSelect(hive.id)}>
          <strong>{hive.name}</strong>
          <span>{[hive.number && `N ${hive.number}`, hive.type, hive.frame_count && `${hive.frame_count} рамок`, hive.super_count && `${hive.super_count} магаз.`].filter(Boolean).join(" · ")}</span>
        </button>
      ))}
    </div>
  );
}

function DeviceAssignment({ devices, hives, disabled, onAssign }: {
  devices: Device[];
  hives: Hive[];
  disabled: boolean;
  onAssign: (deviceId: string, hiveId: string, importMode: string) => void;
}) {
  const [selectedDevice, setSelectedDevice] = useState("");
  const [selectedHive, setSelectedHive] = useState("");
  const [importMode, setImportMode] = useState("attach_to_hive");
  useEffect(() => {
    if (devices.length > 0 && !devices.some((device) => device.id === selectedDevice)) setSelectedDevice(devices[0].id);
    if (hives.length > 0 && !hives.some((hive) => hive.id === selectedHive)) setSelectedHive(hives[0].id);
  }, [devices, hives, selectedDevice, selectedHive]);

  if (devices.length === 0) return <EmptyState title="Новых устройств нет" text="После настройки firmware с apiaryId устройство появится здесь." />;
  return (
    <form className="stack-form" onSubmit={(event) => {
      event.preventDefault();
      if (selectedDevice && selectedHive) onAssign(selectedDevice, selectedHive, importMode);
    }}>
      <select value={selectedDevice} onChange={(event) => setSelectedDevice(event.target.value)} disabled={disabled}>
        {devices.map((device) => <option key={device.id} value={device.id}>{device.device_id} · {device.firmware_version || "fw ?"}</option>)}
      </select>
      <select value={selectedHive} onChange={(event) => setSelectedHive(event.target.value)} disabled={disabled || hives.length === 0}>
        {hives.map((hive) => <option key={hive.id} value={hive.id}>{hive.name}</option>)}
      </select>
      <select value={importMode} onChange={(event) => setImportMode(event.target.value)} disabled={disabled}>
        <option value="attach_to_hive">Привязать старую телеметрию</option>
        <option value="keep">Оставить диагностической</option>
        <option value="delete">Удалить старую телеметрию</option>
      </select>
      <button disabled={disabled || hives.length === 0}>Привязать</button>
    </form>
  );
}

function HiveDashboard({ hive, state, latestByMetric, onReload }: {
  hive?: Hive;
  state: HiveState;
  latestByMetric: Record<string, SensorReading>;
  onReload: () => void;
}) {
  if (!hive) return <EmptyState title="Улей не выбран" text="Выберите улей, чтобы увидеть текущие показатели." />;
  return (
    <>
      <div className="toolbar compact">
        <div><p className="eyebrow">Улей</p><h2>{hive.name}</h2></div>
        <button onClick={onReload} disabled={state.loading}>{state.loading ? "..." : "Обновить улей"}</button>
      </div>
      <div className="reading-grid">
        {metricOrder.map((metric) => <ReadingCard key={metric} metric={metric} reading={latestByMetric[metric]} />)}
      </div>
      <TelemetryChart readings={state.history} />
    </>
  );
}

function ReadingCard({ metric, reading }: { metric: string; reading?: SensorReading }) {
  return (
    <div className="reading">
      <span>{metricLabels[metric] ?? metric}</span>
      <strong>{reading ? formatValue(reading) : "нет данных"}</strong>
      <small>{reading ? formatDate(reading.measured_at) : "ожидается телеметрия"}</small>
    </div>
  );
}

function TelemetryChart({ readings }: { readings: SensorReading[] }) {
  if (readings.length === 0) return <div className="chart-empty">История за 24 часа пока пуста</div>;
  const values = readings.map((item) => item.value);
  const min = Math.min(...values);
  const max = Math.max(...values);
  const range = max - min || 1;
  const points = readings.map((item, index) => {
    const x = readings.length === 1 ? 50 : (index / (readings.length - 1)) * 100;
    const y = 90 - ((item.value - min) / range) * 75;
    return `${x},${y}`;
  }).join(" ");
  return (
    <div className="chart">
      <div className="chart-head"><span>История</span><strong>{formatValue(readings[readings.length - 1])}</strong></div>
      <svg viewBox="0 0 100 100" preserveAspectRatio="none" role="img" aria-label="График телеметрии">
        <polyline points={points} fill="none" stroke="currentColor" strokeWidth="3" vectorEffect="non-scaling-stroke" />
      </svg>
      <div className="chart-scale"><span>{min.toFixed(1)}</span><span>{max.toFixed(1)}</span></div>
    </div>
  );
}

function EventJournal({ title, events }: { title: string; events: DeviceEvent[] }) {
  return (
    <section className="panel">
      <SectionTitle title={title} />
      {events.length === 0 ? <p className="muted">Событий пока нет</p> : (
        <div className="event-list">
          {events.slice(0, 10).map((event) => (
            <div className="event-row" key={event.id}>
              <strong>{event.event_type || event.command || "event"}</strong>
              <span>{event.message || event.command || shortId(event.id)}</span>
              <time>{formatDate(event.occurred_at)}</time>
            </div>
          ))}
        </div>
      )}
    </section>
  );
}

function EmptyState({ title, text }: { title: string; text: string }) {
  return <div className="empty"><strong>{title}</strong><span>{text}</span></div>;
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

function optionalInt(value: string) {
  const parsed = Number.parseInt(value, 10);
  return Number.isFinite(parsed) ? parsed : undefined;
}

function formatValue(reading: SensorReading) {
  if (reading.metric_type === "hive_opened") return reading.value > 0 ? "открыт" : "закрыт";
  const value = Math.abs(reading.value) >= 100 ? reading.value.toFixed(0) : reading.value.toFixed(2);
  return `${value}${reading.unit ? ` ${reading.unit}` : ""}`;
}

function formatDate(value?: string) {
  if (!value) return "";
  return new Intl.DateTimeFormat("ru", { day: "2-digit", month: "2-digit", hour: "2-digit", minute: "2-digit" }).format(new Date(value));
}

function shortId(value: string) {
  return value.slice(0, 8);
}
