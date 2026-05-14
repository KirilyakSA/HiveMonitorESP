import { useEffect, useState } from "react";
import type { FormEvent, ReactNode } from "react";
import type { LucideIcon } from "lucide-react";
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
  X,
  Zap
} from "lucide-react";
import {
  ApiClient,
  AdviceItem,
  Apiary,
  ApiaryTask,
  Device,
  DeviceEvent,
  Hive,
  Organization,
  SensorReading,
  User
} from "../api";
import { ApiaryMap } from "./MapProvider";

export type AuthMode = "login" | "register";
export type AppView = "apiaries" | "apiary-dashboard";

export type HiveState = {
  latest: SensorReading[];
  history: SensorReading[];
  events: DeviceEvent[];
  loading: boolean;
};

export type HiveSnapshot = Record<string, SensorReading[]>;
export type HiveHistory = Record<string, SensorReading[]>;
export type ChartPeriod = "1d" | "10d" | "30d";
export type ApiarySummary = {
  apiaryId: string;
  hiveCount: number;
  activeHiveCount: number;
  alertCount: number;
  eventCount: number;
  unassignedDeviceCount: number;
  telemetryCount: number;
  status: "normal" | "attention" | "no_data";
};
type EventTab = "important" | "scheduled" | "overdue" | "seasonal";
type DetailTab = "overview" | "charts" | "history";

const chartPeriodLabels: Record<ChartPeriod, string> = {
  "1d": "1 день",
  "10d": "10 дней",
  "30d": "30 дней"
};

const sidebarNav: Array<{ id: AppView | "placeholder"; label: string; icon: LucideIcon }> = [
  { id: "apiaries", label: "Пасеки", icon: Map },
  { id: "apiary-dashboard", label: "Обзор пасеки", icon: LayoutDashboard },
  { id: "placeholder", label: "Ульи", icon: Home },
  { id: "placeholder", label: "Карта пасеки", icon: Map },
  { id: "placeholder", label: "Устройства", icon: Radio },
  { id: "placeholder", label: "События", icon: CalendarDays },
  { id: "placeholder", label: "Погодные условия", icon: CloudSun },
  { id: "placeholder", label: "Отчеты", icon: BarChart3 },
  { id: "placeholder", label: "Советы", icon: HelpCircle }
];

const settingsNav = [
  { label: "Пасека и устройства", icon: Settings },
  { label: "Уведомления", icon: Bell },
  { label: "Пользователи", icon: Users },
  { label: "Интеграции", icon: Link2 }
];

const chartColors = ["#2563eb", "#f5a600", "#16a34a", "#ef4444", "#7c3aed", "#06b6d4", "#f97316", "#64748b"];

export function AuthView({ client, onAuth, message, setMessage }: { client: ApiClient; onAuth: (token: string, user: User) => void; message: string; setMessage: (value: string) => void }) {
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
        <BrandLogo compact={false} />
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

export function Sidebar({
  organizations,
  apiaries,
  selectedOrgId,
  selectedApiaryId,
  activeView,
  onSelectOrg,
  onSelectApiary,
  onSelectView,
  onCreateOrg,
  onCreateApiary,
  busy
}: {
  organizations: Organization[];
  apiaries: Apiary[];
  selectedOrgId: string;
  selectedApiaryId: string;
  activeView: AppView;
  onSelectOrg: (id: string) => void;
  onSelectApiary: (id: string) => void;
  onSelectView: (view: AppView) => void;
  onCreateOrg: (name: string) => void;
  onCreateApiary: (input: Partial<Apiary> & { name: string }) => void;
  busy: boolean;
}) {
  return (
    <aside className="sidebar">
      <BrandLogo compact={false} />

      <div className="sidebar-section">
        <span className="sidebar-label">Организация</span>
        <SelectBox value={selectedOrgId} onChange={onSelectOrg} items={organizations} fallback="Нет организаций" />
        <SmallForm placeholder="Новая организация" button="+" onSubmit={onCreateOrg} />
      </div>

      <div className="sidebar-section">
        <span className="sidebar-label">Навигация</span>
        <nav className="side-nav">
          {sidebarNav.map((item) => (
            <button
              key={item.label}
              className={item.id === activeView ? "active" : ""}
              onClick={() => item.id !== "placeholder" && onSelectView(item.id)}
            >
              <item.icon size={18} />{item.label}
            </button>
          ))}
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
        <img src="/images/backgrounds/apiary-sidebar.svg" alt="Пасека" />
        <button className="support-button"><HelpCircle size={16} />Поддержка</button>
      </div>
    </aside>
  );
}

export function Topbar({
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

export function ApiariesScreen({
  organizationName,
  apiaries,
  summaries,
  selectedApiaryId,
  onSelectApiary,
  onOpenApiary,
  onCreateApiary,
  busy
}: {
  organizationName: string;
  apiaries: Apiary[];
  summaries: Record<string, ApiarySummary>;
  selectedApiaryId: string;
  onSelectApiary: (id: string) => void;
  onOpenApiary: (id: string) => void;
  onCreateApiary: (input: Partial<Apiary> & { name: string }) => void | Promise<void>;
  busy: boolean;
}) {
  const [formOpen, setFormOpen] = useState(false);
  const selectedApiary = apiaries.find((apiary) => apiary.id === selectedApiaryId);
  const selectedSummary = selectedApiary ? summaries[selectedApiary.id] : undefined;
  const totals = apiaries.reduce((acc, apiary) => {
    const summary = summaries[apiary.id];
    acc.hives += summary?.hiveCount ?? 0;
    acc.active += summary?.activeHiveCount ?? 0;
    acc.alerts += summary?.alertCount ?? 0;
    acc.unassigned += summary?.unassignedDeviceCount ?? 0;
    return acc;
  }, { hives: 0, active: 0, alerts: 0, unassigned: 0 });

  return (
    <section className="apiaries-screen">
      <div className="workspace-head">
        <div>
          <span className="eyebrow">Организация</span>
          <h2>Пасеки</h2>
          <p>{organizationName || "Организация не выбрана"}</p>
        </div>
        <button className="primary-action" type="button" onClick={() => setFormOpen(true)}><PackagePlus size={17} />Новая пасека</button>
      </div>

      <div className="apiary-summary-strip">
        <SummaryCard icon={<Map />} tone="blue" value={apiaries.length} label="Пасек" hint="В организации" />
        <SummaryCard icon={<Home />} tone="green" value={totals.hives} label="Ульев" hint={`${totals.active} активных`} />
        <SummaryCard icon={<AlertTriangle />} tone="red" value={totals.alerts} label="Алертов" hint="Требуют внимания" />
        <SummaryCard icon={<Radio />} tone="amber" value={totals.unassigned} label="Новых устройств" hint="Нужна привязка" />
      </div>

      <div className="apiaries-layout">
        <section className="panel-card apiaries-list-panel">
          <SectionHeader title="Список пасек" />
          <div className="apiaries-list">
            {apiaries.map((apiary) => (
              <ApiaryListRow
                key={apiary.id}
                apiary={apiary}
                summary={summaries[apiary.id]}
                selected={apiary.id === selectedApiaryId}
                onSelect={() => onSelectApiary(apiary.id)}
              />
            ))}
            {apiaries.length === 0 && <EmptyState title="Пасек пока нет" text="Создайте первую пасеку и укажите ее координаты." />}
          </div>
        </section>

        <section className="panel-card apiary-map-panel">
          <SectionHeader title="Карта пасек" info />
          <ApiaryMap apiaries={apiaries} selectedApiaryId={selectedApiaryId} onSelectApiary={onSelectApiary} />
        </section>

        <ApiarySelectionPanel apiary={selectedApiary} summary={selectedSummary} onOpenApiary={onOpenApiary} />
      </div>

      {formOpen && (
        <CreateApiaryModal
          busy={busy}
          onClose={() => setFormOpen(false)}
          onSubmit={async (input) => {
            await onCreateApiary(input);
            setFormOpen(false);
          }}
        />
      )}
    </section>
  );
}

function ApiaryListRow({ apiary, summary, selected, onSelect }: { apiary: Apiary; summary?: ApiarySummary; selected: boolean; onSelect: () => void }) {
  return (
    <button className={`apiary-row ${selected ? "selected" : ""}`} type="button" onClick={onSelect}>
      <span>
        <strong>{apiary.name}</strong>
        <small>{apiaryLocation(apiary)}</small>
      </span>
      <ApiaryStatusBadge status={summary?.status ?? "no_data"} />
      {summary?.unassignedDeviceCount ? <em>Нужна привязка</em> : <i />}
      <b>{summary?.hiveCount ?? 0}</b>
      <b>{summary?.activeHiveCount ?? 0}</b>
      <b>{summary?.alertCount ?? 0}</b>
    </button>
  );
}

function ApiarySelectionPanel({ apiary, summary, onOpenApiary }: { apiary?: Apiary; summary?: ApiarySummary; onOpenApiary: (id: string) => void }) {
  if (!apiary) {
    return (
      <aside className="panel-card apiary-detail-panel empty-detail">
        <EmptyState title="Выберите пасеку" text="Кликните пасеку в списке или на карте, чтобы увидеть сводку." />
      </aside>
    );
  }

  return (
    <aside className="panel-card apiary-detail-panel">
      <div className="apiary-detail-head">
        <div>
          <span className="eyebrow">Пасека</span>
          <h3>{apiary.name}</h3>
        </div>
        <ApiaryStatusBadge status={summary?.status ?? "no_data"} />
      </div>
      {summary?.unassignedDeviceCount ? <span className="bind-badge">Нужна привязка: {summary.unassignedDeviceCount}</span> : null}
      <div className="apiary-address">
        <strong>{apiaryAddress(apiary)}</strong>
        <span>{apiaryCoordinates(apiary)}</span>
      </div>
      <div className="apiary-detail-metrics">
        <ReadingBox label="Ульев" value={summary?.hiveCount ?? 0} />
        <ReadingBox label="Активные" value={summary?.activeHiveCount ?? 0} />
        <ReadingBox label="Алерты" value={summary?.alertCount ?? 0} />
        <ReadingBox label="События" value={summary?.eventCount ?? 0} />
      </div>
      <button className="primary-action" type="button" onClick={() => onOpenApiary(apiary.id)}>Перейти к пасеке <ArrowUpRight size={17} /></button>
    </aside>
  );
}

function CreateApiaryModal({ busy, onClose, onSubmit }: { busy: boolean; onClose: () => void; onSubmit: (input: Partial<Apiary> & { name: string }) => void | Promise<void> }) {
  const [form, setForm] = useState({
    name: "",
    description: "",
    country: "Украина",
    region: "",
    locality: "",
    address: "",
    location_description: "",
    latitude: "",
    longitude: "",
    timezone: "Europe/Kyiv"
  });
  const pickedPosition = coordinateFromForm(form.latitude, form.longitude);

  async function pickLocation(position: { lat: number; lng: number }) {
    setForm((state) => ({ ...state, latitude: position.lat.toFixed(6), longitude: position.lng.toFixed(6) }));
    const address = await reverseGeocode(position);
    if (!address) return;
    setForm((state) => ({
      ...state,
      country: address.country || state.country,
      region: address.region || state.region,
      locality: address.locality || state.locality,
      address: address.address || state.address
    }));
  }

  return (
    <div className="modal-backdrop" role="dialog" aria-modal="true">
      <form className="apiary-modal" onSubmit={(event) => {
        event.preventDefault();
        if (!form.name.trim()) return;
        void onSubmit({
          name: form.name.trim(),
          description: form.description,
          country: form.country,
          region: form.region,
          locality: form.locality,
          address: form.address,
          location_description: form.location_description,
          latitude: optionalFloat(form.latitude),
          longitude: optionalFloat(form.longitude),
          timezone: form.timezone
        });
      }}>
        <div className="modal-head">
          <div><span className="eyebrow">Новая пасека</span><h3>Создание пасеки</h3></div>
          <button className="icon-button" type="button" onClick={onClose} aria-label="Закрыть"><X size={18} /></button>
        </div>

        <div className="apiary-form-grid">
          <label>Название<input value={form.name} onChange={(event) => setForm({ ...form, name: event.target.value })} required /></label>
          <label>Часовой пояс<input value={form.timezone} onChange={(event) => setForm({ ...form, timezone: event.target.value })} /></label>
          <label className="wide">Описание<textarea value={form.description} onChange={(event) => setForm({ ...form, description: event.target.value })} /></label>
          <label>Страна<input value={form.country} onChange={(event) => setForm({ ...form, country: event.target.value })} /></label>
          <label>Область / регион<input value={form.region} onChange={(event) => setForm({ ...form, region: event.target.value })} /></label>
          <label>Населенный пункт<input value={form.locality} onChange={(event) => setForm({ ...form, locality: event.target.value })} /></label>
          <label>Адрес<input value={form.address} onChange={(event) => setForm({ ...form, address: event.target.value })} /></label>
          <label className="wide">Описание места<textarea value={form.location_description} onChange={(event) => setForm({ ...form, location_description: event.target.value })} /></label>
          <label>Широта<input inputMode="decimal" value={form.latitude} onChange={(event) => setForm({ ...form, latitude: event.target.value })} /></label>
          <label>Долгота<input inputMode="decimal" value={form.longitude} onChange={(event) => setForm({ ...form, longitude: event.target.value })} /></label>
        </div>

        <div className="modal-map">
          <ApiaryMap apiaries={[]} pickMode pickedPosition={pickedPosition} onPick={pickLocation} />
        </div>

        <div className="form-actions">
          <button type="submit" disabled={busy}>Создать пасеку</button>
          <button type="button" className="ghost" onClick={onClose}>Отмена</button>
        </div>
      </form>
    </div>
  );
}

export function SummaryPanel({ hives, devices, events, snapshots }: { hives: Hive[]; devices: Device[]; events: DeviceEvent[]; snapshots: HiveSnapshot }) {
  const active = hives.filter((hive) => effectiveHiveStatus(hive, snapshots[hive.id] ?? []) === "active").length;
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

export function SummaryCard({ icon, tone, value, label, hint }: { icon: ReactNode; tone: string; value: string | number; label: string; hint: string }) {
  return (
    <div className={`summary-card ${tone}`}>
      <span className="summary-icon">{icon}</span>
      <div><strong>{value}</strong><span>{label}</span><small>{hint}</small></div>
    </div>
  );
}

export function AlertsPanel({ events }: { events: DeviceEvent[] }) {
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

export function EventsPanel({ events, tasks = [], advice = [], onTaskStatus }: {
  events: DeviceEvent[];
  tasks?: ApiaryTask[];
  advice?: AdviceItem[];
  onTaskStatus?: (taskId: string, status: ApiaryTask["status"]) => void;
}) {
  const [activeTab, setActiveTab] = useState<EventTab>("important");
  const groups = groupEvents(events);
  const importantTasks = tasks.filter((task) => task.status === "due" || task.severity === "critical" || task.severity === "warning");
  const scheduledTasks = tasks.filter((task) => task.status === "planned" || task.status === "snoozed");
  const overdueTasks = tasks.filter((task) => task.status === "overdue");
  const seasonalAdvice = advice.filter((item) => item.source === "calendar_template" || item.source === "bloom");
  const visibleEvents = groups[activeTab].slice(0, 5);
  const taskItems: Record<EventTab, ApiaryTask[]> = {
    important: importantTasks,
    scheduled: scheduledTasks,
    overdue: overdueTasks,
    seasonal: []
  };
  const emptyText: Record<EventTab, string> = {
    important: "Событий пока нет",
    scheduled: "Запланированных событий пока нет",
    overdue: "Просроченных событий пока нет",
    seasonal: "Сезонных рекомендаций пока нет"
  };
  const activeTasks = taskItems[activeTab].slice(0, 5);
  const activeAdvice = activeTab === "seasonal" ? seasonalAdvice.slice(0, 5) : [];

  return (
    <section className="panel-card list-panel">
      <SectionHeader title="События" />
      <div className="tabs" role="tablist" aria-label="Фильтр событий">
        <EventTabButton active={activeTab === "important"} count={groups.important.length + importantTasks.length} onClick={() => setActiveTab("important")}>Важные</EventTabButton>
        <EventTabButton active={activeTab === "scheduled"} count={groups.scheduled.length + scheduledTasks.length} onClick={() => setActiveTab("scheduled")}>Запланированные</EventTabButton>
        <EventTabButton active={activeTab === "overdue"} count={groups.overdue.length + overdueTasks.length} onClick={() => setActiveTab("overdue")}>Просроченные</EventTabButton>
        <EventTabButton active={activeTab === "seasonal"} count={seasonalAdvice.length} onClick={() => setActiveTab("seasonal")}>Сезонные</EventTabButton>
      </div>
      {activeTasks.map((task) => <TaskLine key={task.id} task={task} onTaskStatus={onTaskStatus} />)}
      {activeAdvice.map((item) => <AdviceEventLine key={item.code} advice={item} />)}
      {activeTab !== "seasonal" && visibleEvents.map((event) => <EventLine key={event.id} event={event} />)}
      {activeTasks.length + activeAdvice.length + (activeTab === "seasonal" ? 0 : visibleEvents.length) === 0 && <EmptyInline text={emptyText[activeTab]} />}
      <button className="text-link">Все события</button>
    </section>
  );
}

function EventTabButton({ active, count, onClick, children }: { active: boolean; count: number; onClick: () => void; children: ReactNode }) {
  return (
    <button className={active ? "active" : ""} role="tab" aria-selected={active} type="button" onClick={onClick}>
      {children} <b>{count}</b>
    </button>
  );
}

export function DevicesPanel({ devices, hives, busy, onAssign }: {
  devices: Device[];
  hives: Hive[];
  busy: boolean;
  onAssign: (deviceId: string, hiveId: string, importMode: string, replaceExisting: boolean) => void | Promise<void>;
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
    </section>
  );
}

export function QuickAssign({ device, hives, busy, onAssign }: {
  device: Device;
  hives: Hive[];
  busy: boolean;
  onAssign: (deviceId: string, hiveId: string, importMode: string, replaceExisting: boolean) => void | Promise<void>;
}) {
  const [open, setOpen] = useState(false);

  return (
    <>
      <button className="small-outline" type="button" disabled={busy || hives.length === 0} onClick={() => setOpen(true)}>
        Привязать
      </button>
      {open ? (
        <AssignDeviceModal
          device={device}
          hives={hives}
          busy={busy}
          onClose={() => setOpen(false)}
          onAssign={async (hiveId, importMode, replaceExisting) => {
            await onAssign(device.id, hiveId, importMode, replaceExisting);
            setOpen(false);
          }}
        />
      ) : null}
    </>
  );
}

function AssignDeviceModal({
  device,
  hives,
  busy,
  onClose,
  onAssign
}: {
  device: Device;
  hives: Hive[];
  busy: boolean;
  onClose: () => void;
  onAssign: (hiveId: string, importMode: string, replaceExisting: boolean) => void | Promise<void>;
}) {
  const [hiveId, setHiveId] = useState("");
  const [importMode, setImportMode] = useState("attach_to_hive");
  const [assignmentMode, setAssignmentMode] = useState<"add" | "replace">("add");
  const selectedHive = hives.find((hive) => hive.id === hiveId);
  const selectedHiveHasDevice = Boolean(selectedHive?.assigned_device_id);
  const selectedHiveDeviceCount = selectedHive?.assigned_device_count ?? 0;
  const assignedDeviceLabel = selectedHive?.assigned_device_public_id || selectedHive?.assigned_device_id || "Устройство";
  const assignedDeviceType = selectedHive?.assigned_device_type || "HiveMonitor device";

  function submit(event: FormEvent<HTMLFormElement>) {
    event.preventDefault();
    if (!hiveId) return;
    void onAssign(hiveId, importMode, selectedHiveHasDevice && assignmentMode === "replace");
  }

  return (
    <div className="modal-backdrop" role="dialog" aria-modal="true" aria-labelledby="assign-device-title">
      <form className="assign-modal" onSubmit={submit}>
        <div className="modal-head">
          <div>
            <span className="eyebrow">Привязка устройства</span>
            <h3 id="assign-device-title">Выберите улей для устройства</h3>
          </div>
          <button className="icon-button" type="button" onClick={onClose} aria-label="Закрыть"><X size={18} /></button>
        </div>

        <div className="assign-device-card">
          <Radio size={22} />
          <div>
            <strong>{device.device_id}</strong>
            <span>{device.device_type || "HiveMonitor device"}</span>
          </div>
          <em>{device.status}</em>
        </div>

        <div className="assign-metrics">
          <ReadingBox label="Последняя телеметрия" value={formatDeviceDate(device.last_telemetry_at)} />
          <ReadingBox label="Интервал" value={`${device.telemetry_interval_minutes || 30} мин`} />
          <ReadingBox label="Пропусков" value={device.missed_telemetry_count ?? 0} />
        </div>

        <label className="field-label">
          Улей
          <select value={hiveId} onChange={(event) => {
            setHiveId(event.target.value);
            setAssignmentMode("add");
          }} required>
            <option value="" disabled>Выберите улей</option>
            {hives.map((hive) => (
              <option key={hive.id} value={hive.id}>
                {hive.number ? `Улей ${hive.number}` : hive.name} · {hive.type || "тип не указан"} · {deviceName(hive, [])}
              </option>
            ))}
          </select>
        </label>

        {selectedHive ? (
          <div className="assign-selected-hive">
            <strong>{selectedHive.number ? `Улей ${selectedHive.number}` : selectedHive.name}</strong>
            <span>{selectedHive.type || "Тип не указан"} · статус: {hiveStatusText(selectedHive.status)}</span>
          </div>
        ) : null}

        {selectedHiveHasDevice ? (
          <fieldset className="assign-history-options warning">
            <legend>В этом улье уже есть устройство</legend>
            <div className="assign-current-device">
              <Radio size={18} />
              <div>
                <strong>{assignedDeviceLabel}</strong>
                <span>
                  {assignedDeviceType}
                  {selectedHiveDeviceCount > 1 ? ` · всего устройств: ${selectedHiveDeviceCount}` : ""}
                </span>
              </div>
            </div>
            <label>
              <input type="radio" name="assignmentMode" value="add" checked={assignmentMode === "add"} onChange={() => setAssignmentMode("add")} />
              <span><strong>Добавить как дополнительное устройство</strong><small>Оставит текущую привязку и добавит новое устройство к этому улью.</small></span>
            </label>
            <label>
              <input type="radio" name="assignmentMode" value="replace" checked={assignmentMode === "replace"} onChange={() => setAssignmentMode("replace")} />
              <span><strong>Заменить текущее устройство</strong><small>Старое устройство будет отвязано от улья, новое станет активным мониторингом.</small></span>
            </label>
          </fieldset>
        ) : null}

        <fieldset className="assign-history-options">
          <legend>Что сделать со старой непривязанной телеметрией</legend>
          <label>
            <input type="radio" name="importMode" value="attach_to_hive" checked={importMode === "attach_to_hive"} onChange={(event) => setImportMode(event.target.value)} />
            <span><strong>Привязать к выбранному улью</strong><small>История устройства станет частью графиков и журнала улья.</small></span>
          </label>
          <label>
            <input type="radio" name="importMode" value="keep" checked={importMode === "keep"} onChange={(event) => setImportMode(event.target.value)} />
            <span><strong>Оставить непривязанной</strong><small>Новые данные пойдут в улей, старая история останется отдельно.</small></span>
          </label>
          <label>
            <input type="radio" name="importMode" value="delete" checked={importMode === "delete"} onChange={(event) => setImportMode(event.target.value)} />
            <span><strong>Удалить старые данные</strong><small>Подходит, если устройство тестировалось или стояло на другом улье.</small></span>
          </label>
        </fieldset>

        <div className="form-actions">
          <button type="submit" disabled={busy || !hiveId}>Привязать устройство</button>
          <button type="button" className="ghost" onClick={onClose}>Отмена</button>
        </div>
      </form>
    </div>
  );
}

export function HivesTable({
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
        <div className="table-actions"><HiveTabs hives={hives} snapshots={snapshots} /><CreateHiveForm disabled={busy} onSubmit={onCreateHive} /><button className="small-outline"><Filter size={15} />Фильтры</button></div>
      </div>
      <div className="hive-table">
        <div className="hive-table-head">
          <span>Улей</span><span>Тип улья</span><span>Статус</span><span>Изменение веса за сутки</span><span>Вес, кг</span><span>Температура</span><span>Алармы</span><span>Устройство</span><span>Обновлено</span>
        </div>
        {hives.map((hive) => {
          const readings = snapshots[hive.id] ?? [];
          const indexed = indexReadings(readings);
          const weight = indexed.weight;
          const temp = indexed.temperature;
          const delta = indexed.weight_change;
          return (
            <button key={hive.id} className={`hive-table-row ${hive.id === selectedHiveId ? "selected" : ""}`} onClick={() => onSelectHive(hive.id)}>
              <span className="hive-name-cell"><ChevronDown size={14} /><b>{hive.number || shortId(hive.id)}</b><em>{hive.name || "Без названия"}</em></span>
              <span>{hive.type || "Дадан"}{hive.super_count ? " с магазином" : ""}</span>
              <StatusPill status={effectiveHiveStatus(hive, readings)} />
              <WeightDelta reading={delta} />
              <span>{weight ? valueOnly(weight) : "-"}</span>
              <span>{temp ? formatValue(temp) : "-"}</span>
              <span>{hive.status === "attention" ? <span className="alarm-pill">Слабый прирост</span> : "-"}</span>
              <span className={`device-signal ${hive.assigned_device_id ? "" : "muted"}`}><Radio size={14} />{deviceName(hive, readings)}</span>
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

export function HiveTabs({ hives, snapshots }: { hives: Hive[]; snapshots: HiveSnapshot }) {
  const active = hives.filter((hive) => effectiveHiveStatus(hive, snapshots[hive.id] ?? []) === "active").length;
  const attention = hives.filter((hive) => hive.status === "attention").length;
  return <div className="hive-tabs"><span>Все ({hives.length})</span><span>Активные ({active})</span><span>С алармами ({attention})</span></div>;
}

export function CreateHiveForm({ disabled, onSubmit }: { disabled: boolean; onSubmit: (input: Partial<Hive> & { name: string }) => void }) {
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
        status: "no_device",
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

export function ComparisonPanel({
  hives,
  histories,
  snapshots,
  selectedHiveId,
  period,
  onPeriodChange,
  onSelectHive
}: {
  hives: Hive[];
  histories: HiveHistory;
  snapshots: HiveSnapshot;
  selectedHiveId: string;
  period: ChartPeriod;
  onPeriodChange: (period: ChartPeriod) => void;
  onSelectHive: (id: string) => void;
}) {
  return (
    <section className="panel-card comparison-panel">
      <div className="chart-heading">
        <SectionHeader title="Сравнительный график изменения веса ульев по пасеке" info />
        <ChartPeriodButtons period={period} onChange={onPeriodChange} />
      </div>
      <ComparisonChart
        hives={hives}
        histories={histories}
        snapshots={snapshots}
        selectedHiveId={selectedHiveId}
        period={period}
        onSelectHive={onSelectHive}
      />
    </section>
  );
}

export function HiveDetail({ hive, state, latestByMetric, onLoadHistory, onClose }: {
  hive: Hive;
  state: HiveState;
  latestByMetric: Record<string, SensorReading>;
  onLoadHistory: (period: ChartPeriod) => Promise<void>;
  onClose: () => void;
}) {
  const [activeTab, setActiveTab] = useState<DetailTab>("overview");
  const [chartPeriod, setChartPeriod] = useState<ChartPeriod>("1d");
  const alertEvents = state.events.filter(isAlertEvent);

  useEffect(() => {
    setActiveTab("overview");
    setChartPeriod("1d");
  }, [hive.id]);

  function changeChartPeriod(period: ChartPeriod) {
    setChartPeriod(period);
    void onLoadHistory(period);
  }

  return (
    <aside className="hive-detail">
      <div className="detail-head">
        <div><span className="back-arrow">←</span><h2>{hive.name}</h2><StatusPill status={effectiveHiveStatus(hive, state.latest)} /></div>
        <button className="icon-button" onClick={onClose} aria-label="Закрыть карточку улья"><span>×</span></button>
      </div>
      <div className="detail-tabs" role="tablist" aria-label="Карточка улья">
        <DetailTabButton active={activeTab === "overview"} onClick={() => setActiveTab("overview")}>Обзор</DetailTabButton>
        <DetailTabButton active={activeTab === "charts"} onClick={() => setActiveTab("charts")}>Графики</DetailTabButton>
        <DetailTabButton active={activeTab === "history"} onClick={() => setActiveTab("history")}>История</DetailTabButton>
      </div>

      {activeTab === "overview" && (
        <>
          {alertEvents.length > 0 ? (
            <div className="warning-box">
              <AlertTriangle size={17} />
              Есть предупреждения
              <button type="button" onClick={() => setActiveTab("history")}>Все аларты ({alertEvents.length})</button>
            </div>
          ) : null}
          <div className="hive-media">
            <HiveImage hive={hive} />
            <div>
              <span>Тип улья</span><strong>{hive.type || "Дадан"}{hive.super_count ? " с магазином" : ""}</strong>
              <span>Устройство</span><strong>{deviceName(hive, state.latest)}</strong>
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
        </>
      )}

      {activeTab === "charts" && (
        <section className="detail-chart">
          <div className="chart-heading detail-chart-heading">
            <SectionHeader title={`Изменение веса за ${chartPeriodLabels[chartPeriod]}`} />
            <ChartPeriodButtons period={chartPeriod} onChange={changeChartPeriod} />
          </div>
          <TelemetryChart readings={state.history} compact label={hive.name} hives={[hive]} emptyText={`История за ${chartPeriodLabels[chartPeriod]} пока пуста`} />
        </section>
      )}

      {activeTab === "history" && (
        <section className="detail-events">
          <SectionHeader title="Последние события" action="Все события" />
          {state.events.slice(0, 8).map((event) => <EventLine key={event.id} event={event} />)}
          {state.events.length === 0 && <EmptyInline text="Событий пока нет" />}
        </section>
      )}

      <div className="detail-actions">
        <button className="ghost" type="button" onClick={() => { setActiveTab("charts"); void onLoadHistory(chartPeriod); }} disabled={state.loading}><BarChart3 size={17} />Открыть графики</button>
        <button className="primary-action" type="button" onClick={() => setActiveTab("overview")}>Перейти к улью <ArrowUpRight size={17} /></button>
      </div>
    </aside>
  );
}

function DetailTabButton({ active, onClick, children }: { active: boolean; onClick: () => void; children: ReactNode }) {
  return (
    <button className={active ? "active" : ""} role="tab" aria-selected={active} type="button" onClick={onClick}>
      {children}
    </button>
  );
}

function ChartPeriodButtons({ period, onChange }: { period: ChartPeriod; onChange: (period: ChartPeriod) => void }) {
  return (
    <div className="range-buttons">
      {(["1d", "10d", "30d"] as ChartPeriod[]).map((item) => (
        <button key={item} type="button" className={period === item ? "active" : ""} onClick={() => onChange(item)}>
          {chartPeriodLabels[item]}
        </button>
      ))}
    </div>
  );
}

export function ReadingMini({ title, reading, accent }: { title: string; reading?: SensorReading; accent?: boolean }) {
  return (
    <div className="reading-mini">
      <span>{title}</span>
      <strong className={accent ? "positive" : ""}>{reading ? formatValue(reading) : "-"}</strong>
      {reading?.metric_type === "rssi" && <small>Отличный</small>}
      {reading?.metric_type === "battery_percent" && <small>{readingValue([reading], "battery_voltage")?.toFixed(1) ?? "3.7"} V</small>}
    </div>
  );
}

export function WeatherCard({ apiary }: { apiary?: Apiary }) {
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

export function TipsCard({ advice = [], events, showAll = false, onAdviceState, onToggleAll }: {
  advice?: AdviceItem[];
  events: DeviceEvent[];
  showAll?: boolean;
  onAdviceState?: (code: string, status: "dismissed" | "snoozed") => void;
  onToggleAll?: () => void;
}) {
  const attention = events.find((event) => event.ok === false);
  const fallbackAdvice: AdviceItem[] = [
    {
      id: "fallback_sunflower",
      code: "fallback_sunflower",
      title: "Скоро цветение подсолнечника",
      body: "Проверьте сильные семьи и не забудьте добавить магазины.",
      category: "bloom",
      severity: "notice",
      priority: 100,
      source: "mock",
      action_label: "Подробнее",
      action_type: "details",
      is_user_dismissible: false
    },
    {
      id: "fallback_weather",
      code: "fallback_weather",
      title: "Ожидается похолодание",
      body: "Обратите внимание на слабые семьи и утепление.",
      category: "weather",
      severity: "warning",
      priority: 110,
      source: "mock",
      action_label: "Подробнее",
      action_type: "details",
      is_user_dismissible: false
    },
    {
      id: "fallback_attention",
      code: "fallback_attention",
      title: "Проверьте ульи",
      body: attention?.message ?? "Проверьте ульи с низким зарядом устройства.",
      category: "telemetry",
      severity: attention ? "warning" : "info",
      priority: 120,
      source: "mock",
      action_label: "Подробнее",
      action_type: "details",
      is_user_dismissible: false
    }
  ];
  const items = (advice.length > 0 ? advice : fallbackAdvice).slice(0, 5);
  return (
    <section className="panel-card tips-card">
      <SectionHeader title="Советы / Подсказки" />
      {items.map((item) => (
        <TipLine
          key={item.code}
          advice={item}
          onDismiss={item.is_user_dismissible ? () => onAdviceState?.(item.code, "dismissed") : undefined}
          onSnooze={item.is_user_dismissible ? () => onAdviceState?.(item.code, "snoozed") : undefined}
        />
      ))}
      <button className="text-link" type="button" onClick={onToggleAll}>{showAll ? "Только актуальные" : "Все советы"}</button>
    </section>
  );
}

export function TipLine({ advice, onDismiss, onSnooze }: { advice: AdviceItem; onDismiss?: () => void; onSnooze?: () => void }) {
  const [open, setOpen] = useState(false);
  return (
    <div className={`tip-line severity-${advice.severity} ${advice.state ? `state-${advice.state}` : ""}`}>
      <button type="button" className="tip-toggle" onClick={() => setOpen(!open)} aria-expanded={open}>
        <span>{tipIcon(advice)}</span>
        <div>
          <strong>{advice.title}</strong>
          {advice.state === "dismissed" && <small>Скрыто</small>}
          {advice.state === "snoozed" && <small>Отложено</small>}
          <p>{open ? advice.body : advice.body.slice(0, 78)}</p>
        </div>
        <ChevronDown size={16} />
      </button>
      {open && (
        <div className="inline-actions tip-actions">
          {advice.action_label && <button type="button">{advice.action_label}</button>}
          {onSnooze && advice.state !== "snoozed" && <button type="button" onClick={onSnooze}>Отложить</button>}
          {onDismiss && advice.state !== "dismissed" && <button type="button" onClick={onDismiss}>Скрыть</button>}
        </div>
      )}
    </div>
  );
}

function tipIcon(advice: AdviceItem) {
  if (advice.category === "weather") return <CloudSun />;
  if (advice.category === "telemetry") return <AlertTriangle />;
  if (advice.category === "bloom") return <Zap />;
  if (advice.category === "wintering" || advice.category === "feeding") return <Thermometer />;
  return <ListChecks />;
}

export function TelemetryChart({ readings, label, hives, compact = false, emptyText = "История пока пуста" }: { readings: SensorReading[]; label: string; hives: Hive[]; compact?: boolean; emptyText?: string }) {
  if (readings.length === 0) return <div className="chart-empty">{emptyText}</div>;
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

export function ComparisonChart({
  hives,
  histories,
  snapshots,
  selectedHiveId,
  period,
  onSelectHive
}: {
  hives: Hive[];
  histories: HiveHistory;
  snapshots: HiveSnapshot;
  selectedHiveId: string;
  period: ChartPeriod;
  onSelectHive: (id: string) => void;
}) {
  const [hoveredHiveId, setHoveredHiveId] = useState("");
  const series = hives
    .map((hive, index) => {
      const readings = histories[hive.id] ?? [];
      const points = deltaPoints(readings);
      return {
        hive,
        readings,
        points,
        color: chartColors[index % chartColors.length],
        latest: indexReadings(snapshots[hive.id] ?? [])
      };
    })
    .filter((item) => item.readings.length > 0);

  if (series.length === 0) return <div className="chart-empty">История за {chartPeriodLabels[period]} пока пуста</div>;

  const activeHiveId = hoveredHiveId || selectedHiveId;
  const activeSeries = series.find((item) => item.hive.id === activeHiveId);
  const allValues = series.flatMap((item) => item.points.map((point) => point.deltaKg));
  const min = Math.min(...allValues);
  const max = Math.max(...allValues);
  const range = max - min || 1;

  return (
    <div className="chart comparison-chart">
      <svg viewBox="0 0 100 100" preserveAspectRatio="none" role="img" aria-label="Сравнительный график веса ульев">
        {[20, 40, 60, 80].map((y) => <line key={y} x1="0" x2="100" y1={y} y2={y} className="grid-line" />)}
        {series.map((item) => (
          <g key={item.hive.id}>
            <polyline
              points={deltaLinePoints(item.points, min, range)}
              className={`series-line ${selectedHiveId && selectedHiveId !== item.hive.id ? "dimmed" : ""} ${selectedHiveId === item.hive.id ? "selected" : ""} ${hoveredHiveId === item.hive.id ? "hovered" : ""}`}
              style={{ stroke: item.color }}
              vectorEffect="non-scaling-stroke"
            />
            <polyline
              points={deltaLinePoints(item.points, min, range)}
              className="series-hitarea"
              vectorEffect="non-scaling-stroke"
              onMouseEnter={() => setHoveredHiveId(item.hive.id)}
              onMouseLeave={() => setHoveredHiveId("")}
              onClick={() => onSelectHive(item.hive.id)}
            />
          </g>
        ))}
      </svg>
      {activeSeries && <ChartTooltip series={activeSeries} />}
      <div className="multi-legend">
        {series.map((item) => (
          <button
            key={item.hive.id}
            className={selectedHiveId === item.hive.id ? "active" : ""}
            onMouseEnter={() => setHoveredHiveId(item.hive.id)}
            onMouseLeave={() => setHoveredHiveId("")}
            onClick={() => onSelectHive(item.hive.id)}
          >
            <i style={{ background: item.color }} />{item.hive.name}
          </button>
        ))}
      </div>
    </div>
  );
}

export function ChartTooltip({ series }: {
  series: {
    hive: Hive;
    readings: SensorReading[];
    points: Array<{ reading: SensorReading; deltaKg: number }>;
    latest: Record<string, SensorReading>;
  };
}) {
  const lastPoint = series.points[series.points.length - 1];
  const currentWeight = series.latest.weight ?? series.readings[series.readings.length - 1];
  const temperature = series.latest.temperature;

  return (
    <div className="chart-tooltip">
      <strong>{series.hive.name}</strong>
      <span>Тип: {series.hive.type || "Не задан"}</span>
      <span>Изменение: {formatSigned(lastPoint.deltaKg, " кг")}</span>
      <span>Вес: {currentWeight ? formatValue(currentWeight) : "-"}</span>
      <span>Температура: {temperature ? formatValue(temperature) : "-"}</span>
      <span>Статус: {series.hive.status === "attention" ? "Внимание" : "Активен"}</span>
      <time>{formatDate(lastPoint.reading.measured_at)}</time>
    </div>
  );
}

export function SectionHeader({ title, action, info }: { title: string; action?: string; info?: boolean }) {
  return (
    <div className="section-header">
      <h3>{title}{info && <HelpCircle size={14} />}</h3>
      {action && <a>{action}</a>}
    </div>
  );
}

export function EventLine({ event, compact = false }: { event: DeviceEvent; compact?: boolean }) {
  return (
    <div className={`event-line ${compact ? "compact" : ""}`}>
      <EventIcon event={event} />
      <div><strong>{eventTitle(event)}</strong><span>{event.message || event.command || shortId(event.id)}</span></div>
      <time>{formatDate(event.occurred_at)}</time>
    </div>
  );
}

export function TaskLine({ task, onTaskStatus }: { task: ApiaryTask; onTaskStatus?: (taskId: string, status: ApiaryTask["status"]) => void }) {
  return (
    <div className="event-line task-line">
      <SeverityIcon severity={task.severity} />
      <div>
        <strong>{task.title}</strong>
        <span>{task.description || taskStatusLabel(task.status)}</span>
        <div className="inline-actions">
          {task.status !== "completed" && <button type="button" onClick={() => onTaskStatus?.(task.id, "completed")}>Выполнено</button>}
          {task.status !== "snoozed" && <button type="button" onClick={() => onTaskStatus?.(task.id, "snoozed")}>Отложить</button>}
          {task.status !== "dismissed" && <button type="button" onClick={() => onTaskStatus?.(task.id, "dismissed")}>Скрыть</button>}
        </div>
      </div>
      <time>{task.due_at ? formatDate(task.due_at) : taskStatusLabel(task.status)}</time>
    </div>
  );
}

export function AdviceEventLine({ advice }: { advice: AdviceItem }) {
  return (
    <div className="event-line task-line">
      <SeverityIcon severity={advice.severity} />
      <div><strong>{advice.title}</strong><span>{advice.body}</span></div>
      <time>{advice.category}</time>
    </div>
  );
}

export function EventIcon({ event }: { event: DeviceEvent }) {
  if (event.ok === false || event.event_type === "alert") return <span className="event-icon red"><AlertTriangle size={15} /></span>;
  if (event.event_type === "hive_opened") return <span className="event-icon amber"><Thermometer size={15} /></span>;
  return <span className="event-icon green"><CheckCircle2 size={15} /></span>;
}

export function SeverityIcon({ severity }: { severity: string }) {
  if (severity === "critical" || severity === "warning") return <span className="event-icon red"><AlertTriangle size={15} /></span>;
  if (severity === "notice") return <span className="event-icon amber"><CalendarDays size={15} /></span>;
  return <span className="event-icon green"><CheckCircle2 size={15} /></span>;
}

export function StatusPill({ status }: { status: string }) {
  if (status === "no_device") return <span className="status-pill no-device">Нет устройства</span>;
  if (status === "no_data") return <span className="status-pill no-data">Нет данных</span>;
  const offline = status === "offline" || status === "no_connection";
  const attention = status === "attention";
  return <span className={`status-pill ${offline ? "offline" : attention ? "attention" : ""}`}>{offline ? "Нет связи" : attention ? "Внимание" : "Активен"}</span>;
}

export function WeightDelta({ reading }: { reading?: SensorReading }) {
  if (!reading) return <span>-</span>;
  const positive = reading.value >= 0;
  return <span className={`weight-delta ${positive ? "positive" : "negative"}`}>{formatSigned(reading.value, " кг")} <ArrowUpRight size={14} /></span>;
}

export function SelectBox<T extends { id: string; name: string }>({ value, onChange, items, fallback }: { value: string; onChange: (id: string) => void; items: T[]; fallback: string }) {
  return (
    <label className="select-box">
      <select value={value} onChange={(event) => onChange(event.target.value)} disabled={items.length === 0}>
        {items.length === 0 ? <option>{fallback}</option> : items.map((item) => <option key={item.id} value={item.id}>{item.name}</option>)}
      </select>
      <ChevronDown size={15} />
    </label>
  );
}

export function SmallForm({ placeholder, button, onSubmit }: { placeholder: string; button: string; onSubmit: (value: string) => void }) {
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

export function CreateApiaryForm({ disabled, onSubmit }: { disabled: boolean; onSubmit: (input: Partial<Apiary> & { name: string }) => void }) {
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

export function BrandLogo({ compact }: { compact: boolean }) {
  return (
    <div className="brand">
      <img
        src={compact ? "/images/brand/hivemonitor-mark.svg" : "/images/brand/hivemonitor-logo.svg"}
        alt="HiveMonitor"
      />
    </div>
  );
}

export function HiveImage({ hive }: { hive: Hive }) {
  const [src, setSrc] = useState(() => getHiveImageByType(`${hive.type} ${hive.super_count ? "магазин" : ""}`));

  useEffect(() => {
    setSrc(getHiveImageByType(`${hive.type} ${hive.super_count ? "магазин" : ""}`));
  }, [hive.type, hive.super_count]);

  return (
    <img
      className="hive-drawer-image"
      src={src}
      alt={`Улей ${hive.number || hive.name}`}
      onError={(event) => {
        if (event.currentTarget.src.endsWith("/images/hives/hive-generic.svg")) return;
        setSrc(getHiveFallbackImageByType(`${hive.type} ${hive.super_count ? "магазин" : ""}`));
      }}
    />
  );
}

export function EmptyState({ title, text }: { title: string; text: string }) {
  return <div className="empty"><strong>{title}</strong><span>{text}</span></div>;
}

export function EmptyInline({ text }: { text: string }) {
  return <p className="empty-inline">{text}</p>;
}

function ReadingBox({ label, value }: { label: string; value: string | number }) {
  return <div className="reading-mini"><span>{label}</span><strong>{value}</strong></div>;
}

function ApiaryStatusBadge({ status }: { status: ApiarySummary["status"] }) {
  const labels: Record<ApiarySummary["status"], string> = {
    normal: "Норма",
    attention: "Внимание",
    no_data: "Нет данных"
  };
  return <span className={`apiary-status ${status}`}>{labels[status]}</span>;
}

function apiaryLocation(apiary: Apiary) {
  return [apiary.region, apiary.locality].filter(Boolean).join(" · ") || apiary.country || "Локация не указана";
}

function apiaryAddress(apiary: Apiary) {
  return [apiary.address, apiary.locality, apiary.region].filter(Boolean).join(", ") || apiary.location_description || "Адрес не указан";
}

function apiaryCoordinates(apiary: Apiary) {
  if (typeof apiary.latitude !== "number" || typeof apiary.longitude !== "number") return "Координаты не указаны";
  return `${apiary.latitude.toFixed(5)}, ${apiary.longitude.toFixed(5)}`;
}

function coordinateFromForm(latitude: string, longitude: string) {
  const lat = Number.parseFloat(latitude.replace(",", "."));
  const lng = Number.parseFloat(longitude.replace(",", "."));
  if (!Number.isFinite(lat) || !Number.isFinite(lng)) return null;
  return { lat, lng };
}

async function reverseGeocode(position: { lat: number; lng: number }) {
  try {
    const params = new URLSearchParams({
      format: "jsonv2",
      lat: String(position.lat),
      lon: String(position.lng),
      addressdetails: "1",
      "accept-language": "ru"
    });
    const response = await fetch(`https://nominatim.openstreetmap.org/reverse?${params.toString()}`);
    if (!response.ok) return null;
    const data = await response.json() as { address?: Record<string, string>; display_name?: string };
    const address = data.address ?? {};
    return {
      country: address.country ?? "",
      region: address.state ?? address.region ?? address.county ?? "",
      locality: address.city ?? address.town ?? address.village ?? address.hamlet ?? "",
      address: [address.road, address.house_number].filter(Boolean).join(" ") || data.display_name || ""
    };
  } catch {
    return null;
  }
}

function groupEvents(events: DeviceEvent[]): Record<EventTab, DeviceEvent[]> {
  const now = Date.now();

  return events.reduce<Record<EventTab, DeviceEvent[]>>((groups, event) => {
    if (isScheduledEvent(event)) {
      const time = new Date(event.occurred_at).getTime();
      groups[time < now ? "overdue" : "scheduled"].push(event);
      return groups;
    }

    groups.important.push(event);
    return groups;
  }, { important: [], scheduled: [], overdue: [], seasonal: [] });
}

function isScheduledEvent(event: DeviceEvent) {
  return event.event_type === "task" || event.event_type === "scheduled_task" || event.event_type === "reminder";
}

function isAlertEvent(event: DeviceEvent) {
  return event.ok === false || event.event_type === "alert" || event.event_type === "warning";
}

export async function refreshAll(client: ApiClient, setters: {
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

export function indexReadings(readings: SensorReading[]) {
  return readings.reduce<Record<string, SensorReading>>((acc, reading) => {
    acc[reading.metric_type] = reading;
    return acc;
  }, {});
}

export function readingValue(readings: SensorReading[] | undefined, metric: string) {
  const reading = readings?.find((item) => item.metric_type === metric);
  return reading?.value;
}

export function isNumber(value: unknown): value is number {
  return typeof value === "number" && Number.isFinite(value);
}

export function average(values: number[]) {
  if (values.length === 0) return 0;
  return values.reduce((sum, value) => sum + value, 0) / values.length;
}

export function percent(value: number, total: number) {
  if (total === 0) return 0;
  return Math.round((value / total) * 100);
}

export function optionalInt(value: string) {
  const parsed = Number.parseInt(value, 10);
  return Number.isFinite(parsed) ? parsed : undefined;
}

export function optionalFloat(value: string) {
  const parsed = Number.parseFloat(value.replace(",", "."));
  return Number.isFinite(parsed) ? parsed : undefined;
}

export function formatValue(reading: SensorReading) {
  if (reading.metric_type === "hive_opened") return reading.value > 0 ? "открыт" : "закрыт";
  const value = valueOnly(reading);
  return `${value}${reading.unit ? ` ${reading.unit}` : ""}`;
}

export function valueOnly(reading: SensorReading) {
  return Math.abs(reading.value) >= 100 ? reading.value.toFixed(0) : reading.value.toFixed(2).replace(".", ",");
}

export function formatSigned(value: number, unit: string) {
  const sign = value > 0 ? "+" : "";
  return `${sign}${value.toFixed(2).replace(".", ",")}${unit}`;
}

export function formatDate(value?: string) {
  if (!value) return "";
  return new Intl.DateTimeFormat("ru", { hour: "2-digit", minute: "2-digit" }).format(new Date(value));
}

function formatDeviceDate(value?: string) {
  if (!value) return "нет данных";
  return new Intl.DateTimeFormat("ru", { day: "2-digit", month: "2-digit", hour: "2-digit", minute: "2-digit" }).format(new Date(value));
}

function hiveStatusText(status?: string) {
  if (status === "attention") return "Внимание";
  if (status === "offline") return "Нет связи";
  if (status === "inactive") return "Неактивен";
  return "Активен";
}

export function isToday(value?: string) {
  if (!value) return false;
  const date = new Date(value);
  const now = new Date();
  return date.toDateString() === now.toDateString();
}

export function shortId(value: string) {
  return value.slice(0, 8);
}

export function initials(value: string) {
  return value.split(/[ @._-]+/).filter(Boolean).slice(0, 2).map((part) => part[0]?.toUpperCase()).join("") || "П";
}

function deviceName(hive: Hive, readings: SensorReading[]) {
  if (hive.assigned_device_public_id) return hive.assigned_device_public_id;
  if (hive.assigned_device_id) return `HM-${shortId(hive.assigned_device_id).toUpperCase()}`;
  const id = readings[0]?.device_id;
  return id ? `HM-${shortId(id).toUpperCase()}` : "Нет устройства";
}

function effectiveHiveStatus(hive: Hive, readings: SensorReading[]) {
  if (hive.status === "attention") return "attention";
  if (!hive.assigned_device_id) return "no_device";
  if (readings.length === 0) return "no_data";
  if (hive.status === "offline" || hive.status === "no_connection") return hive.status;
  return "active";
}

export function getHiveImageByType(type?: string | null): string {
  const normalized = (type || "").toLowerCase();

  if (normalized.includes("дадан") && normalized.includes("магаз")) {
    return "/images/hives/hive-dadan-with-super.webp";
  }
  if (normalized.includes("дадан")) return "/images/hives/hive-dadan.webp";
  if (normalized.includes("рута")) return "/images/hives/hive-ruta.webp";
  if (normalized.includes("лежак")) return "/images/hives/hive-layens.webp";
  if (normalized.includes("украин")) return "/images/hives/hive-ukrainian.webp";
  if (normalized.includes("колод")) return "/images/hives/hive-log-hive.webp";

  return "/images/hives/hive-generic.webp";
}

export function getHiveFallbackImageByType(type?: string | null): string {
  const normalized = (type || "").toLowerCase();

  if (normalized.includes("дадан") && normalized.includes("магаз")) {
    return "/images/hives/hive-dadan-with-super.svg";
  }
  if (normalized.includes("дадан")) return "/images/hives/hive-dadan.svg";
  if (normalized.includes("рута")) return "/images/hives/hive-ruta.svg";
  if (normalized.includes("лежак")) return "/images/hives/hive-layens.svg";
  if (normalized.includes("украин")) return "/images/hives/hive-ukrainian.svg";
  if (normalized.includes("колод")) return "/images/hives/hive-log-hive.svg";

  return "/images/hives/hive-generic.svg";
}

export function eventTitle(event: DeviceEvent) {
  const labels: Record<string, string> = {
    alert: "Алерт",
    hive_opened: "Открытие улья",
    measurement: "Измерение",
    comment: "Комментарий",
    device_seen: "Новое устройство"
  };
  return labels[event.event_type] ?? event.event_type ?? "Событие";
}

export function taskStatusLabel(status: string) {
  const labels: Record<string, string> = {
    planned: "Запланировано",
    due: "Сегодня",
    overdue: "Просрочено",
    completed: "Выполнено",
    snoozed: "Отложено",
    dismissed: "Скрыто"
  };
  return labels[status] ?? status;
}

export function offsetPoints(points: string, index: number) {
  return points.split(" ").map((pair) => {
    const [x, y] = pair.split(",").map(Number);
    const offset = Math.sin((x + index * 13) / 12) * 4 + index * 1.2 - 4;
    return `${x},${Math.min(94, Math.max(6, y + offset))}`;
  }).join(" ");
}

export function periodReadings(readings: SensorReading[], period: ChartPeriod) {
  const maxPoints: Record<ChartPeriod, number> = { "1d": 48, "10d": 96, "30d": 96 };
  return readings.slice(-maxPoints[period]);
}

export function deltaPoints(readings: SensorReading[]) {
  const start = readings[0]?.value ?? 0;
  return readings.map((reading) => ({ reading, deltaKg: reading.value - start }));
}

export function deltaLinePoints(points: Array<{ deltaKg: number }>, min: number, range: number) {
  return points.map((point, index) => {
    const x = points.length === 1 ? 50 : (index / (points.length - 1)) * 100;
    const y = 88 - ((point.deltaKg - min) / range) * 72;
    return `${x},${y}`;
  }).join(" ");
}
