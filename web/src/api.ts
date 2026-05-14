export type User = {
  id: string;
  email: string;
  name: string;
  created_at: string;
};

export type Organization = {
  id: string;
  name: string;
  created_at: string;
};

export type Apiary = {
  id: string;
  organization_id: string;
  name: string;
  description: string;
  country: string;
  region: string;
  locality: string;
  address: string;
  location_description: string;
  latitude?: number;
  longitude?: number;
  timezone: string;
  created_at: string;
};

export type Hive = {
  id: string;
  apiary_id: string;
  name: string;
  number: string;
  type: string;
  frame_count?: number;
  super_count?: number;
  bee_breed: string;
  settled_at?: string;
  queen_year?: number;
  queen_breed: string;
  queen_status: string;
  status: string;
  notes: string;
  created_at: string;
  assigned_device_id?: string;
  assigned_device_public_id?: string;
  assigned_device_type?: string;
  assigned_device_count: number;
};

export type Device = {
  id: string;
  apiary_id?: string;
  device_id: string;
  device_type: string;
  status: string;
  firmware_version: string;
  config_version?: number;
  last_telemetry_at?: string;
  last_status_at?: string;
  expected_next_telemetry_at?: string;
  missed_telemetry_count: number;
  telemetry_interval_minutes: number;
  created_at: string;
};

export type SensorReading = {
  id: string;
  device_id: string;
  apiary_id?: string;
  hive_id?: string;
  metric_type: string;
  value: number;
  unit: string;
  measured_at: string;
  received_at: string;
};

export type DeviceEvent = {
  id: string;
  device_id?: string;
  apiary_id?: string;
  hive_id?: string;
  event_type: string;
  message: string;
  ok?: boolean;
  command: string;
  occurred_at: string;
  received_at: string;
  raw_payload_id?: string;
};

export type AdviceItem = {
  id: string;
  code: string;
  title: string;
  body: string;
  category: string;
  severity: "info" | "notice" | "warning" | "critical" | string;
  priority: number;
  source: string;
  related_hive_id?: string;
  action_label: string;
  action_type: string;
  is_user_dismissible: boolean;
  state?: "dismissed" | "snoozed" | string;
};

export type ApiaryTask = {
  id: string;
  apiary_id: string;
  hive_id?: string;
  source_template_id?: string;
  title: string;
  description: string;
  category: string;
  severity: "info" | "notice" | "warning" | "critical" | string;
  status: "planned" | "due" | "overdue" | "completed" | "dismissed" | "snoozed" | string;
  due_at?: string;
  completed_at?: string;
  dismissed_at?: string;
  snoozed_until?: string;
  created_at: string;
  updated_at: string;
};

export type AuthResponse = {
  access_token: string;
  user: User;
};

const apiBase = import.meta.env.VITE_API_BASE_URL ?? "";

export class ApiError extends Error {
  status: number;

  constructor(status: number, message: string) {
    super(message);
    this.status = status;
  }
}

export class ApiClient {
  private token: string | null;

  constructor(token: string | null) {
    this.token = token;
  }

  setToken(token: string | null) {
    this.token = token;
  }

  async login(email: string, password: string): Promise<AuthResponse> {
    return this.request("/auth/login", {
      method: "POST",
      body: { email, password }
    });
  }

  async register(email: string, name: string, password: string): Promise<AuthResponse> {
    return this.request("/auth/register", {
      method: "POST",
      body: { email, name, password }
    });
  }

  me(): Promise<User> {
    return this.request("/me");
  }

  organizations(): Promise<Organization[]> {
    return this.request<Organization[] | null>("/organizations/").then(asArray);
  }

  createOrganization(name: string): Promise<Organization> {
    return this.request("/organizations/", { method: "POST", body: { name } });
  }

  apiaries(organizationId?: string): Promise<Apiary[]> {
    const query = organizationId ? `?organization_id=${encodeURIComponent(organizationId)}` : "";
    return this.request<Apiary[] | null>(`/apiaries/${query}`).then(asArray);
  }

  createApiary(input: Partial<Apiary> & { organization_id: string; name: string }): Promise<Apiary> {
    return this.request("/apiaries/", { method: "POST", body: input });
  }

  hives(apiaryId: string): Promise<Hive[]> {
    return this.request<Hive[] | null>(`/apiaries/${apiaryId}/hives`).then(asArray);
  }

  createHive(apiaryId: string, input: Partial<Hive> & { name: string }): Promise<Hive> {
    return this.request(`/apiaries/${apiaryId}/hives`, { method: "POST", body: input });
  }

  unassignedDevices(apiaryId: string): Promise<Device[]> {
    return this.request<Device[] | null>(`/apiaries/${apiaryId}/devices/unassigned`).then(asArray);
  }

  assignDevice(apiaryId: string, deviceId: string, hiveId: string, importMode: string, replaceExisting = false) {
    return this.request(`/apiaries/${apiaryId}/devices/${deviceId}/assign`, {
      method: "POST",
      body: { hive_id: hiveId, import_mode: importMode, replace_existing: replaceExisting }
    });
  }

  latestTelemetry(hiveId: string): Promise<SensorReading[]> {
    return this.request<SensorReading[] | null>(`/hives/${hiveId}/telemetry/latest`).then(asArray);
  }

  telemetryHistory(hiveId: string, metric: string, options: { from?: Date; to?: Date; limit?: number } = {}): Promise<SensorReading[]> {
    const to = options.to ?? new Date();
    const from = options.from ?? new Date(to.getTime() - 24 * 60 * 60 * 1000);
    const params = new URLSearchParams({
      from: from.toISOString(),
      to: to.toISOString(),
      metric,
      limit: String(options.limit ?? 200)
    });
    return this.request<SensorReading[] | null>(`/hives/${hiveId}/telemetry/history?${params.toString()}`).then(asArray);
  }

  apiaryEvents(apiaryId: string, limit = 100): Promise<DeviceEvent[]> {
    return this.request<DeviceEvent[] | null>(`/apiaries/${apiaryId}/events?limit=${limit}`).then(asArray);
  }

  apiaryAdvice(apiaryId: string, date = new Date(), includeHidden = false): Promise<AdviceItem[]> {
    const params = new URLSearchParams({ date: date.toISOString() });
    if (includeHidden) params.set("include_hidden", "true");
    return this.request<{ items: AdviceItem[] | null }>(`/apiaries/${apiaryId}/advice?${params.toString()}`)
      .then((response) => asArray(response.items));
  }

  updateAdviceState(apiaryId: string, adviceCode: string, status: "dismissed" | "snoozed", snoozedUntil?: Date) {
    return this.request(`/apiaries/${apiaryId}/advice/${encodeURIComponent(adviceCode)}`, {
      method: "PATCH",
      body: { status, snoozed_until: snoozedUntil?.toISOString() }
    });
  }

  apiaryTasks(apiaryId: string, from: Date, to: Date): Promise<ApiaryTask[]> {
    const params = new URLSearchParams({ from: from.toISOString(), to: to.toISOString() });
    return this.request<{ items: ApiaryTask[] | null }>(`/apiaries/${apiaryId}/calendar/tasks?${params.toString()}`)
      .then((response) => asArray(response.items));
  }

  createApiaryTask(apiaryId: string, input: Partial<ApiaryTask> & { title: string }) {
    return this.request<ApiaryTask>(`/apiaries/${apiaryId}/calendar/tasks`, { method: "POST", body: input });
  }

  updateApiaryTask(apiaryId: string, taskId: string, status: ApiaryTask["status"], snoozedUntil?: Date) {
    return this.request<ApiaryTask>(`/apiaries/${apiaryId}/calendar/tasks/${taskId}`, {
      method: "PATCH",
      body: { status, snoozed_until: snoozedUntil?.toISOString() }
    });
  }

  hiveEvents(hiveId: string, limit = 100): Promise<DeviceEvent[]> {
    return this.request<DeviceEvent[] | null>(`/hives/${hiveId}/events?limit=${limit}`).then(asArray);
  }

  private async request<T>(path: string, options: { method?: string; body?: unknown } = {}): Promise<T> {
    const headers: Record<string, string> = { Accept: "application/json" };
    if (options.body !== undefined) headers["Content-Type"] = "application/json";
    if (this.token) headers.Authorization = `Bearer ${this.token}`;

    const response = await fetch(`${apiBase}${path}`, {
      method: options.method ?? "GET",
      headers,
      body: options.body !== undefined ? JSON.stringify(options.body) : undefined
    });

    if (!response.ok) {
      let message = response.statusText;
      const responseText = await response.text();

      try {
        const data = JSON.parse(responseText);
        message = data.error ?? data.message ?? message;
      } catch {
        message = responseText || message;
      }

      throw new ApiError(response.status, message);
    }

    if (response.status === 204) {
      return undefined as T;
    }

    return response.json() as Promise<T>;
  }
}

function asArray<T>(value: T[] | null): T[] {
  return Array.isArray(value) ? value : [];
}
