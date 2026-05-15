import { expect, type Page } from "@playwright/test";
import { execFileSync } from "node:child_process";

export const demoEmail = process.env.HIVEMONITOR_E2E_EMAIL ?? "demo@hivemonitor.local";
export const demoPassword = process.env.HIVEMONITOR_E2E_PASSWORD ?? "password123";
export const fullCycleDevicePublicId = "e2e-unassigned-full-cycle";
export const replacementDevicePublicId = "e2e-unassigned-replacement";
export const fullCycleHiveName = "E2E улей автотест";
export const fullCycleHiveNumber = "E2E-999";
export const apiaryCycleName = "E2E пасека автотест";
export const northApiaryId = "33333333-3333-3333-3333-333333333331";

export type DatabaseHiveSnapshot = {
  id: string;
  name: string;
  number: string | null;
  type: string | null;
  status: string;
  assigned_device_id: string | null;
  assigned_device_public_id: string | null;
  assigned_device_type: string | null;
  assigned_device_count: number;
  weight: number | null;
  temperature: number | null;
  weight_change: number | null;
  has_weight_history: boolean;
};

export async function login(page: Page) {
  await page.goto("/");
  await page.evaluate(() => localStorage.removeItem("hivemonitor.accessToken"));
  await page.reload();

  await expect(page.getByRole("heading", { name: "Вход" })).toBeVisible();
  await page.getByLabel("Email").fill(demoEmail);
  await page.getByLabel("Пароль").fill(demoPassword);
  await page.getByRole("button", { name: "Войти" }).click();
  await expect(page.getByText("Панель сводок пасеки")).toBeVisible();
}

export async function selectNorthApiary(page: Page) {
  await page.getByLabel("Пасека:").selectOption({ label: "Демо пасека Северная" });
  await expect(page.getByRole("heading", { name: "Демо пасека Северная" })).toBeVisible();
}

export function runDatabaseSQL(sql: string) {
  const container = process.env.HIVEMONITOR_E2E_DB_CONTAINER ?? "deploy-postgres-1";
  const user = process.env.HIVEMONITOR_E2E_DB_USER ?? "hivemonitor";
  const database = process.env.HIVEMONITOR_E2E_DB_NAME ?? "hivemonitor";
  execFileSync("docker", ["exec", "-i", container, "psql", "-v", "ON_ERROR_STOP=1", "-U", user, "-d", database], {
    input: sql,
    encoding: "utf8",
    stdio: ["pipe", "pipe", "pipe"]
  });
}

export function queryDatabaseJSON<T>(sql: string): T {
  const container = process.env.HIVEMONITOR_E2E_DB_CONTAINER ?? "deploy-postgres-1";
  const user = process.env.HIVEMONITOR_E2E_DB_USER ?? "hivemonitor";
  const database = process.env.HIVEMONITOR_E2E_DB_NAME ?? "hivemonitor";
  const output = execFileSync("docker", ["exec", "-i", container, "psql", "-v", "ON_ERROR_STOP=1", "-q", "-t", "-A", "-U", user, "-d", database], {
    input: sql,
    encoding: "utf8",
    stdio: ["pipe", "pipe", "pipe"]
  }).trim();

  return JSON.parse(output || "null") as T;
}

export function northApiaryHiveSnapshotFromDatabase() {
  return queryDatabaseJSON<DatabaseHiveSnapshot[]>(`
with active_assignments as (
  select distinct on (hive_id)
    hive_id,
    device_id
  from device_assignments
  where unassigned_at is null
  order by hive_id, assigned_at desc
),
assignment_counts as (
  select hive_id, count(*)::int as assigned_device_count
  from device_assignments
  where unassigned_at is null
  group by hive_id
),
latest_readings as (
  select distinct on (hive_id, metric_type)
    hive_id,
    metric_type,
    value
  from sensor_readings
  where apiary_id = '${northApiaryId}'::uuid
    and hive_id is not null
    and metric_type in ('weight', 'temperature', 'weight_change')
  order by hive_id, metric_type, measured_at desc
),
weight_history as (
  select hive_id, count(*) > 0 as has_weight_history
  from sensor_readings
  where apiary_id = '${northApiaryId}'::uuid
    and hive_id is not null
    and metric_type = 'weight'
    and measured_at >= now() - interval '30 days'
  group by hive_id
)
select coalesce(json_agg(json_build_object(
  'id', h.id::text,
  'name', h.name,
  'number', h.number,
  'type', h.type,
  'status', h.status,
  'assigned_device_id', d.id::text,
  'assigned_device_public_id', d.device_id,
  'assigned_device_type', d.device_type,
  'assigned_device_count', coalesce(ac.assigned_device_count, 0),
  'weight', w.value,
  'temperature', t.value,
  'weight_change', wc.value,
  'has_weight_history', coalesce(wh.has_weight_history, false)
) order by h.created_at desc), '[]'::json)
from hives h
left join active_assignments aa on aa.hive_id = h.id
left join assignment_counts ac on ac.hive_id = h.id
left join devices d on d.id = aa.device_id
left join latest_readings w on w.hive_id = h.id and w.metric_type = 'weight'
left join latest_readings t on t.hive_id = h.id and t.metric_type = 'temperature'
left join latest_readings wc on wc.hive_id = h.id and wc.metric_type = 'weight_change'
left join weight_history wh on wh.hive_id = h.id
where h.apiary_id = '${northApiaryId}'::uuid;
`);
}

export function cleanupFullCycleFixture() {
  runDatabaseSQL(`
begin;

delete from sensor_readings
where device_id in (select id from devices where device_id = '${fullCycleDevicePublicId}')
   or hive_id in (select id from hives where apiary_id = '${northApiaryId}'::uuid and name = '${fullCycleHiveName}');

delete from device_events
where device_id in (select id from devices where device_id = '${fullCycleDevicePublicId}')
   or hive_id in (select id from hives where apiary_id = '${northApiaryId}'::uuid and name = '${fullCycleHiveName}');

delete from raw_payloads
where device_id in (select id from devices where device_id = '${fullCycleDevicePublicId}');

delete from device_assignments
where device_id in (select id from devices where device_id = '${fullCycleDevicePublicId}')
   or hive_id in (select id from hives where apiary_id = '${northApiaryId}'::uuid and name = '${fullCycleHiveName}');

delete from devices
where device_id = '${fullCycleDevicePublicId}';

delete from devices
where device_id = '${replacementDevicePublicId}';

delete from hives
where apiary_id = '${northApiaryId}'::uuid
  and name = '${fullCycleHiveName}';

delete from apiaries
where name = '${apiaryCycleName}';

commit;
`);
}

export function seedFullCycleUnassignedDevices() {
  runDatabaseSQL(`
begin;

insert into devices (
  apiary_id, device_id, device_type, status, firmware_version, config_version,
  last_telemetry_at, last_status_at, expected_next_telemetry_at,
  missed_telemetry_count, telemetry_interval_minutes, created_at, updated_at
)
values (
  '${northApiaryId}'::uuid,
  '${fullCycleDevicePublicId}',
  'hive_monitor',
  'unassigned',
  '0.3.0-e2e',
  99,
  now() - interval '7 minutes',
  now() - interval '6 minutes',
  now() + interval '23 minutes',
  0,
  30,
  now() - interval '1 hour',
  now()
);

insert into devices (
  apiary_id, device_id, device_type, status, firmware_version, config_version,
  last_telemetry_at, last_status_at, expected_next_telemetry_at,
  missed_telemetry_count, telemetry_interval_minutes, created_at, updated_at
)
values (
  '${northApiaryId}'::uuid,
  '${replacementDevicePublicId}',
  'hive_monitor',
  'unassigned',
  '0.3.0-e2e',
  100,
  now() - interval '5 minutes',
  now() - interval '4 minutes',
  now() + interval '25 minutes',
  0,
  30,
  now() - interval '50 minutes',
  now()
);

insert into sensor_readings (device_id, apiary_id, hive_id, metric_type, value, unit, measured_at, received_at)
select d.id, d.apiary_id, null, sample.metric_type, sample.value, sample.unit, now() - sample.age, now()
from devices d
cross join (values
  ('weight', 37.42::double precision, 'kg', interval '7 minutes'),
  ('weight_change', 1.25::double precision, 'kg', interval '7 minutes'),
  ('temperature', 32.8::double precision, '°C', interval '7 minutes'),
  ('humidity', 61.0::double precision, '%', interval '7 minutes'),
  ('battery_percent', 84.0::double precision, '%', interval '7 minutes')
) as sample(metric_type, value, unit, age)
where d.device_id = '${fullCycleDevicePublicId}';

insert into sensor_readings (device_id, apiary_id, hive_id, metric_type, value, unit, measured_at, received_at)
select d.id, d.apiary_id, null, sample.metric_type, sample.value, sample.unit, now() - sample.age, now()
from devices d
cross join (values
  ('weight', 41.55::double precision, 'kg', interval '5 minutes'),
  ('weight_change', 2.05::double precision, 'kg', interval '5 minutes'),
  ('temperature', 33.4::double precision, '°C', interval '5 minutes'),
  ('humidity', 58.0::double precision, '%', interval '5 minutes'),
  ('battery_percent', 91.0::double precision, '%', interval '5 minutes')
) as sample(metric_type, value, unit, age)
where d.device_id = '${replacementDevicePublicId}';

commit;
`);
}

export function resetFullCycleFixture() {
  cleanupFullCycleFixture();
  seedFullCycleUnassignedDevices();
}
