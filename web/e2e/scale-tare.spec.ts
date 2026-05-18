import { expect, test } from "@playwright/test";
import { login, queryDatabaseJSON, runDatabaseSQL } from "./support";

const apiBase = process.env.HIVEMONITOR_E2E_API_BASE_URL ?? "http://127.0.0.1:8080";
const tareComment = "e2e scale tare verification";

type HiveWithWeight = {
  hive_id: string;
  hive_name: string;
  device_id: string;
  raw_weight: number;
};

test.describe("Hive scale tare", () => {
  test.afterEach(() => cleanupTareFixture());

  test("stores backend hive tare and returns net weight with raw value", async ({ page, request }) => {
    cleanupTareFixture();
    const hive = testHiveWithWeight();
    expect(hive.raw_weight).toBeGreaterThan(10);

    await login(page);
    const token = await page.evaluate(() => localStorage.getItem("hivemonitor.accessToken"));
    expect(token).toBeTruthy();

    const tare = hive.raw_weight - 4.25;
    const saveResponse = await request.post(`${apiBase}/hives/${hive.hive_id}/scale/tare`, {
      headers: { authorization: `Bearer ${token}` },
      data: {
        tare_kind: "hive",
        measured_raw_weight_kg: tare,
        comment: tareComment,
        metadata: { source: "e2e" }
      }
    });
    expect(saveResponse.ok()).toBeTruthy();

    const latestResponse = await request.get(`${apiBase}/hives/${hive.hive_id}/telemetry/latest`, {
      headers: { authorization: `Bearer ${token}` }
    });
    expect(latestResponse.ok()).toBeTruthy();
    const latest = await latestResponse.json();
    const weight = latest.find((reading: { metric_type: string }) => reading.metric_type === "weight");
    expect(weight.raw_value).toBeCloseTo(hive.raw_weight, 2);
    expect(weight.value).toBeCloseTo(4.25, 2);

    const dbProfile = queryDatabaseJSON<{ active_tare_kg: number }>(`
select json_build_object('active_tare_kg', active_tare_kg)
from hive_scale_profiles
where hive_id = '${hive.hive_id}'::uuid;
`);
    expect(dbProfile.active_tare_kg).toBeCloseTo(tare, 2);
  });

  test("removes the latest super tare and restores previous tare baseline", async ({ page, request }) => {
    cleanupTareFixture();
    const hive = testHiveWithWeight();
    const hiveTare = hive.raw_weight - 8;
    const superTare = hive.raw_weight - 3;

    await login(page);
    const token = await page.evaluate(() => localStorage.getItem("hivemonitor.accessToken"));
    expect(token).toBeTruthy();

    await request.post(`${apiBase}/hives/${hive.hive_id}/scale/tare`, {
      headers: { authorization: `Bearer ${token}` },
      data: { tare_kind: "hive", measured_raw_weight_kg: hiveTare, comment: tareComment }
    });
    await request.post(`${apiBase}/hives/${hive.hive_id}/scale/tare`, {
      headers: { authorization: `Bearer ${token}` },
      data: { tare_kind: "super", measured_raw_weight_kg: superTare, comment: tareComment }
    });

    const removeResponse = await request.post(`${apiBase}/hives/${hive.hive_id}/scale/supers/remove`, {
      headers: { authorization: `Bearer ${token}` },
      data: { comment: tareComment }
    });
    expect(removeResponse.ok()).toBeTruthy();

    const dbProfile = queryDatabaseJSON<{ active_tare_kg: number; super_count: number }>(`
select json_build_object(
  'active_tare_kg', active_tare_kg,
  'super_count', jsonb_array_length(super_tares)
)
from hive_scale_profiles
where hive_id = '${hive.hive_id}'::uuid;
`);
    expect(dbProfile.active_tare_kg).toBeCloseTo(hiveTare, 2);
    expect(dbProfile.super_count).toBe(0);
  });

  test("runs the hive tare wizard through UI using device command result", async ({ page }) => {
    cleanupTareFixture();
    const hive = testHiveWithWeight();
    const tare = Number((hive.raw_weight - 6.5).toFixed(2));

    await login(page);
    await page.getByLabel("Пасека:").selectOption({ label: "Демо пасека Северная" });
    await expect(page.getByRole("heading", { name: "Демо пасека Северная" })).toBeVisible();

    await page.locator(".hive-table-row").filter({ hasText: hive.hive_name }).first().click();
    const detail = page.locator(".hive-detail");
    await expect(detail).toBeVisible();
    await detail.getByRole("button", { name: "Тара улья" }).click();

    const modal = page.getByRole("dialog", { name: "Тара улья" });
    await expect(modal).toBeVisible();
    markDeviceAwake(hive.device_id);
    await expect(modal.getByText("Устройство активно")).toBeVisible({ timeout: 6000 });

    await modal.getByRole("button", { name: "Далее" }).click();
    await expect(modal.getByText("Выньте из улья все рамки")).toBeVisible();
    await modal.getByRole("button", { name: "Замерить вес" }).click();

    const captureCommand = latestCaptureCommand(hive.device_id);
    acknowledgeCaptureCommand(captureCommand.command_id, tare);
    await expect(modal.getByText(`${tare.toFixed(2).replace(".", ",")} кг`)).toBeVisible({ timeout: 6000 });
    await modal.getByRole("button", { name: "Сохранить тару" }).click();
    await expect(modal.getByText("Тара сохранена в backend")).toBeVisible();
    await modal.getByRole("button", { name: "Да, отпустить устройство" }).click();
    await expect(modal).toBeHidden();

    const dbProfile = queryDatabaseJSON<{ active_tare_kg: number; empty_hive_tare_kg: number }>(`
select json_build_object(
  'active_tare_kg', active_tare_kg,
  'empty_hive_tare_kg', empty_hive_tare_kg
)
from hive_scale_profiles
where hive_id = '${hive.hive_id}'::uuid;
`);
    expect(dbProfile.active_tare_kg).toBeCloseTo(tare, 2);
    expect(dbProfile.empty_hive_tare_kg).toBeCloseTo(tare, 2);
  });
});

function testHiveWithWeight() {
  return queryDatabaseJSON<HiveWithWeight>(`
select json_build_object(
  'hive_id', h.id::text,
  'hive_name', h.name,
  'device_id', d.id::text,
  'raw_weight', sr.value
)
from hives h
join device_assignments da on da.hive_id = h.id and da.unassigned_at is null
join devices d on d.id = da.device_id
join lateral (
  select value
  from sensor_readings
  where hive_id = h.id and metric_type = 'weight'
  order by measured_at desc
  limit 1
) sr on true
where h.apiary_id = '33333333-3333-3333-3333-333333333331'::uuid
order by h.created_at desc
limit 1;
`);
}

function markDeviceAwake(deviceId: string) {
  runDatabaseSQL(`
update devices
set last_status_at = now() + interval '2 seconds',
    updated_at = now()
where id = '${deviceId}'::uuid;
`);
}

function latestCaptureCommand(deviceId: string) {
  return queryDatabaseJSON<{ command_id: string }>(`
select json_build_object('command_id', id::text)
from device_commands
where device_id = '${deviceId}'::uuid
  and command = 'capture_weight'
  and payload->>'purpose' = 'hive_tare'
order by created_at desc
limit 1;
`);
}

function acknowledgeCaptureCommand(commandId: string, rawWeight: number) {
  runDatabaseSQL(`
update device_commands
set status = 'acknowledged',
    result = jsonb_build_object('raw_weight_kg', ${rawWeight}, 'weight_kg', ${rawWeight}, 'session_active', true),
    acknowledged_at = now(),
    updated_at = now()
where id = '${commandId}'::uuid;
`);
}

function cleanupTareFixture() {
  runDatabaseSQL(`
delete from hive_scale_profiles
where hive_id in (
  select hive_id
  from hive_tare_events
  where comment = '${tareComment}'
);
delete from hive_tare_events where comment = '${tareComment}';
delete from hive_scale_profiles
where hive_id in (
  select hte.hive_id
  from hive_tare_events hte
  join device_commands dc on dc.id = hte.command_id
  where dc.command in ('hold_config_session', 'capture_weight', 'finish_config_session')
    and dc.payload->>'purpose' in ('hive_tare', 'super_tare')
    and dc.created_at >= now() - interval '1 hour'
);
delete from hive_tare_events
where command_id in (
  select id
  from device_commands
  where command in ('hold_config_session', 'capture_weight', 'finish_config_session')
    and payload->>'purpose' in ('hive_tare', 'super_tare')
    and created_at >= now() - interval '1 hour'
);
delete from device_commands
where command in ('hold_config_session', 'capture_weight', 'finish_config_session')
  and payload->>'purpose' in ('hive_tare', 'super_tare')
  and created_at >= now() - interval '1 hour';
`);
}
