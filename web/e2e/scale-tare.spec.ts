import { expect, test } from "@playwright/test";
import { demoEmail, demoPassword, login, queryDatabaseJSON, runDatabaseSQL } from "./support";

const apiBase = process.env.HIVEMONITOR_E2E_API_BASE_URL ?? "http://127.0.0.1:8080";
const tareComment = "e2e scale tare verification";

type HiveWithWeight = {
  hive_id: string;
  raw_weight: number;
};

test.describe("Hive scale tare", () => {
  test.afterEach(() => cleanupTareFixture());

  test("stores backend hive tare and returns net weight with raw value", async ({ page, request }) => {
    cleanupTareFixture();
    const hive = queryDatabaseJSON<HiveWithWeight>(`
select json_build_object(
  'hive_id', h.id::text,
  'raw_weight', sr.value
)
from hives h
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
});

function cleanupTareFixture() {
  runDatabaseSQL(`
delete from hive_scale_profiles
where hive_id in (
  select hive_id
  from hive_tare_events
  where comment = '${tareComment}'
);
delete from hive_tare_events where comment = '${tareComment}';
`);
}
