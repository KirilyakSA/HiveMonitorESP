import { expect, test, type APIRequestContext, type Page } from "@playwright/test";
import type { AuthResponse, Hive, SensorReading } from "../src/api";
import {
  demoEmail,
  demoPassword,
  login,
  northApiaryHiveSnapshotFromDatabase,
  northApiaryId,
  selectNorthApiary,
  type DatabaseHiveSnapshot
} from "./support";

type ApiHiveSnapshot = {
  hive: Hive;
  latest: Record<string, SensorReading>;
  hasWeightHistory: boolean;
};

test.describe("DB, API and UI consistency", () => {
  test("shows the same North apiary hive data in Postgres, API and dashboard UI", async ({ page, request }) => {
    const dbHives = northApiaryHiveSnapshotFromDatabase();
    expect(dbHives.length).toBeGreaterThan(0);

    const apiHives = await northApiaryHiveSnapshotFromApi(request);
    expect(apiHives.map((item) => normalizeApiHive(item.hive))).toEqual(dbHives.map(normalizeDbHive));

    for (const dbHive of dbHives) {
      const apiHive = apiHives.find((item) => item.hive.id === dbHive.id);
      expect(apiHive, `API hive ${dbHive.name} should exist`).toBeTruthy();
      compareLatestMetric(dbHive, apiHive!.latest, "weight");
      compareLatestMetric(dbHive, apiHive!.latest, "temperature");
      compareLatestMetric(dbHive, apiHive!.latest, "weight_change");
      expect(apiHive!.hasWeightHistory).toBe(dbHive.has_weight_history);
    }

    await login(page);
    await selectNorthApiary(page);
    await expect(page.locator(".hive-table-row")).toHaveCount(dbHives.length);

    for (const dbHive of dbHives) {
      await expectUiHiveRow(page, dbHive);
    }

    const expectedSeriesCount = apiHives.filter((item) => item.hasWeightHistory).length;
    const comparison = page.locator(".comparison-panel");
    await comparison.getByRole("button", { name: "30 дней" }).click();
    await expect(comparison.getByRole("img", { name: "Сравнительный график веса ульев" })).toBeVisible();
    await expect(comparison.locator(".multi-legend button")).toHaveCount(expectedSeriesCount);
    await expect(comparison.locator(".series-line")).toHaveCount(expectedSeriesCount);

    for (const item of apiHives.filter((snapshot) => snapshot.hasWeightHistory)) {
      await expect(comparison.locator(".multi-legend")).toContainText(item.hive.name);
    }
  });
});

async function northApiaryHiveSnapshotFromApi(request: APIRequestContext): Promise<ApiHiveSnapshot[]> {
  const token = await loginViaApi(request);
  const headers = { Authorization: `Bearer ${token}` };
  const hivesResponse = await request.get(`/apiaries/${northApiaryId}/hives`, { headers });
  expect(hivesResponse.ok()).toBeTruthy();
  const hives = await hivesResponse.json() as Hive[];

  const to = new Date();
  const from = new Date(to.getTime() - 30 * 24 * 60 * 60 * 1000);
  const snapshots: ApiHiveSnapshot[] = [];

  for (const hive of hives) {
    const latestResponse = await request.get(`/hives/${hive.id}/telemetry/latest`, { headers });
    expect(latestResponse.ok()).toBeTruthy();
    const latestReadings = await latestResponse.json() as SensorReading[];

    const historyParams = new URLSearchParams({
      metric: "weight",
      from: from.toISOString(),
      to: to.toISOString(),
      limit: "900"
    });
    const historyResponse = await request.get(`/hives/${hive.id}/telemetry/history?${historyParams.toString()}`, { headers });
    expect(historyResponse.ok()).toBeTruthy();
    const history = await historyResponse.json() as SensorReading[];

    snapshots.push({
      hive,
      latest: indexReadings(latestReadings),
      hasWeightHistory: history.length > 0
    });
  }

  return snapshots;
}

async function loginViaApi(request: APIRequestContext) {
  const response = await request.post("/auth/login", {
    data: {
      email: demoEmail,
      password: demoPassword
    }
  });
  expect(response.ok()).toBeTruthy();
  const body = await response.json() as AuthResponse;
  return body.access_token;
}

function normalizeDbHive(hive: DatabaseHiveSnapshot) {
  return {
    id: hive.id,
    name: hive.name,
    number: hive.number ?? "",
    type: hive.type ?? "",
    status: hive.status,
    assigned_device_id: hive.assigned_device_id ?? undefined,
    assigned_device_public_id: hive.assigned_device_public_id ?? undefined,
    assigned_device_type: hive.assigned_device_type ?? undefined,
    assigned_device_count: hive.assigned_device_count
  };
}

function normalizeApiHive(hive: Hive) {
  return {
    id: hive.id,
    name: hive.name,
    number: hive.number ?? "",
    type: hive.type ?? "",
    status: hive.status,
    assigned_device_id: hive.assigned_device_id,
    assigned_device_public_id: hive.assigned_device_public_id,
    assigned_device_type: hive.assigned_device_type,
    assigned_device_count: hive.assigned_device_count
  };
}

function compareLatestMetric(dbHive: DatabaseHiveSnapshot, latest: Record<string, SensorReading>, metric: "weight" | "temperature" | "weight_change") {
  const dbValue = dbHive[metric];
  const apiValue = latest[metric]?.value ?? null;
  if (dbValue === null) {
    expect(apiValue, `${dbHive.name} ${metric}`).toBeNull();
    return;
  }
  expect(apiValue, `${dbHive.name} ${metric}`).not.toBeNull();
  expect(apiValue!).toBeCloseTo(dbValue, 3);
}

async function expectUiHiveRow(page: Page, hive: DatabaseHiveSnapshot) {
  const row = page.locator(".hive-table-row")
    .filter({ hasText: hive.name })
    .filter({ hasText: hive.number || hive.name })
    .first();
  await expect(row, `UI row for ${hive.name}`).toBeVisible();
  await expect(row).toContainText(hive.name);
  if (hive.number) await expect(row).toContainText(hive.number);
  await expect(row).toContainText(hive.type || "Дадан");
  await expect(row).toContainText(hive.assigned_device_public_id || "Нет устройства");
  if (hive.weight !== null) await expect(row).toContainText(valueOnly(hive.weight));
  if (hive.temperature !== null) await expect(row).toContainText(`${valueOnly(hive.temperature)} °C`);
}

function indexReadings(readings: SensorReading[]) {
  return readings.reduce<Record<string, SensorReading>>((acc, reading) => {
    acc[reading.metric_type] = reading;
    return acc;
  }, {});
}

function valueOnly(value: number) {
  return Math.abs(value) >= 100 ? value.toFixed(0) : value.toFixed(2).replace(".", ",");
}
