import { expect, test } from "@playwright/test";
import {
  apiaryCycleName,
  cleanupFullCycleFixture,
  fullCycleDevicePublicId,
  fullCycleHiveName,
  fullCycleHiveNumber,
  replacementDevicePublicId,
  resetFullCycleFixture,
  login
} from "./support";

test.describe("HiveMonitor device assignment full cycle", () => {
  test.describe.configure({ mode: "serial" });

  test.beforeEach(() => {
    resetFullCycleFixture();
  });

  test.afterEach(() => {
    cleanupFullCycleFixture();
  });

  test("creates a hive, assigns a seeded unassigned device, verifies telemetry, and cleans it up", async ({ page }) => {
    await login(page);
    await page.getByLabel("Пасека:").selectOption({ label: "Демо пасека Северная" });
    await expect(page.getByRole("heading", { name: "Демо пасека Северная" })).toBeVisible();

    const hivesPanel = page.locator(".hives-panel");
    await hivesPanel.locator(".table-actions").getByRole("button", { name: "Улей", exact: true }).click();
    await hivesPanel.getByPlaceholder("Название").fill(fullCycleHiveName);
    await hivesPanel.getByPlaceholder("Номер").fill(fullCycleHiveNumber);
    await hivesPanel.getByPlaceholder("Тип").fill("Дадан");
    await hivesPanel.getByPlaceholder("Рамки").fill("12");
    await hivesPanel.getByPlaceholder("Магазины").fill("0");
    await hivesPanel.getByRole("button", { name: "Создать" }).click();

    await expect(page.locator(".hive-detail")).toBeVisible();
    await expect(page.locator(".hive-detail").getByRole("heading", { name: fullCycleHiveName })).toBeVisible();
    await expect(page.locator(".hive-detail")).toContainText("Нет устройства");
    await page.getByRole("button", { name: "Закрыть карточку улья" }).click();

    const deviceRow = page.locator(".device-row").filter({ hasText: fullCycleDevicePublicId });
    await expect(deviceRow).toBeVisible();
    await deviceRow.getByRole("button", { name: "Привязать" }).click();

    await expect(page.getByRole("heading", { name: "Выберите улей для устройства" })).toBeVisible();
    const assignDialog = page.getByRole("dialog", { name: "Выберите улей для устройства" });
    await assignDialog.locator("select").selectOption({
      label: `Улей ${fullCycleHiveNumber} · Дадан · Нет устройства`
    });
    await expect(page.getByText("Что сделать со старой непривязанной телеметрией")).toBeVisible();
    await page.getByRole("button", { name: "Привязать устройство" }).click();

    await expect(page.locator(".hive-detail")).toBeVisible();
    await expect(page.locator(".hive-detail")).toContainText(fullCycleDevicePublicId);
    await expect(page.locator(".hive-detail")).toContainText("37,42 kg");
    await expect(deviceRow).toBeHidden();

    await page.getByRole("button", { name: "Закрыть карточку улья" }).click();
    const createdHiveRow = page.locator(".hive-table-row").filter({ hasText: fullCycleHiveName });
    await expect(createdHiveRow).toContainText(fullCycleDevicePublicId);
  });

  test("creates an apiary through UI and cleans it up through the fixture", async ({ page }) => {
    await login(page);

    await page.getByRole("button", { name: "Пасеки", exact: true }).click();
    await expect(page.getByRole("heading", { name: "Пасеки" })).toBeVisible();
    await page.locator(".apiaries-screen").getByRole("button", { name: "Новая пасека" }).click();

    await expect(page.getByRole("heading", { name: "Создание пасеки" })).toBeVisible();
    await page.getByLabel("Название").fill(apiaryCycleName);
    await page.getByLabel("Область / регион").fill("Тестовая область");
    await page.getByLabel("Населенный пункт").fill("Тестовое место");
    await page.getByLabel("Адрес").fill("Тестовая локация 1");
    await page.getByLabel("Широта").fill("50.123456");
    await page.getByLabel("Долгота").fill("30.654321");
    await page.getByRole("button", { name: "Создать пасеку" }).click();

    const createdApiaryRow = page.locator(".apiary-row").filter({ hasText: apiaryCycleName });
    await expect(createdApiaryRow).toBeVisible();
    await createdApiaryRow.click();
    await expect(page.locator(".apiary-detail-panel")).toContainText(apiaryCycleName);
  });

  test("replaces an assigned device with another unassigned device", async ({ page }) => {
    await login(page);
    await page.getByLabel("Пасека:").selectOption({ label: "Демо пасека Северная" });
    await expect(page.getByRole("heading", { name: "Демо пасека Северная" })).toBeVisible();

    const hivesPanel = page.locator(".hives-panel");
    await hivesPanel.locator(".table-actions").getByRole("button", { name: "Улей", exact: true }).click();
    await hivesPanel.getByPlaceholder("Название").fill(fullCycleHiveName);
    await hivesPanel.getByPlaceholder("Номер").fill(fullCycleHiveNumber);
    await hivesPanel.getByPlaceholder("Тип").fill("Дадан");
    await hivesPanel.getByRole("button", { name: "Создать" }).click();
    await expect(page.locator(".hive-detail")).toContainText("Нет устройства");
    await page.getByRole("button", { name: "Закрыть карточку улья" }).click();

    const firstDeviceRow = page.locator(".device-row").filter({ hasText: fullCycleDevicePublicId });
    await firstDeviceRow.getByRole("button", { name: "Привязать" }).click();
    const firstAssignDialog = page.getByRole("dialog", { name: "Выберите улей для устройства" });
    await firstAssignDialog.locator("select").selectOption({
      label: `Улей ${fullCycleHiveNumber} · Дадан · Нет устройства`
    });
    await page.getByRole("button", { name: "Привязать устройство" }).click();
    await expect(page.locator(".hive-detail")).toContainText(fullCycleDevicePublicId);
    await page.getByRole("button", { name: "Закрыть карточку улья" }).click();

    const replacementDeviceRow = page.locator(".device-row").filter({ hasText: replacementDevicePublicId });
    await expect(replacementDeviceRow).toBeVisible();
    await replacementDeviceRow.getByRole("button", { name: "Привязать" }).click();

    const replacementDialog = page.getByRole("dialog", { name: "Выберите улей для устройства" });
    await replacementDialog.locator("select").selectOption({
      label: `Улей ${fullCycleHiveNumber} · Дадан · ${fullCycleDevicePublicId}`
    });
    await expect(page.getByText("В этом улье уже есть устройство")).toBeVisible();
    await page.getByLabel(/Заменить текущее устройство/).check();
    await page.getByRole("button", { name: "Привязать устройство" }).click();

    await expect(page.locator(".hive-detail")).toContainText(replacementDevicePublicId);
    await expect(page.locator(".hive-detail")).toContainText("41,55 kg");
    await expect(page.locator(".device-row").filter({ hasText: fullCycleDevicePublicId })).toBeVisible();
  });

  test("deletes an assigned device through the hive drawer", async ({ page }) => {
    await login(page);
    page.on("dialog", (dialog) => dialog.accept());
    await page.getByLabel("Пасека:").selectOption({ label: "Демо пасека Северная" });

    const hivesPanel = page.locator(".hives-panel");
    await hivesPanel.locator(".table-actions").getByRole("button", { name: "Улей", exact: true }).click();
    await hivesPanel.getByPlaceholder("Название").fill(fullCycleHiveName);
    await hivesPanel.getByPlaceholder("Номер").fill(fullCycleHiveNumber);
    await hivesPanel.getByRole("button", { name: "Создать" }).click();
    await page.getByRole("button", { name: "Закрыть карточку улья" }).click();

    await page.locator(".device-row").filter({ hasText: fullCycleDevicePublicId }).getByRole("button", { name: "Привязать" }).click();
    const assignDialog = page.getByRole("dialog", { name: "Выберите улей для устройства" });
    await assignDialog.locator("select").selectOption({
      label: `Улей ${fullCycleHiveNumber} · Дадан · Нет устройства`
    });
    await page.getByRole("button", { name: "Привязать устройство" }).click();
    await expect(page.locator(".hive-detail")).toContainText(fullCycleDevicePublicId);

    await page.getByRole("button", { name: "Удалить устройство" }).click();
    await expect(page.locator(".hive-detail")).toContainText("Нет устройства");
    await expect(page.locator(".hive-detail")).not.toContainText(fullCycleDevicePublicId);
  });

  test("deletes a hive through the hive drawer", async ({ page }) => {
    await login(page);
    page.on("dialog", (dialog) => dialog.accept());
    await page.getByLabel("Пасека:").selectOption({ label: "Демо пасека Северная" });

    const hivesPanel = page.locator(".hives-panel");
    await hivesPanel.locator(".table-actions").getByRole("button", { name: "Улей", exact: true }).click();
    await hivesPanel.getByPlaceholder("Название").fill(fullCycleHiveName);
    await hivesPanel.getByPlaceholder("Номер").fill(fullCycleHiveNumber);
    await hivesPanel.getByRole("button", { name: "Создать" }).click();
    await expect(page.locator(".hive-detail")).toContainText(fullCycleHiveName);

    await page.getByRole("button", { name: "Удалить улей" }).click();
    await expect(page.locator(".hive-detail")).toBeHidden();
    await expect(page.locator(".hive-table-row").filter({ hasText: fullCycleHiveName })).toBeHidden();
  });

  test("deletes an apiary through the apiaries screen", async ({ page }) => {
    await login(page);
    page.on("dialog", (dialog) => dialog.accept());

    await page.getByRole("button", { name: "Пасеки", exact: true }).click();
    await page.locator(".apiaries-screen").getByRole("button", { name: "Новая пасека" }).click();
    await page.getByLabel("Название").fill(apiaryCycleName);
    await page.getByLabel("Область / регион").fill("Тестовая область");
    await page.getByLabel("Населенный пункт").fill("Тестовое место");
    await page.getByRole("button", { name: "Создать пасеку" }).click();

    const createdApiaryRow = page.locator(".apiary-row").filter({ hasText: apiaryCycleName });
    await expect(createdApiaryRow).toBeVisible();
    await createdApiaryRow.click();
    await page.locator(".apiary-detail-panel").getByRole("button", { name: "Удалить пасеку" }).click();
    await expect(createdApiaryRow).toBeHidden();
  });
});
