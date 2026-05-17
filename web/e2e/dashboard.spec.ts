import { expect, test } from "@playwright/test";
import { demoEmail, demoPassword, login, selectNorthApiary } from "./support";

test.describe("HiveMonitor dashboard", () => {
  test("switches auth modes and validates the login form", async ({ page }) => {
    await page.goto("/");
    await page.evaluate(() => localStorage.removeItem("hivemonitor.accessToken"));
    await page.reload();

    await expect(page.getByRole("heading", { name: "Вход" })).toBeVisible();
    await page.getByRole("button", { name: "Регистрация" }).click();
    await expect(page.getByRole("heading", { name: "Регистрация" })).toBeVisible();
    await expect(page.getByLabel("Имя")).toBeVisible();
    await page.getByRole("button", { name: "Вход" }).click();
    await expect(page.getByRole("heading", { name: "Вход" })).toBeVisible();

    await page.getByLabel("Email").fill("wrong@example.com");
    await page.getByLabel("Пароль").fill("wrong-password");
    await page.getByRole("button", { name: "Войти" }).click();
    await expect(page.locator(".notice.error")).toBeVisible();
  });

  test("opens the dashboard after demo login", async ({ page }) => {
    await login(page);
    await selectNorthApiary(page);

    await expect(page.getByRole("button", { name: "Пасеки", exact: true })).toBeVisible();
    await expect(page.getByText("Нераспознанные устройства").first()).toBeVisible();
    await expect(page.getByRole("heading", { name: "Погодные условия" })).toBeVisible();
    await expect(page.getByRole("heading", { name: "Советы / Подсказки" })).toBeVisible();
    await expect(page.getByText(/^Ульи \(/)).toBeVisible();
    const hiveRows = page.locator(".hive-table-row");
    await expect(hiveRows.first()).toBeVisible();
    const hiveRowCount = await hiveRows.count();
    expect(hiveRowCount).toBeGreaterThanOrEqual(4);

    const comparison = page.locator(".comparison-panel");
    await comparison.getByRole("button", { name: "30 дней" }).click();
    await expect(comparison.getByRole("img", { name: "Сравнительный график веса ульев" })).toBeVisible();
    const legendCount = await comparison.locator(".multi-legend button").count();
    const seriesCount = await comparison.locator(".series-line").count();
    expect(legendCount).toBeGreaterThanOrEqual(4);
    expect(seriesCount).toBe(legendCount);
    expect(hiveRowCount).toBeGreaterThanOrEqual(legendCount);
  });

  test("logs out and returns to the login screen", async ({ page }) => {
    await login(page);

    await page.getByRole("button", { name: "Выйти" }).click();
    await expect(page.getByRole("heading", { name: "Вход" })).toBeVisible();
    await expect(page.getByLabel("Email")).toBeVisible();
  });

  test("opens hive drawer from the hive list and closes it", async ({ page }) => {
    await login(page);
    await selectNorthApiary(page);

    const hiveRow = page.locator(".hive-table-row").first();
    await expect(hiveRow).toBeVisible();
    await hiveRow.click();

    await expect(page.locator(".hive-detail")).toBeVisible();
    await expect(page.getByRole("tab", { name: "Обзор" })).toBeVisible();
    await expect(page.getByRole("tab", { name: "Графики" })).toBeVisible();
    await page.getByRole("button", { name: "Закрыть карточку улья" }).click();
    await expect(page.locator(".hive-detail")).toBeHidden();
  });

  test("switches hive drawer tabs and chart periods", async ({ page }) => {
    await login(page);
    await selectNorthApiary(page);

    await page.locator(".hive-table-row").first().click();
    await expect(page.locator(".hive-detail")).toBeVisible();

    await page.getByRole("button", { name: "Открыть графики" }).click();
    await expect(page.getByRole("tab", { name: "Графики" })).toHaveAttribute("aria-selected", "true");
    await expect(page.getByRole("heading", { name: "Изменение веса за 1 день" })).toBeVisible();

    const detail = page.locator(".hive-detail");
    await detail.getByRole("button", { name: "10 дней" }).click();
    await expect(detail.getByRole("heading", { name: "Изменение веса за 10 дней" })).toBeVisible();
    await detail.getByRole("button", { name: "30 дней" }).click();
    await expect(detail.getByRole("heading", { name: "Изменение веса за 30 дней" })).toBeVisible();

    await page.getByRole("tab", { name: "История" }).click();
    await expect(page.getByRole("tab", { name: "История" })).toHaveAttribute("aria-selected", "true");
    await expect(page.getByRole("heading", { name: "Последние события" })).toBeVisible();
  });

  test("switches event tabs without changing task statuses", async ({ page }) => {
    await login(page);
    await selectNorthApiary(page);

    const eventsPanel = page.locator(".content-row .panel-card").filter({ hasText: "События" }).first();
    await expect(eventsPanel).toBeVisible();

    for (const tabName of ["Запланированные", "Просроченные", "Сезонные", "Важные"]) {
      await eventsPanel.getByRole("tab", { name: new RegExp(tabName) }).click();
      await expect(eventsPanel.getByRole("tab", { name: new RegExp(tabName) })).toHaveAttribute("aria-selected", "true");
    }
  });

  test("switches comparison chart periods and selects a hive from the chart legend", async ({ page }) => {
    await login(page);
    await selectNorthApiary(page);

    const comparison = page.locator(".comparison-panel");
    await expect(comparison).toBeVisible();
    await comparison.getByRole("button", { name: "10 дней" }).click();
    await expect(comparison.getByRole("button", { name: "10 дней" })).toHaveClass(/active/);
    await comparison.getByRole("button", { name: "30 дней" }).click();
    await expect(comparison.getByRole("button", { name: "30 дней" })).toHaveClass(/active/);

    const legendButton = comparison.locator(".multi-legend button").first();
    if (await legendButton.count() > 0) {
      await legendButton.click();
      await expect(page.locator(".hive-detail")).toBeVisible();
    }
  });

  test("opens and cancels the create hive form", async ({ page }) => {
    await login(page);
    await selectNorthApiary(page);

    const hivesPanel = page.locator(".hives-panel");
    const createHiveButton = hivesPanel.locator(".table-actions").getByRole("button", { name: "Улей", exact: true });
    await expect(createHiveButton).toBeEnabled();
    await createHiveButton.click();
    await expect(hivesPanel.getByPlaceholder("Название")).toBeVisible();
    await expect(hivesPanel.getByPlaceholder("Номер")).toBeVisible();
    await expect(hivesPanel.getByPlaceholder("Тип")).toBeVisible();
    await hivesPanel.getByRole("button", { name: "Отмена" }).click();
    await expect(hivesPanel.getByPlaceholder("Название")).toBeHidden();
  });

  test("expands tips and toggles all advice view", async ({ page }) => {
    await login(page);
    await selectNorthApiary(page);

    const tips = page.locator(".tips-card");
    await expect(tips).toBeVisible();
    const firstTip = tips.locator(".tip-line").first();
    await expect(firstTip).toBeVisible();
    await firstTip.locator(".tip-toggle").click();
    await expect(firstTip.locator(".tip-actions")).toBeVisible();

    const toggle = tips.getByRole("button", { name: /Все советы|Только актуальные/ });
    if (await toggle.count() > 0) {
      await toggle.click();
      await expect(tips.getByRole("button", { name: /Все советы|Только актуальные/ })).toBeVisible();
    }
  });

  test("opens apiaries screen and create apiary form", async ({ page }) => {
    await login(page);

    await page.getByRole("button", { name: "Пасеки", exact: true }).click();
    await expect(page.getByRole("heading", { name: "Пасеки" })).toBeVisible();
    await expect(page.getByText("Список пасек")).toBeVisible();

    await page.locator(".apiaries-screen").getByRole("button", { name: "Новая пасека" }).click();
    await expect(page.getByRole("heading", { name: "Создание пасеки" })).toBeVisible();
    await expect(page.getByLabel("Название")).toBeVisible();
    await expect(page.getByLabel("Широта")).toBeVisible();
    await page.getByRole("button", { name: "Отмена" }).click();
  });

  test("selects an apiary and opens its dashboard", async ({ page }) => {
    await login(page);

    await page.getByRole("button", { name: "Пасеки", exact: true }).click();
    await expect(page.getByRole("heading", { name: "Пасеки" })).toBeVisible();

    const apiaryRow = page.locator(".apiary-row").first();
    await expect(apiaryRow).toBeVisible();
    await apiaryRow.click();
    await expect(page.locator(".apiary-detail-panel").getByRole("button", { name: /Перейти к пасеке/ })).toBeVisible();
    await page.locator(".apiary-detail-panel").getByRole("button", { name: /Перейти к пасеке/ }).click();
    await expect(page.getByText("Панель сводок пасеки")).toBeVisible();
  });

  test("shows device assignment entry point without mutating data", async ({ page }) => {
    await login(page);
    await selectNorthApiary(page);

    const assignButton = page.getByRole("button", { name: "Привязать" }).first();
    const devicesPanel = page.locator(".devices-panel");
    await expect(devicesPanel).toBeVisible();
    if (!(await assignButton.isVisible())) {
      await expect(devicesPanel.getByText(/Новых устройств нет|Все устройства|Нераспознанные устройства/).first()).toBeVisible();
      return;
    }

    await expect(assignButton).toBeVisible();
    await assignButton.click();

    await expect(page.getByRole("heading", { name: "Выберите улей для устройства" })).toBeVisible();
    await expect(page.getByRole("button", { name: "Привязать устройство" })).toBeDisabled();
    await page.getByRole("dialog", { name: "Выберите улей для устройства" }).locator("select").selectOption({ index: 1 });
    await expect(page.getByRole("button", { name: "Привязать устройство" })).toBeEnabled();
    await expect(page.getByText("Что сделать со старой непривязанной телеметрией")).toBeVisible();
    await expect(page.getByText("Привязать к выбранному улью")).toBeVisible();
    await expect(page.getByText("Оставить непривязанной")).toBeVisible();
    await expect(page.getByText("Удалить старые данные")).toBeVisible();
    await page.getByRole("button", { name: "Отмена" }).click();
    await expect(page.getByRole("heading", { name: "Выберите улей для устройства" })).toBeHidden();
  });
});
