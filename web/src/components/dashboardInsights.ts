import type { Apiary, Hive, SensorReading, WeatherReading } from "../api";

export type WeatherSnapshot = {
  temperatureC: number;
  condition: string;
  windMps: number;
  humidityPercent: number;
  pressureHPa: number;
  rainExpected: boolean;
  nightTemperatureC: number;
  provider: string;
  measuredAt?: string;
};

export type WeatherInsight = {
  severity: "info" | "notice" | "warning";
  title: string;
  body: string;
};

export function weatherForApiary(apiary?: Apiary): WeatherSnapshot {
  const base = `${apiary?.id ?? ""}${apiary?.region ?? ""}`;
  const seed = [...base].reduce((sum, char) => sum + char.charCodeAt(0), 0);
  const rainExpected = seed % 3 === 0;
  return {
    temperatureC: rainExpected ? 18 : 22,
    condition: rainExpected ? "Ожидается дождь" : "Переменная облачность",
    windMps: rainExpected ? 9 : 6,
    humidityPercent: rainExpected ? 72 : 58,
    pressureHPa: rainExpected ? 1008 : 1015,
    rainExpected,
    nightTemperatureC: seed % 4 === 0 ? 6 : 11,
    provider: "ui_mock"
  };
}

export function weatherFromReadings(apiary: Apiary | undefined, readings: WeatherReading[]): WeatherSnapshot {
  if (readings.length === 0) return weatherForApiary(apiary);
  const latest = [...readings].sort((a, b) => new Date(b.measured_at).getTime() - new Date(a.measured_at).getTime())[0];
  const nightTemperature = readings.reduce<number | null>((min, item) => {
    if (typeof item.temperature_c !== "number") return min;
    return min === null ? item.temperature_c : Math.min(min, item.temperature_c);
  }, null);
  return {
    temperatureC: Math.round(latest.temperature_c ?? 0),
    condition: latest.condition || "Погодные данные",
    windMps: Math.round(latest.wind_speed_mps ?? 0),
    humidityPercent: Math.round(latest.humidity_percent ?? 0),
    pressureHPa: Math.round(latest.pressure_hpa ?? 0),
    rainExpected: readings.some((item) => (item.rain_mm ?? 0) > 0),
    nightTemperatureC: Math.round(nightTemperature ?? latest.temperature_c ?? 0),
    provider: latest.provider,
    measuredAt: latest.measured_at
  };
}

export function buildWeatherInsights(hives: Hive[], snapshots: Record<string, SensorReading[]>, weather: WeatherSnapshot): WeatherInsight[] {
  const deltas = hives
    .map((hive) => ({ hive, delta: latestMetric(snapshots[hive.id] ?? [], "weight_change")?.value }))
    .filter((item): item is { hive: Hive; delta: number } => typeof item.delta === "number" && Number.isFinite(item.delta));

  const insights: WeatherInsight[] = [];
  if (weather.rainExpected || weather.windMps >= 8 || weather.nightTemperatureC <= 6) {
    insights.push({
      severity: "notice",
      title: "Погода может влиять на вес",
      body: "Если прирост слабый, сначала учитывайте дождь, ветер или ночное похолодание, а затем сравнивайте ульи между собой."
    });
  }

  if (deltas.length > 1) {
    const median = medianValue(deltas.map((item) => item.delta));
    const lagging = deltas
      .filter((item) => item.delta < median - 0.5)
      .sort((a, b) => a.delta - b.delta)
      .slice(0, 2);
    const favorable = weather.temperatureC >= 18 && weather.temperatureC <= 28 && weather.windMps < 8 && !weather.rainExpected;
    if (favorable && lagging.length > 0) {
      insights.push({
        severity: "warning",
        title: "Есть отстающие ульи при нормальной погоде",
        body: `${lagging.map((item) => item.hive.name || `Улей ${item.hive.number}`).join(", ")} набирают вес заметно хуже медианы пасеки. Стоит проверить место, силу семьи и матку.`
      });
    }

    const average = deltas.reduce((sum, item) => sum + item.delta, 0) / deltas.length;
    if (!favorable && average < 0.2) {
      insights.push({
        severity: "info",
        title: "Слабый общий прирост объясним погодой",
        body: "Средний прирост по пасеке низкий, но погодные условия тоже неблагоприятные. Сравнивайте этот день с соседними сухими днями."
      });
    }
  }

  if (insights.length === 0) {
    insights.push({
      severity: "info",
      title: "Погодный фон спокойный",
      body: "Сейчас погода не выглядит главной причиной отклонений. Если улей отстает, полезнее сравнить его с соседними ульями."
    });
  }
  return insights.slice(0, 3);
}

export function weatherWarnings(weather: WeatherSnapshot): string[] {
  const warnings: string[] = [];
  if (weather.nightTemperatureC <= 6) warnings.push(`Похолодание ночью до ${weather.nightTemperatureC}°C`);
  if (weather.rainExpected) warnings.push("Ожидается дождь вечером");
  if (weather.windMps >= 8) warnings.push(`Порывы ветра до ${weather.windMps + 3} м/с`);
  return warnings.length > 0 ? warnings : ["Существенных погодных предупреждений нет"];
}

function latestMetric(readings: SensorReading[], metricType: string) {
  return readings.find((reading) => reading.metric_type === metricType);
}

function medianValue(values: number[]) {
  const sorted = [...values].sort((a, b) => a - b);
  const middle = Math.floor(sorted.length / 2);
  return sorted.length % 2 === 0 ? (sorted[middle - 1] + sorted[middle]) / 2 : sorted[middle];
}
