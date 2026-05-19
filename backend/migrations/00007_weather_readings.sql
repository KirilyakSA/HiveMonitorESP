-- +goose Up
create table weather_readings (
    id uuid primary key default gen_random_uuid(),
    apiary_id uuid not null references apiaries(id) on delete cascade,
    provider text not null default 'mock',
    source_type text not null default 'external_api',
    temperature_c double precision,
    humidity_percent double precision,
    pressure_hpa double precision,
    wind_speed_mps double precision,
    rain_mm double precision,
    condition text not null default '',
    measured_at timestamptz not null,
    received_at timestamptz not null default now(),
    raw_payload jsonb not null default '{}'::jsonb
);

create index weather_readings_apiary_measured_idx on weather_readings(apiary_id, measured_at desc);

-- +goose Down
drop table if exists weather_readings;
