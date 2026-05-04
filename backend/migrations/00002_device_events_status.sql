-- +goose Up
alter table devices
    add column if not exists last_status_at timestamptz;

create table if not exists device_events (
    id uuid primary key default gen_random_uuid(),
    device_id uuid references devices(id) on delete set null,
    apiary_id uuid references apiaries(id) on delete set null,
    hive_id uuid references hives(id) on delete set null,
    event_type text not null default '',
    message text not null default '',
    ok boolean,
    command text not null default '',
    occurred_at timestamptz not null,
    received_at timestamptz not null default now(),
    raw_payload_id uuid references raw_payloads(id) on delete set null
);

create index if not exists device_events_apiary_occurred_idx on device_events(apiary_id, occurred_at desc);
create index if not exists device_events_hive_occurred_idx on device_events(hive_id, occurred_at desc);

-- +goose Down
drop table if exists device_events;

alter table devices
    drop column if exists last_status_at;
