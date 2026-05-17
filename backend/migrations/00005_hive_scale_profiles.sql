-- +goose Up
create table hive_scale_profiles (
    hive_id uuid primary key references hives(id) on delete cascade,
    apiary_id uuid not null references apiaries(id) on delete cascade,
    empty_hive_tare_kg double precision,
    active_tare_kg double precision not null default 0,
    super_tares jsonb not null default '[]'::jsonb,
    updated_by uuid references users(id) on delete set null,
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now()
);

create table hive_tare_events (
    id uuid primary key default gen_random_uuid(),
    hive_id uuid not null references hives(id) on delete cascade,
    apiary_id uuid not null references apiaries(id) on delete cascade,
    device_id uuid references devices(id) on delete set null,
    command_id uuid references device_commands(id) on delete set null,
    tare_kind text not null,
    super_index int,
    measured_raw_weight_kg double precision not null,
    previous_active_tare_kg double precision not null default 0,
    new_active_tare_kg double precision not null,
    comment text not null default '',
    metadata jsonb not null default '{}'::jsonb,
    created_by uuid references users(id) on delete set null,
    created_at timestamptz not null default now()
);

create index hive_tare_events_hive_created_idx on hive_tare_events(hive_id, created_at desc);

-- +goose Down
drop table if exists hive_tare_events;
drop table if exists hive_scale_profiles;
