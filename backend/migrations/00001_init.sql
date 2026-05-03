-- +goose Up
create extension if not exists pgcrypto;

create table users (
    id uuid primary key default gen_random_uuid(),
    email text not null unique,
    name text not null default '',
    password_hash text not null,
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now()
);

create table organizations (
    id uuid primary key default gen_random_uuid(),
    name text not null,
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now()
);

create table organization_members (
    id uuid primary key default gen_random_uuid(),
    organization_id uuid not null references organizations(id) on delete cascade,
    user_id uuid not null references users(id) on delete cascade,
    role text not null,
    status text not null default 'active',
    invited_by uuid references users(id),
    created_at timestamptz not null default now(),
    unique (organization_id, user_id)
);

create table apiaries (
    id uuid primary key default gen_random_uuid(),
    organization_id uuid not null references organizations(id) on delete cascade,
    name text not null,
    description text not null default '',
    country text not null default '',
    region text not null default '',
    locality text not null default '',
    address text not null default '',
    location_description text not null default '',
    latitude double precision,
    longitude double precision,
    timezone text not null default 'UTC',
    status text not null default 'active',
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now()
);

create table hives (
    id uuid primary key default gen_random_uuid(),
    apiary_id uuid not null references apiaries(id) on delete cascade,
    name text not null,
    number text not null default '',
    type text not null default '',
    frame_count integer,
    super_count integer,
    bee_breed text not null default '',
    settled_at timestamptz,
    queen_year integer,
    queen_breed text not null default '',
    queen_status text not null default '',
    status text not null default 'active',
    notes text not null default '',
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now()
);

create table devices (
    id uuid primary key default gen_random_uuid(),
    apiary_id uuid references apiaries(id) on delete set null,
    device_id text not null unique,
    device_type text not null default 'hive_monitor',
    status text not null default 'unassigned',
    firmware_version text not null default '',
    config_version integer,
    last_telemetry_at timestamptz,
    expected_next_telemetry_at timestamptz,
    missed_telemetry_count integer not null default 0,
    telemetry_interval_minutes integer not null default 30,
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now()
);

create table device_assignments (
    id uuid primary key default gen_random_uuid(),
    device_id uuid not null references devices(id) on delete cascade,
    apiary_id uuid not null references apiaries(id) on delete cascade,
    hive_id uuid references hives(id) on delete set null,
    scope_type text not null,
    scope_id uuid not null,
    role text not null default 'primary_monitor',
    assigned_at timestamptz not null default now(),
    unassigned_at timestamptz
);

create table raw_payloads (
    id uuid primary key default gen_random_uuid(),
    device_id uuid references devices(id) on delete set null,
    apiary_id uuid references apiaries(id) on delete set null,
    topic text not null,
    payload jsonb not null,
    received_at timestamptz not null default now()
);

create table sensor_readings (
    id uuid primary key default gen_random_uuid(),
    device_id uuid not null references devices(id) on delete cascade,
    apiary_id uuid references apiaries(id) on delete set null,
    hive_id uuid references hives(id) on delete set null,
    metric_type text not null,
    value double precision not null,
    unit text not null default '',
    measured_at timestamptz not null,
    received_at timestamptz not null default now(),
    raw_payload_id uuid references raw_payloads(id) on delete set null
);

create index organization_members_user_idx on organization_members(user_id);
create index apiaries_organization_idx on apiaries(organization_id);
create index hives_apiary_idx on hives(apiary_id);
create index devices_apiary_idx on devices(apiary_id);
create index device_assignments_device_active_idx on device_assignments(device_id) where unassigned_at is null;
create index device_assignments_hive_idx on device_assignments(hive_id);
create index sensor_readings_device_measured_idx on sensor_readings(device_id, measured_at desc);
create index sensor_readings_hive_measured_idx on sensor_readings(hive_id, measured_at desc);
create index sensor_readings_scope_metric_idx on sensor_readings(hive_id, metric_type, measured_at desc);

-- +goose Down
drop table if exists sensor_readings;
drop table if exists raw_payloads;
drop table if exists device_assignments;
drop table if exists devices;
drop table if exists hives;
drop table if exists apiaries;
drop table if exists organization_members;
drop table if exists organizations;
drop table if exists users;
