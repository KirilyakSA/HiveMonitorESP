-- +goose Up
create table device_commands (
    id uuid primary key default gen_random_uuid(),
    apiary_id uuid not null references apiaries(id) on delete cascade,
    device_id uuid not null references devices(id) on delete cascade,
    device_public_id text not null,
    requested_by uuid references users(id) on delete set null,
    command text not null,
    payload jsonb not null default '{}'::jsonb,
    status text not null default 'created',
    mqtt_topic text not null default '',
    published_topics text[] not null default '{}',
    error_message text not null default '',
    result jsonb,
    created_at timestamptz not null default now(),
    published_at timestamptz,
    acknowledged_at timestamptz,
    expires_at timestamptz not null default now() + interval '10 minutes',
    updated_at timestamptz not null default now()
);

create index device_commands_apiary_created_idx on device_commands(apiary_id, created_at desc);
create index device_commands_device_created_idx on device_commands(device_id, created_at desc);
create index device_commands_status_expires_idx on device_commands(status, expires_at);

-- +goose Down
drop table if exists device_commands;
