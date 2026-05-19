-- +goose Up
create table firmware_releases (
    id uuid primary key default gen_random_uuid(),
    device_type text not null default 'hive_monitor',
    version text not null,
    channel text not null default 'stable',
    artifact_url text not null,
    checksum_sha256 text not null,
    size_bytes bigint,
    release_notes text not null default '',
    is_active boolean not null default true,
    created_by uuid references users(id) on delete set null,
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now(),
    unique (device_type, version, channel)
);

create index firmware_releases_device_type_channel_idx
    on firmware_releases(device_type, channel, created_at desc);

-- +goose Down
drop table if exists firmware_releases;
