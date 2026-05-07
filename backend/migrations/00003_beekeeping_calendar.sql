-- +goose Up
create table if not exists beekeeping_calendar_templates (
    id uuid primary key default gen_random_uuid(),
    code text not null unique,
    name text not null,
    description text not null default '',
    region_code text,
    climate_zone text,
    language text not null default 'ru',
    is_default boolean not null default false,
    is_active boolean not null default true,
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now()
);

create table if not exists beekeeping_periods (
    id uuid primary key default gen_random_uuid(),
    template_id uuid not null references beekeeping_calendar_templates(id) on delete cascade,
    code text not null,
    name text not null,
    description text not null default '',
    start_month integer not null,
    start_day integer not null,
    end_month integer not null,
    end_day integer not null,
    season text not null,
    priority integer not null default 100,
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now(),
    unique(template_id, code)
);

create table if not exists beekeeping_advice_templates (
    id uuid primary key default gen_random_uuid(),
    template_id uuid not null references beekeeping_calendar_templates(id) on delete cascade,
    period_code text,
    code text not null,
    title text not null,
    body text not null,
    category text not null,
    severity text not null default 'info',
    priority integer not null default 100,
    start_month integer,
    start_day integer,
    end_month integer,
    end_day integer,
    trigger_type text not null default 'calendar',
    trigger_config jsonb not null default '{}'::jsonb,
    action_label text,
    action_type text,
    is_user_dismissible boolean not null default true,
    is_active boolean not null default true,
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now(),
    unique(template_id, code)
);

create table if not exists beekeeping_task_templates (
    id uuid primary key default gen_random_uuid(),
    template_id uuid not null references beekeeping_calendar_templates(id) on delete cascade,
    period_code text,
    code text not null,
    title text not null,
    description text not null default '',
    category text not null,
    default_duration_minutes integer,
    severity text not null default 'notice',
    priority integer not null default 100,
    start_month integer,
    start_day integer,
    end_month integer,
    end_day integer,
    recurrence_rule text,
    weather_constraints jsonb not null default '{}'::jsonb,
    telemetry_constraints jsonb not null default '{}'::jsonb,
    is_active boolean not null default true,
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now(),
    unique(template_id, code)
);

create table if not exists apiary_tasks (
    id uuid primary key default gen_random_uuid(),
    apiary_id uuid not null references apiaries(id) on delete cascade,
    hive_id uuid references hives(id) on delete set null,
    source_template_id uuid references beekeeping_task_templates(id) on delete set null,
    title text not null,
    description text not null default '',
    category text not null,
    severity text not null default 'notice',
    status text not null default 'planned',
    due_at timestamptz,
    completed_at timestamptz,
    dismissed_at timestamptz,
    snoozed_until timestamptz,
    metadata jsonb not null default '{}'::jsonb,
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now()
);

create unique index if not exists apiary_tasks_template_due_unique_idx
    on apiary_tasks(apiary_id, source_template_id, due_at)
    where source_template_id is not null;
create index if not exists apiary_tasks_apiary_due_idx on apiary_tasks(apiary_id, due_at, status);

create table if not exists honey_plants (
    id uuid primary key default gen_random_uuid(),
    code text not null unique,
    name_ru text not null,
    name_uk text,
    latin_name text,
    description text not null default '',
    nectar_productivity_kg_ha numeric,
    is_major boolean not null default false,
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now()
);

create table if not exists honey_plant_bloom_periods (
    id uuid primary key default gen_random_uuid(),
    honey_plant_id uuid not null references honey_plants(id) on delete cascade,
    region_code text not null,
    start_month integer not null,
    start_day integer not null,
    end_month integer not null,
    end_day integer not null,
    confidence text not null default 'medium',
    source text,
    notes text,
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now(),
    unique(honey_plant_id, region_code, start_month, start_day, end_month, end_day)
);

create table if not exists apiary_calendar_settings (
    apiary_id uuid primary key references apiaries(id) on delete cascade,
    template_id uuid not null references beekeeping_calendar_templates(id),
    region_code text,
    climate_zone text,
    date_shift_days integer not null default 0,
    enable_weather_tips boolean not null default true,
    enable_bloom_tips boolean not null default true,
    enable_telemetry_tips boolean not null default true,
    enable_task_autogeneration boolean not null default true,
    created_at timestamptz not null default now(),
    updated_at timestamptz not null default now()
);

create table if not exists apiary_advice_states (
    apiary_id uuid not null references apiaries(id) on delete cascade,
    advice_code text not null,
    status text not null,
    snoozed_until timestamptz,
    updated_at timestamptz not null default now(),
    primary key(apiary_id, advice_code)
);

-- +goose Down
drop table if exists apiary_advice_states;
drop table if exists apiary_calendar_settings;
drop table if exists honey_plant_bloom_periods;
drop table if exists honey_plants;
drop table if exists apiary_tasks;
drop table if exists beekeeping_task_templates;
drop table if exists beekeeping_advice_templates;
drop table if exists beekeeping_periods;
drop table if exists beekeeping_calendar_templates;
