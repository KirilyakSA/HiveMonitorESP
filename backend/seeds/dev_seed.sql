begin;

create extension if not exists pgcrypto;

with seed_constants as (
    select
        '11111111-1111-1111-1111-111111111111'::uuid as user_id,
        '22222222-2222-2222-2222-222222222222'::uuid as organization_id,
        '33333333-3333-3333-3333-333333333331'::uuid as north_apiary_id,
        '33333333-3333-3333-3333-333333333332'::uuid as forest_apiary_id
)
insert into users (id, email, name, password_hash)
select user_id, 'demo@hivemonitor.local', 'Демо пасечник', crypt('password123', gen_salt('bf', 10))
from seed_constants
on conflict (email) do update set
    name = excluded.name,
    password_hash = excluded.password_hash,
    updated_at = now();

with seed_constants as (
    select
        '11111111-1111-1111-1111-111111111111'::uuid as user_id,
        '22222222-2222-2222-2222-222222222222'::uuid as organization_id
)
insert into organizations (id, name, created_at, updated_at)
select organization_id, 'Демо хозяйство HiveMonitor', now() - interval '12 days', now()
from seed_constants
on conflict (id) do update set
    name = excluded.name,
    updated_at = now();

with seed_constants as (
    select
        '11111111-1111-1111-1111-111111111111'::uuid as user_id,
        '22222222-2222-2222-2222-222222222222'::uuid as organization_id
)
insert into organization_members (id, organization_id, user_id, role, status)
select
    '22222222-1111-1111-1111-111111111111'::uuid,
    organization_id,
    user_id,
    'organization_owner',
    'active'
from seed_constants
on conflict (organization_id, user_id) do update set
    role = excluded.role,
    status = excluded.status;

insert into apiaries (
    id, organization_id, name, description, country, region, locality, address,
    location_description, latitude, longitude, timezone, status, created_at, updated_at
)
values
    (
        '33333333-3333-3333-3333-333333333331'::uuid,
        '22222222-2222-2222-2222-222222222222'::uuid,
        'Демо пасека Северная',
        'Основная тестовая пасека с несколькими ульями, устройствами и журналом событий.',
        'Украина',
        'Киевская область',
        'Бровары',
        'Учебная локация 1',
        'Рядом с лесополосой, открытая площадка для тестирования показателей.',
        50.5117,
        30.7909,
        'Europe/Kyiv',
        'active',
        now() - interval '10 days',
        now()
    ),
    (
        '33333333-3333-3333-3333-333333333332'::uuid,
        '22222222-2222-2222-2222-222222222222'::uuid,
        'Демо пасека Лесная',
        'Вторая пасека для проверки переключения между локациями.',
        'Украина',
        'Черниговская область',
        'Козелец',
        'Учебная локация 2',
        'Лесная точка, используется для проверки списка пасек.',
        50.9133,
        31.1214,
        'Europe/Kyiv',
        'active',
        now() - interval '9 days',
        now()
    )
on conflict (id) do update set
    name = excluded.name,
    description = excluded.description,
    country = excluded.country,
    region = excluded.region,
    locality = excluded.locality,
    address = excluded.address,
    location_description = excluded.location_description,
    latitude = excluded.latitude,
    longitude = excluded.longitude,
    timezone = excluded.timezone,
    status = excluded.status,
    updated_at = now();

insert into hives (
    id, apiary_id, name, number, type, frame_count, super_count, bee_breed,
    settled_at, queen_year, queen_breed, queen_status, status, notes, created_at, updated_at
)
values
    (
        '44444444-4444-4444-4444-444444444441'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        'Улей 1 - контрольный',
        '1',
        'Дадан',
        12,
        2,
        'Карника',
        now() - interval '420 days',
        2025,
        'Карника F1',
        'active',
        'active',
        'Сильная семья, нормальный набор веса.',
        now() - interval '8 days',
        now()
    ),
    (
        '44444444-4444-4444-4444-444444444442'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        'Улей 2 - требует внимания',
        '2',
        'Рута',
        10,
        1,
        'Бакфаст',
        now() - interval '300 days',
        2024,
        'Бакфаст B24',
        'active',
        'attention',
        'Падение веса за последние часы, нужен осмотр.',
        now() - interval '7 days',
        now()
    ),
    (
        '44444444-4444-4444-4444-444444444443'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        'Улей 3 - отводок',
        '3',
        'Дадан',
        8,
        0,
        'Украинская степная',
        now() - interval '80 days',
        2026,
        'Молодая матка',
        'active',
        'active',
        'Молодая семья, данных меньше, чем по основным ульям.',
        now() - interval '6 days',
        now()
    ),
    (
        '44444444-4444-4444-4444-444444444451'::uuid,
        '33333333-3333-3333-3333-333333333332'::uuid,
        'Лесной улей 1',
        'L1',
        'Лежак',
        16,
        0,
        'Карпатка',
        now() - interval '250 days',
        2025,
        'Карпатка',
        'active',
        'active',
        'Улей на второй демо-пасеке.',
        now() - interval '5 days',
        now()
    )
on conflict (id) do update set
    name = excluded.name,
    number = excluded.number,
    type = excluded.type,
    frame_count = excluded.frame_count,
    super_count = excluded.super_count,
    bee_breed = excluded.bee_breed,
    settled_at = excluded.settled_at,
    queen_year = excluded.queen_year,
    queen_breed = excluded.queen_breed,
    queen_status = excluded.queen_status,
    status = excluded.status,
    notes = excluded.notes,
    updated_at = now();

insert into devices (
    id, apiary_id, device_id, device_type, status, firmware_version, config_version,
    last_telemetry_at, last_status_at, expected_next_telemetry_at,
    missed_telemetry_count, telemetry_interval_minutes, created_at, updated_at
)
values
    (
        '55555555-5555-5555-5555-555555555551'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        'demo-esp8266-001',
        'hive_monitor',
        'assigned',
        '0.3.0-demo',
        7,
        now() - interval '5 minutes',
        now() - interval '4 minutes',
        now() + interval '25 minutes',
        0,
        30,
        now() - interval '8 days',
        now()
    ),
    (
        '55555555-5555-5555-5555-555555555552'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        'demo-esp8266-002',
        'hive_monitor',
        'assigned',
        '0.3.0-demo',
        6,
        now() - interval '65 minutes',
        now() - interval '64 minutes',
        now() - interval '35 minutes',
        1,
        30,
        now() - interval '7 days',
        now()
    ),
    (
        '55555555-5555-5555-5555-555555555553'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        'demo-esp32-003',
        'hive_monitor',
        'assigned',
        '0.3.0-demo',
        9,
        now() - interval '15 minutes',
        now() - interval '14 minutes',
        now() + interval '15 minutes',
        0,
        30,
        now() - interval '6 days',
        now()
    ),
    (
        '55555555-5555-5555-5555-555555555559'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        'demo-unassigned-009',
        'hive_monitor',
        'unassigned',
        '0.3.0-demo',
        1,
        now() - interval '12 minutes',
        now() - interval '11 minutes',
        now() + interval '18 minutes',
        0,
        30,
        now() - interval '1 day',
        now()
    ),
    (
        '55555555-5555-5555-5555-555555555558'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        'demo-unassigned-008',
        'hive_monitor',
        'unassigned',
        '0.3.0-demo',
        2,
        now() - interval '40 minutes',
        now() - interval '39 minutes',
        now() - interval '10 minutes',
        1,
        30,
        now() - interval '2 days',
        now()
    ),
    (
        '55555555-5555-5555-5555-555555555561'::uuid,
        '33333333-3333-3333-3333-333333333332'::uuid,
        'demo-forest-001',
        'hive_monitor',
        'assigned',
        '0.3.0-demo',
        4,
        now() - interval '20 minutes',
        now() - interval '19 minutes',
        now() + interval '10 minutes',
        0,
        30,
        now() - interval '5 days',
        now()
    )
on conflict (device_id) do update set
    apiary_id = excluded.apiary_id,
    device_type = excluded.device_type,
    status = excluded.status,
    firmware_version = excluded.firmware_version,
    config_version = excluded.config_version,
    last_telemetry_at = excluded.last_telemetry_at,
    last_status_at = excluded.last_status_at,
    expected_next_telemetry_at = excluded.expected_next_telemetry_at,
    missed_telemetry_count = excluded.missed_telemetry_count,
    telemetry_interval_minutes = excluded.telemetry_interval_minutes,
    updated_at = now();

delete from sensor_readings
where device_id in (
    '55555555-5555-5555-5555-555555555551'::uuid,
    '55555555-5555-5555-5555-555555555552'::uuid,
    '55555555-5555-5555-5555-555555555553'::uuid,
    '55555555-5555-5555-5555-555555555558'::uuid,
    '55555555-5555-5555-5555-555555555559'::uuid,
    '55555555-5555-5555-5555-555555555561'::uuid
);

delete from device_events
where device_id in (
    '55555555-5555-5555-5555-555555555551'::uuid,
    '55555555-5555-5555-5555-555555555552'::uuid,
    '55555555-5555-5555-5555-555555555553'::uuid,
    '55555555-5555-5555-5555-555555555558'::uuid,
    '55555555-5555-5555-5555-555555555559'::uuid,
    '55555555-5555-5555-5555-555555555561'::uuid
);

delete from raw_payloads
where device_id in (
    '55555555-5555-5555-5555-555555555551'::uuid,
    '55555555-5555-5555-5555-555555555552'::uuid,
    '55555555-5555-5555-5555-555555555553'::uuid,
    '55555555-5555-5555-5555-555555555558'::uuid,
    '55555555-5555-5555-5555-555555555559'::uuid,
    '55555555-5555-5555-5555-555555555561'::uuid
);

delete from device_assignments
where device_id in (
    '55555555-5555-5555-5555-555555555551'::uuid,
    '55555555-5555-5555-5555-555555555552'::uuid,
    '55555555-5555-5555-5555-555555555553'::uuid,
    '55555555-5555-5555-5555-555555555558'::uuid,
    '55555555-5555-5555-5555-555555555559'::uuid,
    '55555555-5555-5555-5555-555555555561'::uuid
);

insert into device_assignments (
    id, device_id, apiary_id, hive_id, scope_type, scope_id, role, assigned_at
)
values
    (
        '66666666-6666-6666-6666-666666666651'::uuid,
        '55555555-5555-5555-5555-555555555551'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        '44444444-4444-4444-4444-444444444441'::uuid,
        'hive',
        '44444444-4444-4444-4444-444444444441'::uuid,
        'primary_monitor',
        now() - interval '8 days'
    ),
    (
        '66666666-6666-6666-6666-666666666652'::uuid,
        '55555555-5555-5555-5555-555555555552'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        '44444444-4444-4444-4444-444444444442'::uuid,
        'hive',
        '44444444-4444-4444-4444-444444444442'::uuid,
        'primary_monitor',
        now() - interval '7 days'
    ),
    (
        '66666666-6666-6666-6666-666666666653'::uuid,
        '55555555-5555-5555-5555-555555555553'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        '44444444-4444-4444-4444-444444444443'::uuid,
        'hive',
        '44444444-4444-4444-4444-444444444443'::uuid,
        'primary_monitor',
        now() - interval '6 days'
    ),
    (
        '66666666-6666-6666-6666-666666666661'::uuid,
        '55555555-5555-5555-5555-555555555561'::uuid,
        '33333333-3333-3333-3333-333333333332'::uuid,
        '44444444-4444-4444-4444-444444444451'::uuid,
        'hive',
        '44444444-4444-4444-4444-444444444451'::uuid,
        'primary_monitor',
        now() - interval '5 days'
    );

with hive_devices as (
    select *
    from (values
        (
            '55555555-5555-5555-5555-555555555551'::uuid,
            '33333333-3333-3333-3333-333333333331'::uuid,
            '44444444-4444-4444-4444-444444444441'::uuid,
            48.5::double precision,
            34.7::double precision,
            58.0::double precision,
            88.0::double precision,
            0.03::double precision
        ),
        (
            '55555555-5555-5555-5555-555555555552'::uuid,
            '33333333-3333-3333-3333-333333333331'::uuid,
            '44444444-4444-4444-4444-444444444442'::uuid,
            42.0::double precision,
            33.2::double precision,
            64.0::double precision,
            73.0::double precision,
            -0.11::double precision
        ),
        (
            '55555555-5555-5555-5555-555555555553'::uuid,
            '33333333-3333-3333-3333-333333333331'::uuid,
            '44444444-4444-4444-4444-444444444443'::uuid,
            28.0::double precision,
            34.0::double precision,
            60.0::double precision,
            91.0::double precision,
            0.01::double precision
        ),
        (
            '55555555-5555-5555-5555-555555555561'::uuid,
            '33333333-3333-3333-3333-333333333332'::uuid,
            '44444444-4444-4444-4444-444444444451'::uuid,
            55.0::double precision,
            34.4::double precision,
            57.0::double precision,
            82.0::double precision,
            0.02::double precision
        )
    ) as v(device_id, apiary_id, hive_id, base_weight, base_temp, base_humidity, base_battery, trend)
),
samples as (
    select
        hd.*,
        gs.step,
        date_trunc('minute', now()) - (gs.step * interval '30 minutes') as measured_at
    from hive_devices hd
    cross join generate_series(47, 0, -1) as gs(step)
),
readings as (
    select
        device_id,
        apiary_id,
        hive_id,
        metric_type,
        value,
        unit,
        measured_at
    from samples
    cross join lateral (
        values
            ('weight', base_weight + ((47 - step) * trend) + sin(step::double precision / 3.0) * 0.45, 'kg'),
            ('weight_change', trend * 30.0 + sin(step::double precision / 4.0) * 0.2, 'kg'),
            ('temperature', base_temp + sin(step::double precision / 5.0) * 0.8, '°C'),
            ('humidity', base_humidity + cos(step::double precision / 6.0) * 3.0, '%'),
            ('battery_percent', greatest(12.0, base_battery - ((47 - step)::double precision * 0.08)), '%'),
            ('battery_voltage', 3.55 + (base_battery / 100.0) * 0.65 - ((47 - step)::double precision * 0.001), 'V'),
            ('rssi', -55.0 - abs(sin(step::double precision / 4.0) * 8.0), 'dBm'),
            ('free_heap', 43000.0 - (step % 6) * 280.0, 'bytes'),
            ('hive_opened', case when step in (8, 31) and device_id = '55555555-5555-5555-5555-555555555552'::uuid then 1.0 else 0.0 end, '')
    ) as metrics(metric_type, value, unit)
)
insert into sensor_readings (
    device_id, apiary_id, hive_id, metric_type, value, unit, measured_at, received_at
)
select device_id, apiary_id, hive_id, metric_type, value, unit, measured_at, measured_at + interval '10 seconds'
from readings;

with unassigned_samples as (
    select
        device_id,
        base_weight,
        base_temp,
        base_humidity,
        base_battery,
        '33333333-3333-3333-3333-333333333331'::uuid as apiary_id,
        gs.step,
        date_trunc('minute', now()) - (gs.step * interval '30 minutes') as measured_at
    from (values
        (
            '55555555-5555-5555-5555-555555555559'::uuid,
            18.0::double precision,
            26.5::double precision,
            66.0::double precision,
            96.0::double precision
        ),
        (
            '55555555-5555-5555-5555-555555555558'::uuid,
            31.5::double precision,
            29.4::double precision,
            70.0::double precision,
            54.0::double precision
        )
    ) as devices(device_id, base_weight, base_temp, base_humidity, base_battery)
    cross join generate_series(15, 0, -1) as gs(step)
),
readings as (
    select
        device_id,
        apiary_id,
        null::uuid as hive_id,
        metric_type,
        value,
        unit,
        measured_at
    from unassigned_samples
    cross join lateral (
        values
            ('weight', base_weight + sin(step::double precision / 2.0) * 0.35 - ((15 - step)::double precision * 0.03), 'kg'),
            ('temperature', base_temp + sin(step::double precision / 3.0), '°C'),
            ('humidity', base_humidity + cos(step::double precision / 3.0) * 2.0, '%'),
            ('battery_percent', greatest(5.0, base_battery - ((15 - step)::double precision * 0.12)), '%')
    ) as metrics(metric_type, value, unit)
)
insert into sensor_readings (
    device_id, apiary_id, hive_id, metric_type, value, unit, measured_at, received_at
)
select device_id, apiary_id, hive_id, metric_type, value, unit, measured_at, measured_at + interval '10 seconds'
from readings;

insert into device_events (
    id, device_id, apiary_id, hive_id, event_type, message, ok, command, occurred_at, received_at
)
values
    (
        '77777777-7777-7777-7777-777777777701'::uuid,
        '55555555-5555-5555-5555-555555555551'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        '44444444-4444-4444-4444-444444444441'::uuid,
        'measurement',
        'Плановое измерение выполнено успешно.',
        true,
        'measure',
        now() - interval '35 minutes',
        now() - interval '34 minutes'
    ),
    (
        '77777777-7777-7777-7777-777777777702'::uuid,
        '55555555-5555-5555-5555-555555555552'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        '44444444-4444-4444-4444-444444444442'::uuid,
        'hive_opened',
        'Зафиксировано открытие улья. Требуется подтвердить выполнение работ.',
        true,
        '',
        now() - interval '4 hours',
        now() - interval '4 hours' + interval '1 minute'
    ),
    (
        '77777777-7777-7777-7777-777777777703'::uuid,
        '55555555-5555-5555-5555-555555555552'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        '44444444-4444-4444-4444-444444444442'::uuid,
        'alert',
        'Вес снизился быстрее обычного. Проверьте наличие медосбора, роения или вмешательства.',
        false,
        '',
        now() - interval '2 hours',
        now() - interval '2 hours' + interval '1 minute'
    ),
    (
        '77777777-7777-7777-7777-777777777704'::uuid,
        '55555555-5555-5555-5555-555555555553'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        '44444444-4444-4444-4444-444444444443'::uuid,
        'comment',
        'Отводок развивается стабильно, температура в норме.',
        true,
        '',
        now() - interval '1 day',
        now() - interval '1 day' + interval '2 minutes'
    ),
    (
        '77777777-7777-7777-7777-777777777705'::uuid,
        '55555555-5555-5555-5555-555555555559'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        null,
        'device_seen',
        'Новое устройство обнаружено на пасеке и ожидает привязки к улью.',
        true,
        '',
        now() - interval '20 minutes',
        now() - interval '19 minutes'
    ),
    (
        '77777777-7777-7777-7777-777777777706'::uuid,
        '55555555-5555-5555-5555-555555555558'::uuid,
        '33333333-3333-3333-3333-333333333331'::uuid,
        null,
        'device_seen',
        'Второе новое устройство передает историю, но еще не привязано к улью.',
        true,
        '',
        now() - interval '45 minutes',
        now() - interval '44 minutes'
    )
on conflict (id) do update set
    message = excluded.message,
    ok = excluded.ok,
    occurred_at = excluded.occurred_at,
    received_at = excluded.received_at;

commit;
