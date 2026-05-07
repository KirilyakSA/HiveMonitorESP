-- HiveMonitor beekeeping calendar seed.
-- Idempotent: safe to run multiple times after migrations are applied.

insert into beekeeping_calendar_templates (
    code, name, description, region_code, climate_zone, language, is_default, is_active
) values (
    'ua_forest_steppe',
    'Украина: лесостепь / умеренный климат',
    'Стартовый адаптируемый календарь пасечных работ для Украины. Сроки являются подсказками и могут сдвигаться настройками пасеки.',
    'ua_forest_steppe',
    'temperate',
    'ru',
    true,
    true
) on conflict (code) do update set
    name = excluded.name,
    description = excluded.description,
    region_code = excluded.region_code,
    climate_zone = excluded.climate_zone,
    is_default = true,
    is_active = true,
    updated_at = now();

insert into beekeeping_periods (
    template_id, code, name, description, start_month, start_day, end_month, end_day, season, priority
)
select t.id, v.code, v.name, v.description, v.start_month, v.start_day, v.end_month, v.end_day, v.season, v.priority
from beekeeping_calendar_templates t
cross join (values
    ('wintering', 'Зимовка', 'Минимальное вмешательство, контроль летков, вентиляции и кормовых запасов по весу.', 12, 1, 2, 28, 'winter', 10),
    ('early_spring', 'Ранняя весна', 'Контроль кормов, воды и первая осторожная оценка состояния семей.', 3, 1, 4, 10, 'spring', 20),
    ('spring_build_up', 'Весеннее развитие', 'Ревизия, расширение гнезда, контроль расплода, матки и силы семьи.', 4, 1, 5, 31, 'spring', 30),
    ('swarm_control', 'Контроль роения', 'Период повышенного внимания к роевому состоянию и наличию места в ульях.', 5, 1, 7, 10, 'spring_summer', 40),
    ('main_honey_flow_preparation', 'Подготовка к главному взятку', 'Проверка магазинов, рамок, места под мед и оборудования.', 5, 20, 6, 20, 'summer', 50),
    ('main_honey_flow', 'Главный медосбор', 'Контроль ежедневного прироста веса, вентиляции, воды и свободного места.', 6, 1, 7, 31, 'summer', 60),
    ('sunflower_late_flow', 'Подсолнечник / поздний летний взяток', 'Подготовка к позднему взятку, контроль магазинов и прироста веса.', 6, 21, 8, 15, 'summer', 70),
    ('honey_harvest', 'Откачка меда', 'Проверка зрелости меда, отбор рамок и уборка оборудования.', 7, 1, 8, 31, 'summer', 80),
    ('post_harvest', 'После медосбора', 'Оценка силы семей, контроль клеща, начало подготовки к зимовке.', 8, 1, 9, 20, 'autumn', 90),
    ('winter_preparation', 'Подготовка к зиме', 'Кормовые запасы, формирование гнезда, защита от мышей, сокращение летков.', 9, 1, 10, 20, 'autumn', 100),
    ('late_autumn', 'Поздняя осень', 'Проверка крышек, ветровой защиты, вентиляции и планирование закупок.', 11, 1, 11, 30, 'late_autumn', 110)
) as v(code, name, description, start_month, start_day, end_month, end_day, season, priority)
where t.code = 'ua_forest_steppe'
on conflict (template_id, code) do update set
    name = excluded.name,
    description = excluded.description,
    start_month = excluded.start_month,
    start_day = excluded.start_day,
    end_month = excluded.end_month,
    end_day = excluded.end_day,
    season = excluded.season,
    priority = excluded.priority,
    updated_at = now();

insert into beekeeping_advice_templates (
    template_id, period_code, code, title, body, category, severity, priority,
    start_month, start_day, end_month, end_day, trigger_type, trigger_config,
    action_label, action_type, is_user_dismissible, is_active
)
select t.id, v.period_code, v.code, v.title, v.body, v.category, v.severity, v.priority,
    v.start_month, v.start_day, v.end_month, v.end_day, v.trigger_type, v.trigger_config::jsonb,
    v.action_label, v.action_type, true, true
from beekeeping_calendar_templates t
cross join (values
    ('early_spring', 'spring_check_hive_weight', 'Проверьте вес ульев после зимы', 'Весной семьи активно расходуют корм на развитие расплода. Проверьте вес ульев и наличие кормов.', 'feeding', 'warning', 20, 3, 1, 4, 10, 'calendar', '{}', 'Запланировать проверку', 'create_task'),
    ('spring_build_up', 'spring_full_inspection', 'Запланируйте весеннюю ревизию', 'В теплый безветренный день проверьте расплод, матку, силу семьи и наличие кормов.', 'inspection', 'notice', 30, 4, 1, 5, 15, 'calendar', '{}', 'Запланировать ревизию', 'create_task'),
    ('swarm_control', 'swarm_control', 'Контроль роевого состояния', 'В период активного роста семей проверяйте наличие маточников и достаточность места в улье.', 'swarm_control', 'warning', 25, 5, 1, 7, 10, 'calendar', '{}', 'Добавить работу', 'create_task'),
    ('main_honey_flow_preparation', 'add_supers_before_honey_flow', 'Проверьте магазины перед главным взятком', 'Сильным семьям рекомендуется заранее добавить магазины, чтобы не ограничивать сбор нектара.', 'add_supers', 'notice', 30, 5, 20, 6, 20, 'calendar', '{}', 'Запланировать работу', 'create_task'),
    ('main_honey_flow', 'sunflower_bloom_add_supers', 'Скоро цветение подсолнечника', 'Проверьте сильные семьи и не забудьте добавить магазины перед началом активного взятка.', 'bloom', 'notice', 35, 6, 15, 7, 25, 'bloom', '{"plant_code":"sunflower"}', 'Запланировать работу', 'create_task'),
    ('main_honey_flow', 'summer_water_and_ventilation', 'Жара: проверьте воду и вентиляцию', 'В жаркую погоду обеспечьте доступ к воде и проверьте вентиляцию ульев.', 'weather', 'warning', 25, null, null, null, null, 'weather', '{"min_temperature_c":30}', 'Подробнее', 'details'),
    ('post_harvest', 'post_harvest_varroa_check', 'Проверьте клеща после медосбора', 'После основного медосбора оцените уровень клеща Варроа и запланируйте действия согласно принятой методике и инструкции препарата.', 'varroa', 'warning', 25, 8, 1, 9, 20, 'calendar', '{}', 'Запланировать проверку', 'create_task'),
    ('winter_preparation', 'winter_feed_stores', 'Проверьте кормовые запасы на зиму', 'Перед зимовкой проверьте, достаточно ли кормов и правильно ли сформировано гнездо.', 'wintering', 'warning', 20, 9, 1, 10, 20, 'calendar', '{}', 'Запланировать проверку', 'create_task')
) as v(period_code, code, title, body, category, severity, priority, start_month, start_day, end_month, end_day, trigger_type, trigger_config, action_label, action_type)
where t.code = 'ua_forest_steppe'
on conflict (template_id, code) do update set
    title = excluded.title,
    body = excluded.body,
    category = excluded.category,
    severity = excluded.severity,
    priority = excluded.priority,
    start_month = excluded.start_month,
    start_day = excluded.start_day,
    end_month = excluded.end_month,
    end_day = excluded.end_day,
    trigger_type = excluded.trigger_type,
    trigger_config = excluded.trigger_config,
    action_label = excluded.action_label,
    action_type = excluded.action_type,
    is_active = true,
    updated_at = now();

insert into beekeeping_task_templates (
    template_id, period_code, code, title, description, category, default_duration_minutes,
    severity, priority, start_month, start_day, end_month, end_day, recurrence_rule,
    weather_constraints, telemetry_constraints, is_active
)
select t.id, v.period_code, v.code, v.title, v.description, v.category, v.duration_minutes,
    v.severity, v.priority, v.start_month, v.start_day, v.end_month, v.end_day, v.recurrence_rule,
    '{}'::jsonb, '{}'::jsonb, true
from beekeeping_calendar_templates t
cross join (values
    ('wintering', 'check_entrances_ventilation', 'Проверить летки и вентиляцию', 'Осмотрите летки без открытия ульев: они не должны быть заблокированы снегом или мертвыми пчелами.', 'wintering', 30, 'notice', 60, 12, 10, 2, 20, 'FREQ=MONTHLY;INTERVAL=1'),
    ('early_spring', 'spring_feed_check', 'Оценить вес ульев и наличие корма', 'Проверьте вес ульев и при необходимости запланируйте безопасную проверку кормов.', 'feeding', 45, 'warning', 20, 3, 10, 4, 10, null),
    ('spring_build_up', 'spring_full_inspection_task', 'Полная весенняя ревизия', 'Проверьте расплод, матку, силу семьи, кормовые запасы и состояние гнезда в теплый безветренный день.', 'inspection', 90, 'notice', 30, 4, 10, 5, 15, null),
    ('swarm_control', 'swarm_control_task', 'Контроль роевого состояния', 'Проверьте наличие маточников и достаточность места в ульях.', 'swarm_control', 60, 'warning', 25, 5, 10, 7, 10, 'FREQ=WEEKLY;INTERVAL=1'),
    ('main_honey_flow_preparation', 'add_supers_task', 'Добавить магазины сильным семьям', 'Проверьте место под мед и заранее добавьте магазины сильным семьям.', 'add_supers', 60, 'notice', 30, 5, 25, 6, 20, null),
    ('main_honey_flow', 'honey_flow_weight_control', 'Контроль прироста веса', 'Сравните динамику веса ульев и проверьте ульи, которые заметно отстают.', 'honey_flow', 30, 'notice', 45, 6, 5, 7, 31, 'FREQ=WEEKLY;INTERVAL=1'),
    ('honey_harvest', 'honey_harvest_task', 'Проверить запечатку и отобрать рамки', 'Откачивайте только зрелый запечатанный мед и не забирайте весь корм у семьи.', 'harvest', 120, 'notice', 50, 7, 15, 8, 31, null),
    ('post_harvest', 'varroa_check_task', 'Проверить уровень клеща после медосбора', 'Оцените уровень клеща Варроа и действуйте согласно принятой методике и инструкции препарата.', 'varroa', 60, 'warning', 25, 8, 10, 9, 20, null),
    ('winter_preparation', 'winter_feed_stores_task', 'Проверить кормовые запасы на зиму', 'Проверьте кормовые запасы, сформируйте зимнее гнездо и защитите ульи от мышей.', 'wintering', 90, 'warning', 20, 9, 10, 10, 20, null),
    ('late_autumn', 'late_autumn_site_check', 'Осмотреть пасеку без открытия ульев', 'Проверьте крышки, защиту от ветра и вентиляцию без разборки гнезда.', 'maintenance', 45, 'notice', 60, 11, 10, 11, 30, null)
) as v(period_code, code, title, description, category, duration_minutes, severity, priority, start_month, start_day, end_month, end_day, recurrence_rule)
where t.code = 'ua_forest_steppe'
on conflict (template_id, code) do update set
    title = excluded.title,
    description = excluded.description,
    category = excluded.category,
    default_duration_minutes = excluded.default_duration_minutes,
    severity = excluded.severity,
    priority = excluded.priority,
    start_month = excluded.start_month,
    start_day = excluded.start_day,
    end_month = excluded.end_month,
    end_day = excluded.end_day,
    recurrence_rule = excluded.recurrence_rule,
    is_active = true,
    updated_at = now();

insert into honey_plants (code, name_ru, name_uk, latin_name, is_major)
values
    ('willow', 'Ива / верба', 'Верба', 'Salix', false),
    ('orchards', 'Сады: вишня, слива, груша, яблоня', 'Сади', null, false),
    ('rape', 'Рапс', 'Ріпак', 'Brassica napus', true),
    ('white_acacia', 'Акация белая', 'Робінія звичайна / біла акація', 'Robinia pseudoacacia', true),
    ('clover', 'Клевер', 'Конюшина', 'Trifolium', false),
    ('phacelia', 'Фацелия', 'Фацелія', 'Phacelia', false),
    ('sweet_clover', 'Донник', 'Буркун', 'Melilotus', false),
    ('linden', 'Липа', 'Липа', 'Tilia', true),
    ('buckwheat', 'Гречиха', 'Гречка', 'Fagopyrum', true),
    ('sunflower', 'Подсолнечник', 'Соняшник', 'Helianthus', true),
    ('heather', 'Вереск', 'Верес', 'Calluna vulgaris', false),
    ('goldenrod', 'Золотарник', 'Золотушник', 'Solidago', false)
on conflict (code) do update set
    name_ru = excluded.name_ru,
    name_uk = excluded.name_uk,
    latin_name = excluded.latin_name,
    is_major = excluded.is_major,
    updated_at = now();

insert into honey_plant_bloom_periods (
    honey_plant_id, region_code, start_month, start_day, end_month, end_day, confidence, source, notes
)
select p.id, v.region_code, v.start_month, v.start_day, v.end_month, v.end_day, 'medium', 'HiveMonitor seed', 'Стартовый период, требует адаптации под местность.'
from honey_plants p
join (values
    ('willow', 'ua_forest_steppe', 3, 15, 4, 20),
    ('orchards', 'ua_forest_steppe', 4, 10, 5, 20),
    ('rape', 'ua_forest_steppe', 4, 25, 5, 25),
    ('white_acacia', 'ua_forest_steppe', 5, 1, 5, 20),
    ('clover', 'ua_forest_steppe', 5, 20, 7, 31),
    ('phacelia', 'ua_forest_steppe', 5, 20, 8, 15),
    ('sweet_clover', 'ua_forest_steppe', 6, 1, 8, 10),
    ('linden', 'ua_forest_steppe', 6, 1, 7, 16),
    ('buckwheat', 'ua_forest_steppe', 6, 12, 7, 31),
    ('sunflower', 'ua_forest_steppe', 6, 21, 8, 15),
    ('heather', 'ua_forest_steppe', 8, 1, 9, 15),
    ('goldenrod', 'ua_forest_steppe', 8, 10, 9, 30)
) as v(code, region_code, start_month, start_day, end_month, end_day) on v.code = p.code
on conflict (honey_plant_id, region_code, start_month, start_day, end_month, end_day) do update set
    confidence = excluded.confidence,
    source = excluded.source,
    notes = excluded.notes;
