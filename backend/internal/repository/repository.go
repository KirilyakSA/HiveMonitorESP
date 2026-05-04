package repository

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"time"

	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/domain"
	"github.com/jackc/pgx/v5"
	"github.com/jackc/pgx/v5/pgxpool"
)

var ErrNotFound = errors.New("not found")

type Repository struct {
	db *pgxpool.Pool
}

func New(db *pgxpool.Pool) *Repository {
	return &Repository{db: db}
}

func (r *Repository) CreateUser(ctx context.Context, email, name, passwordHash string) (*domain.User, error) {
	row := r.db.QueryRow(ctx, `
		insert into users (email, name, password_hash)
		values ($1, $2, $3)
		returning id, email, name, password_hash, created_at
	`, email, name, passwordHash)
	return scanUser(row)
}

func (r *Repository) GetUserByEmail(ctx context.Context, email string) (*domain.User, error) {
	row := r.db.QueryRow(ctx, `
		select id, email, name, password_hash, created_at
		from users
		where email = $1
	`, email)
	return scanUser(row)
}

func (r *Repository) GetUserByID(ctx context.Context, id string) (*domain.User, error) {
	row := r.db.QueryRow(ctx, `
		select id, email, name, password_hash, created_at
		from users
		where id = $1
	`, id)
	return scanUser(row)
}

func scanUser(row pgx.Row) (*domain.User, error) {
	var user domain.User
	if err := row.Scan(&user.ID, &user.Email, &user.Name, &user.PasswordHash, &user.CreatedAt); err != nil {
		if errors.Is(err, pgx.ErrNoRows) {
			return nil, ErrNotFound
		}
		return nil, err
	}
	return &user, nil
}

func (r *Repository) CreateOrganization(ctx context.Context, userID, name string) (*domain.Organization, error) {
	tx, err := r.db.Begin(ctx)
	if err != nil {
		return nil, err
	}
	defer tx.Rollback(ctx)

	var org domain.Organization
	err = tx.QueryRow(ctx, `
		insert into organizations (name)
		values ($1)
		returning id, name, created_at
	`, name).Scan(&org.ID, &org.Name, &org.CreatedAt)
	if err != nil {
		return nil, err
	}

	_, err = tx.Exec(ctx, `
		insert into organization_members (organization_id, user_id, role, status)
		values ($1, $2, 'organization_owner', 'active')
	`, org.ID, userID)
	if err != nil {
		return nil, err
	}

	return &org, tx.Commit(ctx)
}

func (r *Repository) ListOrganizations(ctx context.Context, userID string) ([]domain.Organization, error) {
	rows, err := r.db.Query(ctx, `
		select o.id, o.name, o.created_at
		from organizations o
		join organization_members om on om.organization_id = o.id
		where om.user_id = $1 and om.status = 'active'
		order by o.created_at desc
	`, userID)
	if err != nil {
		return nil, err
	}
	defer rows.Close()

	var result []domain.Organization
	for rows.Next() {
		var org domain.Organization
		if err := rows.Scan(&org.ID, &org.Name, &org.CreatedAt); err != nil {
			return nil, err
		}
		result = append(result, org)
	}
	return result, rows.Err()
}

func (r *Repository) UserCanAccessOrganization(ctx context.Context, userID, organizationID string) (bool, error) {
	var ok bool
	err := r.db.QueryRow(ctx, `
		select exists (
			select 1 from organization_members
			where user_id = $1 and organization_id = $2 and status = 'active'
		)
	`, userID, organizationID).Scan(&ok)
	return ok, err
}

func (r *Repository) CreateApiary(ctx context.Context, userID string, input domain.Apiary) (*domain.Apiary, error) {
	ok, err := r.UserCanAccessOrganization(ctx, userID, input.OrganizationID)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, ErrNotFound
	}

	row := r.db.QueryRow(ctx, `
		insert into apiaries (
			organization_id, name, description, country, region, locality, address,
			location_description, latitude, longitude, timezone
		)
		values ($1,$2,$3,$4,$5,$6,$7,$8,$9,$10,coalesce(nullif($11,''), 'UTC'))
		returning id, organization_id, name, description, country, region, locality,
			address, location_description, latitude, longitude, timezone, created_at
	`, input.OrganizationID, input.Name, input.Description, input.Country, input.Region, input.Locality,
		input.Address, input.LocationDescription, input.Latitude, input.Longitude, input.Timezone)
	return scanApiary(row)
}

func (r *Repository) ListApiaries(ctx context.Context, userID, organizationID string) ([]domain.Apiary, error) {
	rows, err := r.db.Query(ctx, `
		select a.id, a.organization_id, a.name, a.description, a.country, a.region,
			a.locality, a.address, a.location_description, a.latitude, a.longitude,
			a.timezone, a.created_at
		from apiaries a
		join organization_members om on om.organization_id = a.organization_id
		where om.user_id = $1 and om.status = 'active'
			and (nullif($2, '') is null or a.organization_id = nullif($2, '')::uuid)
		order by a.created_at desc
	`, userID, organizationID)
	if err != nil {
		return nil, err
	}
	defer rows.Close()

	var result []domain.Apiary
	for rows.Next() {
		apiary, err := scanApiary(rows)
		if err != nil {
			return nil, err
		}
		result = append(result, *apiary)
	}
	return result, rows.Err()
}

func (r *Repository) UserCanAccessApiary(ctx context.Context, userID, apiaryID string) (bool, error) {
	var ok bool
	err := r.db.QueryRow(ctx, `
		select exists (
			select 1
			from apiaries a
			join organization_members om on om.organization_id = a.organization_id
			where a.id = $2 and om.user_id = $1 and om.status = 'active'
		)
	`, userID, apiaryID).Scan(&ok)
	return ok, err
}

func scanApiary(row pgx.Row) (*domain.Apiary, error) {
	var apiary domain.Apiary
	if err := row.Scan(
		&apiary.ID, &apiary.OrganizationID, &apiary.Name, &apiary.Description,
		&apiary.Country, &apiary.Region, &apiary.Locality, &apiary.Address,
		&apiary.LocationDescription, &apiary.Latitude, &apiary.Longitude,
		&apiary.Timezone, &apiary.CreatedAt,
	); err != nil {
		if errors.Is(err, pgx.ErrNoRows) {
			return nil, ErrNotFound
		}
		return nil, err
	}
	return &apiary, nil
}

func (r *Repository) CreateHive(ctx context.Context, userID string, input domain.Hive) (*domain.Hive, error) {
	ok, err := r.UserCanAccessApiary(ctx, userID, input.ApiaryID)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, ErrNotFound
	}

	row := r.db.QueryRow(ctx, `
		insert into hives (
			apiary_id, name, number, type, frame_count, super_count, bee_breed,
			settled_at, queen_year, queen_breed, queen_status, status, notes
		)
		values ($1,$2,$3,$4,$5,$6,$7,$8,$9,$10,$11,coalesce(nullif($12,''), 'active'),$13)
		returning id, apiary_id, name, number, type, frame_count, super_count, bee_breed,
			settled_at, queen_year, queen_breed, queen_status, status, notes, created_at
	`, input.ApiaryID, input.Name, input.Number, input.Type, input.FrameCount, input.SuperCount,
		input.BeeBreed, input.SettledAt, input.QueenYear, input.QueenBreed, input.QueenStatus,
		input.Status, input.Notes)
	return scanHive(row)
}

func (r *Repository) ListHives(ctx context.Context, userID, apiaryID string) ([]domain.Hive, error) {
	ok, err := r.UserCanAccessApiary(ctx, userID, apiaryID)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, ErrNotFound
	}

	rows, err := r.db.Query(ctx, `
		select id, apiary_id, name, number, type, frame_count, super_count, bee_breed,
			settled_at, queen_year, queen_breed, queen_status, status, notes, created_at
		from hives
		where apiary_id = $1
		order by created_at desc
	`, apiaryID)
	if err != nil {
		return nil, err
	}
	defer rows.Close()

	var result []domain.Hive
	for rows.Next() {
		hive, err := scanHive(rows)
		if err != nil {
			return nil, err
		}
		result = append(result, *hive)
	}
	return result, rows.Err()
}

func scanHive(row pgx.Row) (*domain.Hive, error) {
	var hive domain.Hive
	if err := row.Scan(
		&hive.ID, &hive.ApiaryID, &hive.Name, &hive.Number, &hive.Type,
		&hive.FrameCount, &hive.SuperCount, &hive.BeeBreed, &hive.SettledAt,
		&hive.QueenYear, &hive.QueenBreed, &hive.QueenStatus, &hive.Status,
		&hive.Notes, &hive.CreatedAt,
	); err != nil {
		if errors.Is(err, pgx.ErrNoRows) {
			return nil, ErrNotFound
		}
		return nil, err
	}
	return &hive, nil
}

func (r *Repository) ListUnassignedDevices(ctx context.Context, userID, apiaryID string) ([]domain.Device, error) {
	ok, err := r.UserCanAccessApiary(ctx, userID, apiaryID)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, ErrNotFound
	}
	return r.listDevices(ctx, `
		where d.apiary_id = $1 and not exists (
			select 1 from device_assignments da
			where da.device_id = d.id and da.unassigned_at is null
		)
	`, apiaryID)
}

func (r *Repository) listDevices(ctx context.Context, where string, args ...any) ([]domain.Device, error) {
	rows, err := r.db.Query(ctx, `
		select d.id, d.apiary_id, d.device_id, d.device_type, d.status, d.firmware_version,
			d.config_version, d.last_telemetry_at, d.last_status_at, d.expected_next_telemetry_at,
			d.missed_telemetry_count, d.telemetry_interval_minutes, d.created_at
		from devices d
		`+where+`
		order by d.created_at desc
	`, args...)
	if err != nil {
		return nil, err
	}
	defer rows.Close()

	var result []domain.Device
	for rows.Next() {
		device, err := scanDevice(rows)
		if err != nil {
			return nil, err
		}
		result = append(result, *device)
	}
	return result, rows.Err()
}

func scanDevice(row pgx.Row) (*domain.Device, error) {
	var device domain.Device
	if err := row.Scan(
		&device.ID, &device.ApiaryID, &device.DeviceID, &device.DeviceType, &device.Status,
		&device.FirmwareVersion, &device.ConfigVersion, &device.LastTelemetryAt,
		&device.LastStatusAt, &device.ExpectedNextTelemetryAt, &device.MissedTelemetryCount,
		&device.TelemetryIntervalMinutes, &device.CreatedAt,
	); err != nil {
		if errors.Is(err, pgx.ErrNoRows) {
			return nil, ErrNotFound
		}
		return nil, err
	}
	return &device, nil
}

func (r *Repository) AssignDeviceToHive(ctx context.Context, userID, apiaryID, deviceUUID, hiveID, importMode string) (*domain.DeviceAssignment, error) {
	ok, err := r.UserCanAccessApiary(ctx, userID, apiaryID)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, ErrNotFound
	}

	tx, err := r.db.Begin(ctx)
	if err != nil {
		return nil, err
	}
	defer tx.Rollback(ctx)

	var hiveExists bool
	if err := tx.QueryRow(ctx, `select exists(select 1 from hives where id = $1 and apiary_id = $2)`, hiveID, apiaryID).Scan(&hiveExists); err != nil {
		return nil, err
	}
	if !hiveExists {
		return nil, ErrNotFound
	}

	var deviceExists bool
	if err := tx.QueryRow(ctx, `select exists(select 1 from devices where id = $1 and apiary_id = $2)`, deviceUUID, apiaryID).Scan(&deviceExists); err != nil {
		return nil, err
	}
	if !deviceExists {
		return nil, ErrNotFound
	}

	_, err = tx.Exec(ctx, `
		update device_assignments
		set unassigned_at = now()
		where device_id = $1 and unassigned_at is null
	`, deviceUUID)
	if err != nil {
		return nil, err
	}

	var assignment domain.DeviceAssignment
	err = tx.QueryRow(ctx, `
		insert into device_assignments (device_id, apiary_id, hive_id, scope_type, scope_id, role)
		values ($1, $2, $3, 'hive', $3, 'primary_monitor')
		returning id, device_id, apiary_id, hive_id, scope_type, scope_id, role, assigned_at, unassigned_at
	`, deviceUUID, apiaryID, hiveID).Scan(
		&assignment.ID, &assignment.DeviceID, &assignment.ApiaryID, &assignment.HiveID,
		&assignment.ScopeType, &assignment.ScopeID, &assignment.Role,
		&assignment.AssignedAt, &assignment.UnassignedAt,
	)
	if err != nil {
		return nil, err
	}

	switch importMode {
	case "attach_to_hive":
		_, err = tx.Exec(ctx, `
			update sensor_readings
			set hive_id = $1
			where device_id = $2 and apiary_id = $3 and hive_id is null
		`, hiveID, deviceUUID, apiaryID)
	case "delete":
		_, err = tx.Exec(ctx, `
			delete from sensor_readings
			where device_id = $1 and apiary_id = $2 and hive_id is null
		`, deviceUUID, apiaryID)
	case "", "keep":
	default:
		err = fmt.Errorf("unsupported import mode %q", importMode)
	}
	if err != nil {
		return nil, err
	}

	_, err = tx.Exec(ctx, `update devices set status = 'assigned' where id = $1`, deviceUUID)
	if err != nil {
		return nil, err
	}
	return &assignment, tx.Commit(ctx)
}

type IngestReading struct {
	MetricType string
	Value      float64
	Unit       string
}

type IngestTelemetryInput struct {
	APIaryID                string
	DeviceID                string
	DeviceType              string
	FirmwareVersion         string
	ConfigVersion           *int
	MeasuredAt              time.Time
	TelemetryIntervalMinute int
	RawPayload              json.RawMessage
	Readings                []IngestReading
}

type IngestDeviceEventInput struct {
	APIaryID        string
	DeviceID        string
	DeviceType      string
	FirmwareVersion string
	ConfigVersion   *int
	EventType       string
	Message         string
	OK              *bool
	Command         string
	OccurredAt      time.Time
	RawPayload      json.RawMessage
	RawTopic        string
}

type IngestDeviceStatusInput struct {
	APIaryID        string
	DeviceID        string
	DeviceType      string
	FirmwareVersion string
	ConfigVersion   *int
	StatusAt        time.Time
	RawPayload      json.RawMessage
	RawTopic        string
}

func (r *Repository) IngestTelemetry(ctx context.Context, input IngestTelemetryInput) (*domain.Device, error) {
	tx, err := r.db.Begin(ctx)
	if err != nil {
		return nil, err
	}
	defer tx.Rollback(ctx)

	var apiaryID *string
	if input.APIaryID != "" {
		apiaryID = &input.APIaryID
	} else {
		resolvedAPIaryID, err := r.existingDeviceAPIaryID(ctx, tx, input.DeviceID)
		if err != nil {
			return nil, err
		}
		apiaryID = resolvedAPIaryID
	}
	interval := input.TelemetryIntervalMinute
	if interval <= 0 {
		interval = 30
	}
	expectedNext := input.MeasuredAt.Add(time.Duration(interval) * time.Minute)
	deviceType := input.DeviceType
	if deviceType == "" {
		deviceType = "hive_monitor"
	}

	var device domain.Device
	err = tx.QueryRow(ctx, `
		insert into devices (
			apiary_id, device_id, device_type, status, firmware_version, config_version,
			last_telemetry_at, expected_next_telemetry_at, missed_telemetry_count,
			telemetry_interval_minutes
		)
		values ($1, $2, $3, 'unassigned', $4, $5, $6, $7, 0, $8)
		on conflict (device_id) do update set
			apiary_id = coalesce(devices.apiary_id, excluded.apiary_id),
			firmware_version = excluded.firmware_version,
			config_version = excluded.config_version,
			last_telemetry_at = excluded.last_telemetry_at,
			expected_next_telemetry_at = excluded.expected_next_telemetry_at,
			missed_telemetry_count = 0,
			telemetry_interval_minutes = excluded.telemetry_interval_minutes,
			updated_at = now()
		returning id, apiary_id, device_id, device_type, status, firmware_version,
			config_version, last_telemetry_at, last_status_at, expected_next_telemetry_at,
			missed_telemetry_count, telemetry_interval_minutes, created_at
	`, apiaryID, input.DeviceID, deviceType, input.FirmwareVersion, input.ConfigVersion,
		input.MeasuredAt, expectedNext, interval).Scan(
		&device.ID, &device.ApiaryID, &device.DeviceID, &device.DeviceType, &device.Status,
		&device.FirmwareVersion, &device.ConfigVersion, &device.LastTelemetryAt,
		&device.LastStatusAt, &device.ExpectedNextTelemetryAt, &device.MissedTelemetryCount,
		&device.TelemetryIntervalMinutes, &device.CreatedAt,
	)
	if err != nil {
		return nil, err
	}

	var assignmentHiveID *string
	var assignmentAPIaryID *string
	err = tx.QueryRow(ctx, `
		select apiary_id, hive_id
		from device_assignments
		where device_id = $1 and unassigned_at is null
		order by assigned_at desc
		limit 1
	`, device.ID).Scan(&assignmentAPIaryID, &assignmentHiveID)
	if err != nil && !errors.Is(err, pgx.ErrNoRows) {
		return nil, err
	}
	if assignmentAPIaryID != nil {
		apiaryID = assignmentAPIaryID
	}

	var rawPayloadID string
	err = tx.QueryRow(ctx, `
		insert into raw_payloads (device_id, apiary_id, topic, payload, received_at)
		values ($1, $2, 'mqtt.telemetry', $3::jsonb, now())
		returning id
	`, device.ID, apiaryID, string(input.RawPayload)).Scan(&rawPayloadID)
	if err != nil {
		return nil, err
	}

	for _, reading := range input.Readings {
		if reading.MetricType == "" {
			continue
		}
		_, err = tx.Exec(ctx, `
			insert into sensor_readings (
				device_id, apiary_id, hive_id, metric_type, value, unit,
				measured_at, raw_payload_id
			)
			values ($1,$2,$3,$4,$5,$6,$7,$8)
		`, device.ID, apiaryID, assignmentHiveID, reading.MetricType, reading.Value,
			reading.Unit, input.MeasuredAt, rawPayloadID)
		if err != nil {
			return nil, err
		}
	}

	return &device, tx.Commit(ctx)
}

func (r *Repository) IngestDeviceEvent(ctx context.Context, input IngestDeviceEventInput) (*domain.DeviceEvent, error) {
	tx, err := r.db.Begin(ctx)
	if err != nil {
		return nil, err
	}
	defer tx.Rollback(ctx)

	device, apiaryID, hiveID, err := r.upsertMessageDevice(ctx, tx, messageDeviceInput{
		APIaryID:        input.APIaryID,
		DeviceID:        input.DeviceID,
		DeviceType:      input.DeviceType,
		FirmwareVersion: input.FirmwareVersion,
		ConfigVersion:   input.ConfigVersion,
	})
	if err != nil {
		return nil, err
	}

	var rawPayloadID string
	topic := input.RawTopic
	if topic == "" {
		topic = "mqtt.event"
	}
	err = tx.QueryRow(ctx, `
		insert into raw_payloads (device_id, apiary_id, topic, payload, received_at)
		values ($1, $2, $3, $4::jsonb, now())
		returning id
	`, device.ID, apiaryID, topic, string(input.RawPayload)).Scan(&rawPayloadID)
	if err != nil {
		return nil, err
	}

	occurredAt := input.OccurredAt
	if occurredAt.IsZero() {
		occurredAt = time.Now().UTC()
	}
	eventType := input.EventType
	if eventType == "" {
		eventType = "device_event"
	}

	var event domain.DeviceEvent
	err = tx.QueryRow(ctx, `
		insert into device_events (
			device_id, apiary_id, hive_id, event_type, message, ok, command,
			occurred_at, raw_payload_id
		)
		values ($1,$2,$3,$4,$5,$6,$7,$8,$9)
		returning id, device_id, apiary_id, hive_id, event_type, message, ok, command,
			occurred_at, received_at, raw_payload_id
	`, device.ID, apiaryID, hiveID, eventType, input.Message, input.OK, input.Command,
		occurredAt, rawPayloadID).Scan(
		&event.ID, &event.DeviceID, &event.ApiaryID, &event.HiveID, &event.EventType,
		&event.Message, &event.OK, &event.Command, &event.OccurredAt,
		&event.ReceivedAt, &event.RawPayloadID,
	)
	if err != nil {
		return nil, err
	}
	return &event, tx.Commit(ctx)
}

func (r *Repository) IngestDeviceStatus(ctx context.Context, input IngestDeviceStatusInput) (*domain.Device, error) {
	tx, err := r.db.Begin(ctx)
	if err != nil {
		return nil, err
	}
	defer tx.Rollback(ctx)

	device, apiaryID, _, err := r.upsertMessageDevice(ctx, tx, messageDeviceInput{
		APIaryID:        input.APIaryID,
		DeviceID:        input.DeviceID,
		DeviceType:      input.DeviceType,
		FirmwareVersion: input.FirmwareVersion,
		ConfigVersion:   input.ConfigVersion,
	})
	if err != nil {
		return nil, err
	}

	topic := input.RawTopic
	if topic == "" {
		topic = "mqtt.status"
	}
	_, err = tx.Exec(ctx, `
		insert into raw_payloads (device_id, apiary_id, topic, payload, received_at)
		values ($1, $2, $3, $4::jsonb, now())
	`, device.ID, apiaryID, topic, string(input.RawPayload))
	if err != nil {
		return nil, err
	}

	statusAt := input.StatusAt
	if statusAt.IsZero() {
		statusAt = time.Now().UTC()
	}
	err = tx.QueryRow(ctx, `
		update devices
		set last_status_at = $2,
			firmware_version = coalesce(nullif($3, ''), firmware_version),
			config_version = coalesce($4, config_version),
			updated_at = now()
		where id = $1
		returning id, apiary_id, device_id, device_type, status, firmware_version,
			config_version, last_telemetry_at, last_status_at, expected_next_telemetry_at,
			missed_telemetry_count, telemetry_interval_minutes, created_at
	`, device.ID, statusAt, input.FirmwareVersion, input.ConfigVersion).Scan(
		&device.ID, &device.ApiaryID, &device.DeviceID, &device.DeviceType, &device.Status,
		&device.FirmwareVersion, &device.ConfigVersion, &device.LastTelemetryAt,
		&device.LastStatusAt, &device.ExpectedNextTelemetryAt, &device.MissedTelemetryCount,
		&device.TelemetryIntervalMinutes, &device.CreatedAt,
	)
	if err != nil {
		return nil, err
	}
	return device, tx.Commit(ctx)
}

type messageDeviceInput struct {
	APIaryID        string
	DeviceID        string
	DeviceType      string
	FirmwareVersion string
	ConfigVersion   *int
}

func (r *Repository) upsertMessageDevice(ctx context.Context, tx pgx.Tx, input messageDeviceInput) (*domain.Device, *string, *string, error) {
	var apiaryID *string
	if input.APIaryID != "" {
		apiaryID = &input.APIaryID
	} else {
		resolvedAPIaryID, err := r.existingDeviceAPIaryID(ctx, tx, input.DeviceID)
		if err != nil {
			return nil, nil, nil, err
		}
		apiaryID = resolvedAPIaryID
	}
	deviceType := input.DeviceType
	if deviceType == "" {
		deviceType = "hive_monitor"
	}

	var device domain.Device
	err := tx.QueryRow(ctx, `
		insert into devices (
			apiary_id, device_id, device_type, status, firmware_version, config_version
		)
		values ($1, $2, $3, 'unassigned', $4, $5)
		on conflict (device_id) do update set
			apiary_id = coalesce(devices.apiary_id, excluded.apiary_id),
			firmware_version = coalesce(nullif(excluded.firmware_version, ''), devices.firmware_version),
			config_version = coalesce(excluded.config_version, devices.config_version),
			updated_at = now()
		returning id, apiary_id, device_id, device_type, status, firmware_version,
			config_version, last_telemetry_at, last_status_at, expected_next_telemetry_at,
			missed_telemetry_count, telemetry_interval_minutes, created_at
	`, apiaryID, input.DeviceID, deviceType, input.FirmwareVersion, input.ConfigVersion).Scan(
		&device.ID, &device.ApiaryID, &device.DeviceID, &device.DeviceType, &device.Status,
		&device.FirmwareVersion, &device.ConfigVersion, &device.LastTelemetryAt,
		&device.LastStatusAt, &device.ExpectedNextTelemetryAt, &device.MissedTelemetryCount,
		&device.TelemetryIntervalMinutes, &device.CreatedAt,
	)
	if err != nil {
		return nil, nil, nil, err
	}

	var assignmentHiveID *string
	var assignmentAPIaryID *string
	err = tx.QueryRow(ctx, `
		select apiary_id, hive_id
		from device_assignments
		where device_id = $1 and unassigned_at is null
		order by assigned_at desc
		limit 1
	`, device.ID).Scan(&assignmentAPIaryID, &assignmentHiveID)
	if err != nil && !errors.Is(err, pgx.ErrNoRows) {
		return nil, nil, nil, err
	}
	if assignmentAPIaryID != nil {
		apiaryID = assignmentAPIaryID
	}
	return &device, apiaryID, assignmentHiveID, nil
}

func (r *Repository) existingDeviceAPIaryID(ctx context.Context, tx pgx.Tx, deviceID string) (*string, error) {
	var apiaryID *string
	err := tx.QueryRow(ctx, `select apiary_id from devices where device_id = $1`, deviceID).Scan(&apiaryID)
	if errors.Is(err, pgx.ErrNoRows) {
		return nil, fmt.Errorf("apiary id is required for unknown legacy device %q", deviceID)
	}
	if err != nil {
		return nil, err
	}
	if apiaryID == nil {
		return nil, fmt.Errorf("apiary id is required for unassigned legacy device %q", deviceID)
	}
	return apiaryID, nil
}

func (r *Repository) LatestTelemetryForHive(ctx context.Context, userID, hiveID string) ([]domain.SensorReading, error) {
	var apiaryID string
	if err := r.db.QueryRow(ctx, `select apiary_id from hives where id = $1`, hiveID).Scan(&apiaryID); err != nil {
		if errors.Is(err, pgx.ErrNoRows) {
			return nil, ErrNotFound
		}
		return nil, err
	}
	ok, err := r.UserCanAccessApiary(ctx, userID, apiaryID)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, ErrNotFound
	}

	rows, err := r.db.Query(ctx, `
		select distinct on (metric_type)
			id, device_id, apiary_id, hive_id, metric_type, value, unit, measured_at, received_at
		from sensor_readings
		where hive_id = $1
		order by metric_type, measured_at desc
	`, hiveID)
	if err != nil {
		return nil, err
	}
	defer rows.Close()
	return scanReadings(rows)
}

func (r *Repository) TelemetryHistoryForHive(ctx context.Context, userID, hiveID string, from, to time.Time, metric string, limit int) ([]domain.SensorReading, error) {
	var apiaryID string
	if err := r.db.QueryRow(ctx, `select apiary_id from hives where id = $1`, hiveID).Scan(&apiaryID); err != nil {
		if errors.Is(err, pgx.ErrNoRows) {
			return nil, ErrNotFound
		}
		return nil, err
	}
	ok, err := r.UserCanAccessApiary(ctx, userID, apiaryID)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, ErrNotFound
	}

	if limit <= 0 || limit > 5000 {
		limit = 1000
	}
	rows, err := r.db.Query(ctx, `
		select id, device_id, apiary_id, hive_id, metric_type, value, unit, measured_at, received_at
		from sensor_readings
		where hive_id = $1
			and measured_at >= $2
			and measured_at <= $3
			and ($4 = '' or metric_type = $4)
		order by measured_at asc
		limit $5
	`, hiveID, from, to, metric, limit)
	if err != nil {
		return nil, err
	}
	defer rows.Close()
	return scanReadings(rows)
}

func (r *Repository) EventsForApiary(ctx context.Context, userID, apiaryID string, limit int) ([]domain.DeviceEvent, error) {
	ok, err := r.UserCanAccessApiary(ctx, userID, apiaryID)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, ErrNotFound
	}
	if limit <= 0 || limit > 1000 {
		limit = 200
	}
	rows, err := r.db.Query(ctx, `
		select id, device_id, apiary_id, hive_id, event_type, message, ok, command,
			occurred_at, received_at, raw_payload_id
		from device_events
		where apiary_id = $1
		order by occurred_at desc
		limit $2
	`, apiaryID, limit)
	if err != nil {
		return nil, err
	}
	defer rows.Close()
	return scanDeviceEvents(rows)
}

func (r *Repository) EventsForHive(ctx context.Context, userID, hiveID string, limit int) ([]domain.DeviceEvent, error) {
	var apiaryID string
	if err := r.db.QueryRow(ctx, `select apiary_id from hives where id = $1`, hiveID).Scan(&apiaryID); err != nil {
		if errors.Is(err, pgx.ErrNoRows) {
			return nil, ErrNotFound
		}
		return nil, err
	}
	ok, err := r.UserCanAccessApiary(ctx, userID, apiaryID)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, ErrNotFound
	}
	if limit <= 0 || limit > 1000 {
		limit = 200
	}
	rows, err := r.db.Query(ctx, `
		select id, device_id, apiary_id, hive_id, event_type, message, ok, command,
			occurred_at, received_at, raw_payload_id
		from device_events
		where hive_id = $1
		order by occurred_at desc
		limit $2
	`, hiveID, limit)
	if err != nil {
		return nil, err
	}
	defer rows.Close()
	return scanDeviceEvents(rows)
}

func scanReadings(rows pgx.Rows) ([]domain.SensorReading, error) {
	var result []domain.SensorReading
	for rows.Next() {
		var reading domain.SensorReading
		if err := rows.Scan(
			&reading.ID, &reading.DeviceID, &reading.ApiaryID, &reading.HiveID,
			&reading.MetricType, &reading.Value, &reading.Unit, &reading.MeasuredAt,
			&reading.ReceivedAt,
		); err != nil {
			return nil, err
		}
		result = append(result, reading)
	}
	return result, rows.Err()
}

func scanDeviceEvents(rows pgx.Rows) ([]domain.DeviceEvent, error) {
	var result []domain.DeviceEvent
	for rows.Next() {
		var event domain.DeviceEvent
		if err := rows.Scan(
			&event.ID, &event.DeviceID, &event.ApiaryID, &event.HiveID,
			&event.EventType, &event.Message, &event.OK, &event.Command,
			&event.OccurredAt, &event.ReceivedAt, &event.RawPayloadID,
		); err != nil {
			return nil, err
		}
		result = append(result, event)
	}
	return result, rows.Err()
}
