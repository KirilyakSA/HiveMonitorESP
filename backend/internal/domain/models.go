package domain

import (
	"encoding/json"
	"time"
)

type User struct {
	ID           string
	Email        string
	Name         string
	PasswordHash string
	CreatedAt    time.Time
}

type Organization struct {
	ID        string    `json:"id"`
	Name      string    `json:"name"`
	CreatedAt time.Time `json:"created_at"`
}

type Apiary struct {
	ID                  string    `json:"id"`
	OrganizationID      string    `json:"organization_id"`
	Name                string    `json:"name"`
	Description         string    `json:"description"`
	Country             string    `json:"country"`
	Region              string    `json:"region"`
	Locality            string    `json:"locality"`
	Address             string    `json:"address"`
	LocationDescription string    `json:"location_description"`
	Latitude            *float64  `json:"latitude,omitempty"`
	Longitude           *float64  `json:"longitude,omitempty"`
	Timezone            string    `json:"timezone"`
	CreatedAt           time.Time `json:"created_at"`
}

type Hive struct {
	ID                     string     `json:"id"`
	ApiaryID               string     `json:"apiary_id"`
	Name                   string     `json:"name"`
	Number                 string     `json:"number"`
	Type                   string     `json:"type"`
	FrameCount             *int       `json:"frame_count,omitempty"`
	SuperCount             *int       `json:"super_count,omitempty"`
	BeeBreed               string     `json:"bee_breed"`
	SettledAt              *time.Time `json:"settled_at,omitempty"`
	QueenYear              *int       `json:"queen_year,omitempty"`
	QueenBreed             string     `json:"queen_breed"`
	QueenStatus            string     `json:"queen_status"`
	Status                 string     `json:"status"`
	Notes                  string     `json:"notes"`
	CreatedAt              time.Time  `json:"created_at"`
	AssignedDeviceID       *string    `json:"assigned_device_id,omitempty"`
	AssignedDevicePublicID *string    `json:"assigned_device_public_id,omitempty"`
	AssignedDeviceType     *string    `json:"assigned_device_type,omitempty"`
	AssignedDeviceCount    int        `json:"assigned_device_count"`
}

type Device struct {
	ID                       string     `json:"id"`
	ApiaryID                 *string    `json:"apiary_id,omitempty"`
	DeviceID                 string     `json:"device_id"`
	DeviceType               string     `json:"device_type"`
	Status                   string     `json:"status"`
	FirmwareVersion          string     `json:"firmware_version"`
	ConfigVersion            *int       `json:"config_version,omitempty"`
	LastTelemetryAt          *time.Time `json:"last_telemetry_at,omitempty"`
	LastStatusAt             *time.Time `json:"last_status_at,omitempty"`
	ExpectedNextTelemetryAt  *time.Time `json:"expected_next_telemetry_at,omitempty"`
	MissedTelemetryCount     int        `json:"missed_telemetry_count"`
	TelemetryIntervalMinutes int        `json:"telemetry_interval_minutes"`
	CreatedAt                time.Time  `json:"created_at"`
}

type DeviceAssignment struct {
	ID           string     `json:"id"`
	DeviceID     string     `json:"device_id"`
	ApiaryID     string     `json:"apiary_id"`
	HiveID       *string    `json:"hive_id,omitempty"`
	ScopeType    string     `json:"scope_type"`
	ScopeID      string     `json:"scope_id"`
	Role         string     `json:"role"`
	AssignedAt   time.Time  `json:"assigned_at"`
	UnassignedAt *time.Time `json:"unassigned_at,omitempty"`
}

type SensorReading struct {
	ID         string    `json:"id"`
	DeviceID   string    `json:"device_id"`
	ApiaryID   *string   `json:"apiary_id,omitempty"`
	HiveID     *string   `json:"hive_id,omitempty"`
	MetricType string    `json:"metric_type"`
	Value      float64   `json:"value"`
	RawValue   *float64  `json:"raw_value,omitempty"`
	Unit       string    `json:"unit"`
	MeasuredAt time.Time `json:"measured_at"`
	ReceivedAt time.Time `json:"received_at"`
}

type HiveScaleProfile struct {
	HiveID          string          `json:"hive_id"`
	APIaryID        string          `json:"apiary_id"`
	EmptyHiveTareKG *float64        `json:"empty_hive_tare_kg,omitempty"`
	ActiveTareKG    float64         `json:"active_tare_kg"`
	SuperTares      json.RawMessage `json:"super_tares"`
	UpdatedBy       *string         `json:"updated_by,omitempty"`
	CreatedAt       time.Time       `json:"created_at"`
	UpdatedAt       time.Time       `json:"updated_at"`
}

type HiveTareEvent struct {
	ID                   string          `json:"id"`
	HiveID               string          `json:"hive_id"`
	APIaryID             string          `json:"apiary_id"`
	DeviceID             *string         `json:"device_id,omitempty"`
	CommandID            *string         `json:"command_id,omitempty"`
	TareKind             string          `json:"tare_kind"`
	SuperIndex           *int            `json:"super_index,omitempty"`
	MeasuredRawWeightKG  float64         `json:"measured_raw_weight_kg"`
	PreviousActiveTareKG float64         `json:"previous_active_tare_kg"`
	NewActiveTareKG      float64         `json:"new_active_tare_kg"`
	Comment              string          `json:"comment"`
	Metadata             json.RawMessage `json:"metadata"`
	CreatedBy            *string         `json:"created_by,omitempty"`
	CreatedAt            time.Time       `json:"created_at"`
}

type SaveHiveTareInput struct {
	TareKind            string          `json:"tare_kind"`
	SuperIndex          *int            `json:"super_index,omitempty"`
	MeasuredRawWeightKG float64         `json:"measured_raw_weight_kg"`
	DeviceID            *string         `json:"device_id,omitempty"`
	CommandID           *string         `json:"command_id,omitempty"`
	Comment             string          `json:"comment"`
	Metadata            json.RawMessage `json:"metadata"`
}

type RemoveHiveSuperInput struct {
	SuperIndex *int            `json:"super_index,omitempty"`
	Comment    string          `json:"comment"`
	Metadata   json.RawMessage `json:"metadata"`
}

type DeviceEvent struct {
	ID           string    `json:"id"`
	DeviceID     *string   `json:"device_id,omitempty"`
	ApiaryID     *string   `json:"apiary_id,omitempty"`
	HiveID       *string   `json:"hive_id,omitempty"`
	EventType    string    `json:"event_type"`
	Message      string    `json:"message"`
	OK           *bool     `json:"ok,omitempty"`
	Command      string    `json:"command"`
	OccurredAt   time.Time `json:"occurred_at"`
	ReceivedAt   time.Time `json:"received_at"`
	RawPayloadID *string   `json:"raw_payload_id,omitempty"`
}

type DeviceCommand struct {
	ID              string          `json:"id"`
	APIaryID        string          `json:"apiary_id"`
	DeviceID        string          `json:"device_id"`
	DevicePublicID  string          `json:"device_public_id"`
	RequestedBy     *string         `json:"requested_by,omitempty"`
	Command         string          `json:"command"`
	Payload         json.RawMessage `json:"payload"`
	Status          string          `json:"status"`
	MQTTTopic       string          `json:"mqtt_topic"`
	PublishedTopics []string        `json:"published_topics"`
	ErrorMessage    string          `json:"error_message"`
	Result          json.RawMessage `json:"result,omitempty"`
	CreatedAt       time.Time       `json:"created_at"`
	PublishedAt     *time.Time      `json:"published_at,omitempty"`
	AcknowledgedAt  *time.Time      `json:"acknowledged_at,omitempty"`
	ExpiresAt       time.Time       `json:"expires_at"`
	UpdatedAt       time.Time       `json:"updated_at"`
}

type CreateDeviceCommandInput struct {
	Command          string          `json:"command"`
	Payload          json.RawMessage `json:"payload"`
	ExpiresInSeconds int             `json:"expires_in_seconds"`
}

type FirmwareRelease struct {
	ID             string    `json:"id"`
	DeviceType     string    `json:"device_type"`
	Version        string    `json:"version"`
	Channel        string    `json:"channel"`
	ArtifactURL    string    `json:"artifact_url"`
	ChecksumSHA256 string    `json:"checksum_sha256"`
	SizeBytes      *int64    `json:"size_bytes,omitempty"`
	ReleaseNotes   string    `json:"release_notes"`
	IsActive       bool      `json:"is_active"`
	CreatedBy      *string   `json:"created_by,omitempty"`
	CreatedAt      time.Time `json:"created_at"`
	UpdatedAt      time.Time `json:"updated_at"`
}

type CreateFirmwareReleaseInput struct {
	DeviceType     string `json:"device_type"`
	Version        string `json:"version"`
	Channel        string `json:"channel"`
	ArtifactURL    string `json:"artifact_url"`
	ChecksumSHA256 string `json:"checksum_sha256"`
	SizeBytes      *int64 `json:"size_bytes,omitempty"`
	ReleaseNotes   string `json:"release_notes"`
	IsActive       *bool  `json:"is_active,omitempty"`
}

type CalendarSettings struct {
	APIaryID                 string    `json:"apiary_id"`
	TemplateID               string    `json:"template_id"`
	RegionCode               string    `json:"region_code"`
	ClimateZone              string    `json:"climate_zone"`
	DateShiftDays            int       `json:"date_shift_days"`
	EnableWeatherTips        bool      `json:"enable_weather_tips"`
	EnableBloomTips          bool      `json:"enable_bloom_tips"`
	EnableTelemetryTips      bool      `json:"enable_telemetry_tips"`
	EnableTaskAutogeneration bool      `json:"enable_task_autogeneration"`
	CreatedAt                time.Time `json:"created_at"`
	UpdatedAt                time.Time `json:"updated_at"`
}

type AdviceTemplate struct {
	ID                string          `json:"id"`
	TemplateID        string          `json:"template_id"`
	PeriodCode        string          `json:"period_code"`
	Code              string          `json:"code"`
	Title             string          `json:"title"`
	Body              string          `json:"body"`
	Category          string          `json:"category"`
	Severity          string          `json:"severity"`
	Priority          int             `json:"priority"`
	StartMonth        *int            `json:"start_month,omitempty"`
	StartDay          *int            `json:"start_day,omitempty"`
	EndMonth          *int            `json:"end_month,omitempty"`
	EndDay            *int            `json:"end_day,omitempty"`
	TriggerType       string          `json:"trigger_type"`
	TriggerConfig     json.RawMessage `json:"trigger_config"`
	ActionLabel       string          `json:"action_label"`
	ActionType        string          `json:"action_type"`
	IsUserDismissible bool            `json:"is_user_dismissible"`
}

type BeekeepingAdvice struct {
	ID                string  `json:"id"`
	Code              string  `json:"code"`
	Title             string  `json:"title"`
	Body              string  `json:"body"`
	Category          string  `json:"category"`
	Severity          string  `json:"severity"`
	Priority          int     `json:"priority"`
	Source            string  `json:"source"`
	RelatedHiveID     *string `json:"related_hive_id,omitempty"`
	ActionLabel       string  `json:"action_label"`
	ActionType        string  `json:"action_type"`
	IsUserDismissible bool    `json:"is_user_dismissible"`
	State             string  `json:"state,omitempty"`
}

type TaskTemplate struct {
	ID                     string          `json:"id"`
	TemplateID             string          `json:"template_id"`
	PeriodCode             string          `json:"period_code"`
	Code                   string          `json:"code"`
	Title                  string          `json:"title"`
	Description            string          `json:"description"`
	Category               string          `json:"category"`
	DefaultDurationMinutes *int            `json:"default_duration_minutes,omitempty"`
	Severity               string          `json:"severity"`
	Priority               int             `json:"priority"`
	StartMonth             *int            `json:"start_month,omitempty"`
	StartDay               *int            `json:"start_day,omitempty"`
	EndMonth               *int            `json:"end_month,omitempty"`
	EndDay                 *int            `json:"end_day,omitempty"`
	RecurrenceRule         string          `json:"recurrence_rule"`
	WeatherConstraints     json.RawMessage `json:"weather_constraints"`
	TelemetryConstraints   json.RawMessage `json:"telemetry_constraints"`
}

type ApiaryTask struct {
	ID               string          `json:"id"`
	APIaryID         string          `json:"apiary_id"`
	HiveID           *string         `json:"hive_id,omitempty"`
	SourceTemplateID *string         `json:"source_template_id,omitempty"`
	Title            string          `json:"title"`
	Description      string          `json:"description"`
	Category         string          `json:"category"`
	Severity         string          `json:"severity"`
	Status           string          `json:"status"`
	DueAt            *time.Time      `json:"due_at,omitempty"`
	CompletedAt      *time.Time      `json:"completed_at,omitempty"`
	DismissedAt      *time.Time      `json:"dismissed_at,omitempty"`
	SnoozedUntil     *time.Time      `json:"snoozed_until,omitempty"`
	Metadata         json.RawMessage `json:"metadata"`
	CreatedAt        time.Time       `json:"created_at"`
	UpdatedAt        time.Time       `json:"updated_at"`
}

type CreateApiaryTaskInput struct {
	Title       string     `json:"title"`
	Description string     `json:"description"`
	Category    string     `json:"category"`
	Severity    string     `json:"severity"`
	DueAt       *time.Time `json:"due_at,omitempty"`
	HiveID      *string    `json:"hive_id,omitempty"`
}

type UpdateApiaryTaskInput struct {
	Status       string     `json:"status"`
	SnoozedUntil *time.Time `json:"snoozed_until,omitempty"`
}

type AdviceStateInput struct {
	Status       string     `json:"status"`
	SnoozedUntil *time.Time `json:"snoozed_until,omitempty"`
}
