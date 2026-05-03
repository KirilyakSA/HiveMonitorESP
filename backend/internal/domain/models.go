package domain

import "time"

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
	ID          string     `json:"id"`
	ApiaryID    string     `json:"apiary_id"`
	Name        string     `json:"name"`
	Number      string     `json:"number"`
	Type        string     `json:"type"`
	FrameCount  *int       `json:"frame_count,omitempty"`
	SuperCount  *int       `json:"super_count,omitempty"`
	BeeBreed    string     `json:"bee_breed"`
	SettledAt   *time.Time `json:"settled_at,omitempty"`
	QueenYear   *int       `json:"queen_year,omitempty"`
	QueenBreed  string     `json:"queen_breed"`
	QueenStatus string     `json:"queen_status"`
	Status      string     `json:"status"`
	Notes       string     `json:"notes"`
	CreatedAt   time.Time  `json:"created_at"`
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
	Unit       string    `json:"unit"`
	MeasuredAt time.Time `json:"measured_at"`
	ReceivedAt time.Time `json:"received_at"`
}
