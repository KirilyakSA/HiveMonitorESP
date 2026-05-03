package ingestion

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"log/slog"
	"strconv"
	"strings"
	"time"

	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/config"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/events"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/repository"
	mqtt "github.com/eclipse/paho.mqtt.golang"
)

type Service struct {
	cfg    config.Config
	repo   *repository.Repository
	bus    *events.Bus
	logger *slog.Logger
}

func NewService(cfg config.Config, repo *repository.Repository, bus *events.Bus, logger *slog.Logger) *Service {
	return &Service{cfg: cfg, repo: repo, bus: bus, logger: logger}
}

func (s *Service) Run(ctx context.Context) error {
	opts := mqtt.NewClientOptions().
		AddBroker(s.cfg.MQTTBrokerURL).
		SetClientID(s.cfg.MQTTClientID).
		SetAutoReconnect(true).
		SetConnectRetry(true).
		SetDefaultPublishHandler(s.handleMessage)

	if s.cfg.MQTTUsername != "" {
		opts.SetUsername(s.cfg.MQTTUsername)
		opts.SetPassword(s.cfg.MQTTPassword)
	}
	opts.OnConnect = func(client mqtt.Client) {
		topics := strings.Split(s.cfg.MQTTTelemetryTopic, ",")
		s.logger.Info("mqtt connected", "topics", topics)
		for _, topic := range topics {
			topic = strings.TrimSpace(topic)
			if topic == "" {
				continue
			}
			token := client.Subscribe(topic, 1, s.handleMessage)
			token.Wait()
			if err := token.Error(); err != nil {
				s.logger.Error("mqtt subscribe failed", "topic", topic, "error", err)
			}
		}
	}
	opts.OnConnectionLost = func(_ mqtt.Client, err error) {
		s.logger.Warn("mqtt connection lost", "error", err)
	}

	client := mqtt.NewClient(opts)
	token := client.Connect()
	token.Wait()
	if err := token.Error(); err != nil {
		return err
	}
	defer client.Disconnect(250)

	<-ctx.Done()
	return nil
}

func (s *Service) handleMessage(_ mqtt.Client, message mqtt.Message) {
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()

	input, err := s.parseTelemetry(message.Topic(), message.Payload())
	if err != nil {
		s.logger.Warn("invalid telemetry", "topic", message.Topic(), "error", err)
		return
	}

	device, err := s.repo.IngestTelemetry(ctx, input)
	if err != nil {
		s.logger.Error("ingest telemetry", "device_id", input.DeviceID, "error", err)
		return
	}

	_ = s.bus.PublishJSON("telemetry.received", map[string]any{
		"device_uuid": device.ID,
		"device_id":   device.DeviceID,
		"apiary_id":   device.ApiaryID,
		"measured_at": input.MeasuredAt,
	})
	s.logger.Info("telemetry ingested", "device_id", input.DeviceID, "readings", len(input.Readings))
}

type telemetryPayload struct {
	SchemaVersion              int              `json:"schemaVersion"`
	DeviceID                   string           `json:"deviceId"`
	DeviceIDSnake              string           `json:"device_id"`
	Timestamp                  string           `json:"timestamp"`
	FirmwareVersion            string           `json:"firmwareVersion"`
	FirmwareVersionSnake       string           `json:"firmware_version"`
	ConfigVersion              *int             `json:"configVersion"`
	UptimeSeconds              *int64           `json:"uptimeSeconds"`
	MeasurementIntervalSeconds *int             `json:"measurementIntervalSeconds"`
	Readings                   []readingPayload `json:"readings"`
	Weight                     *float64         `json:"weight"`
	WeightChange               *float64         `json:"weightChange"`
	Temperature                *float64         `json:"temperature"`
	Humidity                   *float64         `json:"humidity"`
	HiveOpened                 *bool            `json:"hiveOpened"`
	BatteryPercent             *float64         `json:"batteryPercent"`
	BatteryVoltage             *float64         `json:"batteryVoltage"`
	RSSI                       *float64         `json:"rssi"`
	FreeHeap                   *float64         `json:"freeHeap"`
	Battery                    *struct {
		Percent *float64 `json:"percent"`
		Voltage *float64 `json:"voltage"`
	} `json:"battery"`
}

type readingPayload struct {
	Type  string  `json:"type"`
	Value float64 `json:"value"`
	Unit  string  `json:"unit"`
}

func (s *Service) parseTelemetry(topic string, payload []byte) (repository.IngestTelemetryInput, error) {
	var parsed telemetryPayload
	if err := json.Unmarshal(payload, &parsed); err != nil {
		return repository.IngestTelemetryInput{}, err
	}

	deviceID := firstNonEmpty(parsed.DeviceID, parsed.DeviceIDSnake, deviceIDFromTopic(topic))
	if deviceID == "" {
		return repository.IngestTelemetryInput{}, errors.New("device id is required")
	}

	measuredAt := time.Now().UTC()
	if parsed.Timestamp != "" {
		if ts, err := parseTimestamp(parsed.Timestamp); err == nil {
			measuredAt = ts
		}
	}

	intervalMinutes := s.cfg.DefaultIntervalMinute
	if parsed.MeasurementIntervalSeconds != nil && *parsed.MeasurementIntervalSeconds > 0 {
		intervalMinutes = max(1, *parsed.MeasurementIntervalSeconds/60)
	}

	readings := make([]repository.IngestReading, 0, len(parsed.Readings)+8)
	for _, reading := range parsed.Readings {
		readings = append(readings, repository.IngestReading{
			MetricType: normalizeMetric(reading.Type),
			Value:      reading.Value,
			Unit:       reading.Unit,
		})
	}
	addReading := func(metric string, value *float64, unit string) {
		if value == nil {
			return
		}
		readings = append(readings, repository.IngestReading{MetricType: metric, Value: *value, Unit: unit})
	}
	addReading("weight", parsed.Weight, "kg")
	addReading("weight_change", parsed.WeightChange, "kg")
	addReading("temperature", parsed.Temperature, "celsius")
	addReading("humidity", parsed.Humidity, "percent")
	addReading("battery_percent", parsed.BatteryPercent, "percent")
	addReading("battery_voltage", parsed.BatteryVoltage, "volt")
	addReading("rssi", parsed.RSSI, "dbm")
	addReading("free_heap", parsed.FreeHeap, "bytes")
	if parsed.Battery != nil {
		addReading("battery_percent", parsed.Battery.Percent, "percent")
		addReading("battery_voltage", parsed.Battery.Voltage, "volt")
	}
	if parsed.HiveOpened != nil {
		value := 0.0
		if *parsed.HiveOpened {
			value = 1
		}
		readings = append(readings, repository.IngestReading{MetricType: "hive_opened", Value: value, Unit: "bool"})
	}

	if len(readings) == 0 {
		return repository.IngestTelemetryInput{}, errors.New("no readings")
	}

	apiaryID := firstNonEmpty(apiaryIDFromTopic(topic), s.cfg.DefaultAPIaryID)

	return repository.IngestTelemetryInput{
		APIaryID:                apiaryID,
		DeviceID:                deviceID,
		DeviceType:              "hive_monitor",
		FirmwareVersion:         firstNonEmpty(parsed.FirmwareVersion, parsed.FirmwareVersionSnake),
		ConfigVersion:           parsed.ConfigVersion,
		MeasuredAt:              measuredAt,
		TelemetryIntervalMinute: intervalMinutes,
		RawPayload:              json.RawMessage(payload),
		Readings:                readings,
	}, nil
}

func deviceIDFromTopic(topic string) string {
	parts := strings.Split(topic, "/")
	if len(parts) >= 3 && parts[0] == "hives" && parts[2] == "telemetry" {
		return parts[1]
	}
	if len(parts) >= 5 && parts[0] == "apiaries" && parts[2] == "devices" && parts[4] == "telemetry" {
		return parts[3]
	}
	return ""
}

func apiaryIDFromTopic(topic string) string {
	parts := strings.Split(topic, "/")
	if len(parts) >= 5 && parts[0] == "apiaries" && parts[2] == "devices" && parts[4] == "telemetry" {
		return parts[1]
	}
	return ""
}

func parseTimestamp(value string) (time.Time, error) {
	if ts, err := time.Parse(time.RFC3339, value); err == nil {
		return ts.UTC(), nil
	}
	if unix, err := strconv.ParseInt(value, 10, 64); err == nil {
		return time.Unix(unix, 0).UTC(), nil
	}
	return time.Time{}, fmt.Errorf("unsupported timestamp %q", value)
}

func normalizeMetric(value string) string {
	return strings.TrimSpace(strings.ToLower(value))
}

func firstNonEmpty(values ...string) string {
	for _, value := range values {
		if strings.TrimSpace(value) != "" {
			return strings.TrimSpace(value)
		}
	}
	return ""
}
