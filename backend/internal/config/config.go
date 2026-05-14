package config

import (
	"os"
	"strconv"
	"time"

	"github.com/joho/godotenv"
)

type Config struct {
	HTTPAddr              string
	DatabaseURL           string
	JWTSecret             string
	AccessTokenTTL        time.Duration
	NATSURL               string
	MQTTBrokerURL         string
	MQTTUsername          string
	MQTTPassword          string
	MQTTClientID          string
	MQTTTelemetryTopic    string
	DefaultAPIaryID       string
	DefaultIntervalMinute int
	WorkerTickInterval    time.Duration
}

func Load() Config {
	_ = godotenv.Load()
	return Config{
		HTTPAddr:              env("HTTP_ADDR", ":8080"),
		DatabaseURL:           env("DATABASE_URL", "postgres://hivemonitor:hivemonitor@localhost:5432/hivemonitor?sslmode=disable"),
		JWTSecret:             env("JWT_SECRET", "dev-secret-change-me"),
		AccessTokenTTL:        time.Duration(envInt("ACCESS_TOKEN_TTL_MINUTES", 1440)) * time.Minute,
		NATSURL:               env("NATS_URL", "nats://localhost:4222"),
		MQTTBrokerURL:         env("MQTT_BROKER_URL", "tcp://localhost:1883"),
		MQTTUsername:          env("MQTT_USERNAME", ""),
		MQTTPassword:          env("MQTT_PASSWORD", ""),
		MQTTClientID:          env("MQTT_CLIENT_ID", "hivemonitor-mqtt-ingestion"),
		MQTTTelemetryTopic:    env("MQTT_TELEMETRY_TOPIC", "hives/+/telemetry,apiaries/+/devices/+/telemetry,hives/+/events,apiaries/+/devices/+/events,hives/+/status,apiaries/+/devices/+/status"),
		DefaultAPIaryID:       env("DEFAULT_APIARY_ID", ""),
		DefaultIntervalMinute: envInt("DEFAULT_TELEMETRY_INTERVAL_MINUTES", 30),
		WorkerTickInterval:    time.Duration(envInt("WORKER_TICK_SECONDS", 60)) * time.Second,
	}
}

func env(key, fallback string) string {
	if value := os.Getenv(key); value != "" {
		return value
	}
	return fallback
}

func envInt(key string, fallback int) int {
	value := os.Getenv(key)
	if value == "" {
		return fallback
	}
	parsed, err := strconv.Atoi(value)
	if err != nil {
		return fallback
	}
	return parsed
}
