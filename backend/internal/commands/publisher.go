package commands

import (
	"context"
	"encoding/json"
	"fmt"
	"log/slog"
	"time"

	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/config"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/domain"
	mqtt "github.com/eclipse/paho.mqtt.golang"
)

type Publisher struct {
	client mqtt.Client
	logger *slog.Logger
}

func NewPublisher(cfg config.Config, logger *slog.Logger) *Publisher {
	opts := mqtt.NewClientOptions().
		AddBroker(cfg.MQTTBrokerURL).
		SetClientID(cfg.MQTTCommandClientID).
		SetAutoReconnect(true).
		SetConnectRetry(true)

	if cfg.MQTTUsername != "" {
		opts.SetUsername(cfg.MQTTUsername)
		opts.SetPassword(cfg.MQTTPassword)
	}

	return &Publisher{
		client: mqtt.NewClient(opts),
		logger: logger,
	}
}

func (p *Publisher) Connect() error {
	if p == nil || p.client == nil {
		return nil
	}
	token := p.client.Connect()
	token.Wait()
	return token.Error()
}

func (p *Publisher) Close() {
	if p != nil && p.client != nil && p.client.IsConnected() {
		p.client.Disconnect(250)
	}
}

type CommandMessage struct {
	ID        string          `json:"id"`
	Command   string          `json:"command"`
	Payload   json.RawMessage `json:"payload"`
	CreatedAt time.Time       `json:"created_at"`
	ExpiresAt time.Time       `json:"expires_at"`
}

func (p *Publisher) PublishDeviceCommand(ctx context.Context, command domain.DeviceCommand) ([]string, error) {
	if p == nil || p.client == nil {
		return nil, fmt.Errorf("mqtt command publisher is not configured")
	}
	if !p.client.IsConnected() {
		return nil, fmt.Errorf("mqtt command publisher is not connected")
	}

	message := CommandMessage{
		ID:        command.ID,
		Command:   command.Command,
		Payload:   command.Payload,
		CreatedAt: command.CreatedAt,
		ExpiresAt: command.ExpiresAt,
	}
	body, err := json.Marshal(message)
	if err != nil {
		return nil, err
	}

	topics := []string{
		fmt.Sprintf("apiaries/%s/devices/%s/commands", command.APIaryID, command.DevicePublicID),
		fmt.Sprintf("hives/%s/commands", command.DevicePublicID),
	}
	published := make([]string, 0, len(topics))
	for _, topic := range topics {
		if err := p.publish(ctx, topic, body); err != nil {
			return published, err
		}
		published = append(published, topic)
	}
	return published, nil
}

func (p *Publisher) publish(ctx context.Context, topic string, body []byte) error {
	token := p.client.Publish(topic, 1, false, body)
	done := make(chan struct{})
	go func() {
		token.Wait()
		close(done)
	}()

	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-done:
		if err := token.Error(); err != nil {
			if p.logger != nil {
				p.logger.Warn("mqtt command publish failed", "topic", topic, "error", err)
			}
			return err
		}
		return nil
	}
}
