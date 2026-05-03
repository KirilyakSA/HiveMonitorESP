package events

import (
	"encoding/json"

	"github.com/nats-io/nats.go"
)

type Bus struct {
	conn *nats.Conn
}

func Connect(url string) (*Bus, error) {
	conn, err := nats.Connect(url)
	if err != nil {
		return nil, err
	}
	return &Bus{conn: conn}, nil
}

func (b *Bus) Close() {
	if b != nil && b.conn != nil {
		b.conn.Close()
	}
}

func (b *Bus) PublishJSON(subject string, value any) error {
	if b == nil || b.conn == nil {
		return nil
	}
	data, err := json.Marshal(value)
	if err != nil {
		return err
	}
	return b.conn.Publish(subject, data)
}
