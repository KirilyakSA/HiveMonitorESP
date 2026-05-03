package main

import (
	"context"
	"log/slog"
	"os"
	"os/signal"
	"syscall"

	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/config"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/database"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/events"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/ingestion"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/repository"
)

func main() {
	logger := slog.New(slog.NewJSONHandler(os.Stdout, nil))
	cfg := config.Load()

	ctx, stop := signal.NotifyContext(context.Background(), syscall.SIGINT, syscall.SIGTERM)
	defer stop()

	db, err := database.Open(ctx, cfg.DatabaseURL)
	if err != nil {
		logger.Error("open database", "error", err)
		os.Exit(1)
	}
	defer db.Close()

	bus, err := events.Connect(cfg.NATSURL)
	if err != nil {
		logger.Warn("nats disabled", "error", err)
	}
	if bus != nil {
		defer bus.Close()
	}

	repo := repository.New(db)
	service := ingestion.NewService(cfg, repo, bus, logger)
	if err := service.Run(ctx); err != nil {
		logger.Error("mqtt ingestion stopped", "error", err)
		os.Exit(1)
	}
}
