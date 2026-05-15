package main

import (
	"context"
	"errors"
	"log/slog"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/api"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/commands"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/config"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/database"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/events"
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
	commandPublisher := commands.NewPublisher(cfg, logger)
	if err := commandPublisher.Connect(); err != nil {
		logger.Warn("mqtt command publisher disabled", "error", err)
	} else {
		defer commandPublisher.Close()
	}
	server := api.NewServer(cfg, repo, bus, logger, commandPublisher)

	httpServer := &http.Server{
		Addr:              cfg.HTTPAddr,
		Handler:           server.Routes(),
		ReadHeaderTimeout: 5 * time.Second,
	}

	go func() {
		logger.Info("api service listening", "addr", cfg.HTTPAddr)
		if err := httpServer.ListenAndServe(); err != nil && !errors.Is(err, http.ErrServerClosed) {
			logger.Error("api server failed", "error", err)
			stop()
		}
	}()

	<-ctx.Done()
	shutdownCtx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	if err := httpServer.Shutdown(shutdownCtx); err != nil {
		logger.Error("api server shutdown", "error", err)
	}
}
