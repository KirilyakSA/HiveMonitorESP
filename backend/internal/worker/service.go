package worker

import (
	"context"
	"log/slog"
	"time"

	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/config"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/events"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/repository"
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
	interval := s.cfg.WorkerTickInterval
	if interval <= 0 {
		interval = time.Minute
	}

	s.logger.Info("worker service started", "interval", interval.String())
	if err := s.tick(ctx); err != nil {
		s.logger.Error("worker tick failed", "error", err)
	}

	ticker := time.NewTicker(interval)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return nil
		case <-ticker.C:
			if err := s.tick(ctx); err != nil {
				s.logger.Error("worker tick failed", "error", err)
			}
		}
	}
}

func (s *Service) tick(ctx context.Context) error {
	now := time.Now().UTC()

	taskRows, err := s.repo.UpdateAllApiaryTaskDueStatuses(ctx, now)
	if err != nil {
		return err
	}

	missedRows, err := s.repo.UpdateMissedTelemetryCounts(ctx, now)
	if err != nil {
		return err
	}

	expiredCommandRows, err := s.repo.ExpireDeviceCommands(ctx, now)
	if err != nil {
		return err
	}

	s.logger.Info(
		"worker tick completed",
		"task_rows", taskRows,
		"missed_telemetry_rows", missedRows,
		"expired_command_rows", expiredCommandRows,
	)
	return s.bus.PublishJSON("worker.tick.completed", map[string]any{
		"task_rows":             taskRows,
		"missed_telemetry_rows": missedRows,
		"expired_command_rows":  expiredCommandRows,
		"checked_at":            now,
	})
}
