package beekeeping

import (
	"context"
	"fmt"
	"sort"
	"strings"
	"time"

	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/domain"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/repository"
)

type Service struct {
	repo *repository.Repository
}

func New(repo *repository.Repository) *Service {
	return &Service{repo: repo}
}

func (s *Service) ActiveAdvice(ctx context.Context, userID, apiaryID string, date time.Time, includeHidden bool) ([]domain.BeekeepingAdvice, error) {
	settings, err := s.repo.EnsureCalendarSettingsForApiary(ctx, userID, apiaryID)
	if err != nil {
		return nil, err
	}

	templates, err := s.repo.AdviceTemplates(ctx, settings.TemplateID)
	if err != nil {
		return nil, err
	}
	states, err := s.repo.AdviceStates(ctx, apiaryID)
	if err != nil {
		return nil, err
	}

	shifted := date.AddDate(0, 0, settings.DateShiftDays)
	now := time.Now().UTC()
	advice := make([]domain.BeekeepingAdvice, 0, len(templates))
	for _, template := range templates {
		if !adviceTemplateActive(template, shifted) {
			continue
		}
		state := states[template.Code]
		if !includeHidden && stateFiltered(state, now) {
			continue
		}
		source := "calendar_template"
		if template.TriggerType != "" && template.TriggerType != "calendar" {
			source = template.TriggerType
		}
		advice = append(advice, domain.BeekeepingAdvice{
			ID:                template.ID,
			Code:              template.Code,
			Title:             template.Title,
			Body:              template.Body,
			Category:          template.Category,
			Severity:          template.Severity,
			Priority:          template.Priority,
			Source:            source,
			ActionLabel:       template.ActionLabel,
			ActionType:        template.ActionType,
			IsUserDismissible: template.IsUserDismissible,
			State:             state.Status,
		})
	}

	if settings.EnableTelemetryTips {
		telemetryAdvice, err := s.telemetryAdvice(ctx, userID, apiaryID, states, now, includeHidden)
		if err != nil {
			return nil, err
		}
		advice = append(advice, telemetryAdvice...)
	}

	sort.SliceStable(advice, func(i, j int) bool {
		left, right := severityRank(advice[i].Severity), severityRank(advice[j].Severity)
		if left != right {
			return left < right
		}
		if advice[i].Priority != advice[j].Priority {
			return advice[i].Priority < advice[j].Priority
		}
		return advice[i].Title < advice[j].Title
	})
	if len(advice) > 10 {
		advice = advice[:10]
	}
	return advice, nil
}

func (s *Service) CalendarTasks(ctx context.Context, userID, apiaryID string, from, to time.Time) ([]domain.ApiaryTask, error) {
	settings, err := s.repo.EnsureCalendarSettingsForApiary(ctx, userID, apiaryID)
	if err != nil {
		return nil, err
	}
	if settings.EnableTaskAutogeneration {
		if err := s.generateTasks(ctx, apiaryID, settings, from, to); err != nil {
			return nil, err
		}
	}
	if err := s.repo.UpdateApiaryTaskDueStatuses(ctx, apiaryID, time.Now().UTC()); err != nil {
		return nil, err
	}
	return s.repo.ListApiaryTasks(ctx, userID, apiaryID, from, to)
}

func (s *Service) CreateTask(ctx context.Context, userID, apiaryID string, input domain.CreateApiaryTaskInput) (*domain.ApiaryTask, error) {
	input.Title = strings.TrimSpace(input.Title)
	if input.Title == "" {
		return nil, fmt.Errorf("title is required")
	}
	return s.repo.CreateApiaryTask(ctx, userID, apiaryID, input)
}

func (s *Service) UpdateTask(ctx context.Context, userID, apiaryID, taskID string, input domain.UpdateApiaryTaskInput) (*domain.ApiaryTask, error) {
	if !validTaskStatus(input.Status) {
		return nil, fmt.Errorf("unsupported task status")
	}
	return s.repo.UpdateApiaryTaskStatus(ctx, userID, apiaryID, taskID, input)
}

func (s *Service) SetAdviceState(ctx context.Context, userID, apiaryID, adviceCode string, input domain.AdviceStateInput) error {
	if input.Status != "dismissed" && input.Status != "snoozed" {
		return fmt.Errorf("unsupported advice status")
	}
	return s.repo.SetAdviceState(ctx, userID, apiaryID, adviceCode, input)
}

func (s *Service) generateTasks(ctx context.Context, apiaryID string, settings *domain.CalendarSettings, from, to time.Time) error {
	templates, err := s.repo.TaskTemplates(ctx, settings.TemplateID)
	if err != nil {
		return err
	}
	for _, template := range templates {
		dueAt, ok := templateDueAt(template, from, to, settings.DateShiftDays)
		if !ok {
			continue
		}
		if err := s.repo.InsertGeneratedApiaryTask(ctx, apiaryID, template, dueAt); err != nil {
			return err
		}
	}
	return nil
}

func (s *Service) telemetryAdvice(ctx context.Context, userID, apiaryID string, states map[string]domain.AdviceStateInput, now time.Time, includeHidden bool) ([]domain.BeekeepingAdvice, error) {
	hives, err := s.repo.ListHives(ctx, userID, apiaryID)
	if err != nil {
		return nil, err
	}
	var result []domain.BeekeepingAdvice
	for _, hive := range hives {
		latest, err := s.repo.LatestTelemetryForHive(ctx, userID, hive.ID)
		if err != nil {
			return nil, err
		}
		byMetric := map[string]domain.SensorReading{}
		for _, reading := range latest {
			byMetric[reading.MetricType] = reading
		}
		hiveLabel := hive.Number
		if hiveLabel == "" {
			hiveLabel = hive.Name
		}
		if delta, ok := byMetric["weight_change"]; ok {
			if delta.Value < 0 {
				code := "telemetry_weight_loss_" + hive.ID
				state := states[code]
				if includeHidden || !stateFiltered(state, now) {
					result = append(result, telemetryItem(code, hive.ID, "Вес улья снизился", fmt.Sprintf("Улей %s: вес снизился за сутки. Проверьте возможное роение, отбор меда, открытие улья или проблему с весами.", hiveLabel), "warning", state.Status))
				}
			} else if delta.Value > 0 && delta.Value < 0.2 {
				code := "telemetry_weak_gain_" + hive.ID
				state := states[code]
				if includeHidden || !stateFiltered(state, now) {
					result = append(result, telemetryItem(code, hive.ID, "Слабый прирост веса", fmt.Sprintf("Улей %s: слабый прирост веса за сутки. Проверьте силу семьи, наличие места и состояние матки.", hiveLabel), "notice", state.Status))
				}
			}
		}
		if temperature, ok := byMetric["temperature"]; ok && temperature.Value > 35 {
			code := "telemetry_high_temperature_" + hive.ID
			state := states[code]
			if includeHidden || !stateFiltered(state, now) {
				result = append(result, telemetryItem(code, hive.ID, "Температура выше нормы", fmt.Sprintf("Улей %s: температура выше нормы. Проверьте вентиляцию, затенение и состояние семьи.", hiveLabel), "warning", state.Status))
			}
		}
	}
	return result, nil
}

func telemetryItem(code, hiveID, title, body, severity, state string) domain.BeekeepingAdvice {
	return domain.BeekeepingAdvice{
		ID:                code,
		Code:              code,
		Title:             title,
		Body:              body,
		Category:          "telemetry",
		Severity:          severity,
		Priority:          50,
		Source:            "telemetry",
		RelatedHiveID:     &hiveID,
		ActionLabel:       "Открыть улей",
		ActionType:        "open_hive",
		IsUserDismissible: true,
		State:             state,
	}
}

func adviceTemplateActive(template domain.AdviceTemplate, date time.Time) bool {
	if template.StartMonth == nil || template.StartDay == nil || template.EndMonth == nil || template.EndDay == nil {
		return template.TriggerType == "calendar"
	}
	return monthDayInRange(date.Month(), date.Day(), *template.StartMonth, *template.StartDay, *template.EndMonth, *template.EndDay)
}

func templateDueAt(template domain.TaskTemplate, from, to time.Time, shiftDays int) (time.Time, bool) {
	if template.StartMonth == nil || template.StartDay == nil {
		return time.Time{}, false
	}
	for year := from.Year() - 1; year <= to.Year()+1; year++ {
		due := time.Date(year, time.Month(*template.StartMonth), *template.StartDay, 9, 0, 0, 0, time.UTC).AddDate(0, 0, shiftDays)
		if !due.Before(from) && !due.After(to) {
			return due, true
		}
	}
	return time.Time{}, false
}

func monthDayInRange(month time.Month, day, startMonth, startDay, endMonth, endDay int) bool {
	value := int(month)*100 + day
	start := startMonth*100 + startDay
	end := endMonth*100 + endDay
	if start <= end {
		return value >= start && value <= end
	}
	return value >= start || value <= end
}

func stateFiltered(state domain.AdviceStateInput, now time.Time) bool {
	switch state.Status {
	case "dismissed":
		return true
	case "snoozed":
		return state.SnoozedUntil == nil || state.SnoozedUntil.After(now)
	default:
		return false
	}
}

func severityRank(severity string) int {
	switch severity {
	case "critical":
		return 1
	case "warning":
		return 2
	case "notice":
		return 3
	default:
		return 4
	}
}

func validTaskStatus(status string) bool {
	switch status {
	case "planned", "due", "overdue", "completed", "dismissed", "snoozed":
		return true
	default:
		return false
	}
}
