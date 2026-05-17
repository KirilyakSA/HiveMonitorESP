package api

import (
	"context"
	"encoding/json"
	"errors"
	"log/slog"
	"net/http"
	"reflect"
	"strconv"
	"strings"
	"time"

	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/auth"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/beekeeping"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/config"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/domain"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/events"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/repository"
	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
)

type Server struct {
	cfg      config.Config
	repo     *repository.Repository
	bus      *events.Bus
	logger   *slog.Logger
	calendar *beekeeping.Service
	commands commandPublisher
}

type commandPublisher interface {
	PublishDeviceCommand(ctx context.Context, command domain.DeviceCommand) ([]string, error)
}

func NewServer(cfg config.Config, repo *repository.Repository, bus *events.Bus, logger *slog.Logger, publishers ...commandPublisher) *Server {
	var publisher commandPublisher
	if len(publishers) > 0 {
		publisher = publishers[0]
	}
	return &Server{cfg: cfg, repo: repo, bus: bus, logger: logger, calendar: beekeeping.New(repo), commands: publisher}
}

func (s *Server) Routes() http.Handler {
	r := chi.NewRouter()
	r.Use(middleware.RequestID)
	r.Use(middleware.RealIP)
	r.Use(middleware.Recoverer)
	r.Use(middleware.Timeout(30 * time.Second))

	r.Get("/healthz", func(w http.ResponseWriter, r *http.Request) {
		writeJSON(w, http.StatusOK, map[string]string{"status": "ok"})
	})

	r.Route("/auth", func(r chi.Router) {
		r.Post("/register", s.register)
		r.Post("/login", s.login)
	})

	r.Group(func(r chi.Router) {
		r.Use(s.authMiddleware)
		r.Get("/me", s.me)

		r.Route("/organizations", func(r chi.Router) {
			r.Get("/", s.listOrganizations)
			r.Post("/", s.createOrganization)
		})

		r.Route("/apiaries", func(r chi.Router) {
			r.Get("/", s.listApiaries)
			r.Post("/", s.createApiary)
			r.Delete("/{apiaryID}", s.deleteApiary)
			r.Get("/{apiaryID}/hives", s.listHives)
			r.Post("/{apiaryID}/hives", s.createHive)
			r.Get("/{apiaryID}/devices/unassigned", s.listUnassignedDevices)
			r.Get("/{apiaryID}/devices/{deviceUUID}", s.getDevice)
			r.Delete("/{apiaryID}/devices/{deviceUUID}", s.deleteDevice)
			r.Post("/{apiaryID}/devices/{deviceUUID}/assign", s.assignDevice)
			r.Get("/{apiaryID}/devices/{deviceUUID}/commands", s.listDeviceCommands)
			r.Post("/{apiaryID}/devices/{deviceUUID}/commands", s.createDeviceCommand)
			r.Get("/{apiaryID}/events", s.apiaryEvents)
			r.Get("/{apiaryID}/advice", s.apiaryAdvice)
			r.Patch("/{apiaryID}/advice/{adviceCode}", s.updateAdviceState)
			r.Get("/{apiaryID}/calendar/tasks", s.apiaryCalendarTasks)
			r.Post("/{apiaryID}/calendar/tasks", s.createApiaryTask)
			r.Patch("/{apiaryID}/calendar/tasks/{taskID}", s.updateApiaryTask)
		})

		r.Route("/hives", func(r chi.Router) {
			r.Delete("/{hiveID}", s.deleteHive)
			r.Get("/{hiveID}/scale/profile", s.hiveScaleProfile)
			r.Post("/{hiveID}/scale/tare", s.saveHiveTare)
			r.Post("/{hiveID}/scale/supers/remove", s.removeHiveSuperTare)
			r.Get("/{hiveID}/telemetry/latest", s.latestTelemetry)
			r.Get("/{hiveID}/telemetry/history", s.telemetryHistory)
			r.Get("/{hiveID}/events", s.hiveEvents)
		})
	})

	return r
}

type contextKey string

const userIDKey contextKey = "userID"

func (s *Server) authMiddleware(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		header := r.Header.Get("Authorization")
		tokenValue, ok := strings.CutPrefix(header, "Bearer ")
		if !ok || tokenValue == "" {
			writeError(w, http.StatusUnauthorized, "missing bearer token")
			return
		}
		claims, err := auth.ParseToken(s.cfg.JWTSecret, tokenValue)
		if err != nil {
			writeError(w, http.StatusUnauthorized, "invalid token")
			return
		}
		ctx := context.WithValue(r.Context(), userIDKey, claims.UserID)
		next.ServeHTTP(w, r.WithContext(ctx))
	})
}

func userIDFromContext(ctx context.Context) string {
	value, _ := ctx.Value(userIDKey).(string)
	return value
}

func (s *Server) register(w http.ResponseWriter, r *http.Request) {
	var input struct {
		Email    string `json:"email"`
		Name     string `json:"name"`
		Password string `json:"password"`
	}
	if !decodeJSON(w, r, &input) {
		return
	}
	input.Email = strings.TrimSpace(strings.ToLower(input.Email))
	if input.Email == "" || len(input.Password) < 8 {
		writeError(w, http.StatusBadRequest, "email and password with at least 8 characters are required")
		return
	}

	hash, err := auth.HashPassword(input.Password)
	if err != nil {
		s.internalError(w, err)
		return
	}
	user, err := s.repo.CreateUser(r.Context(), input.Email, input.Name, hash)
	if err != nil {
		s.internalError(w, err)
		return
	}
	token, err := auth.CreateToken(s.cfg.JWTSecret, user.ID, user.Email, s.cfg.AccessTokenTTL)
	if err != nil {
		s.internalError(w, err)
		return
	}
	writeJSON(w, http.StatusCreated, map[string]any{"access_token": token, "user": publicUser(user)})
}

func (s *Server) login(w http.ResponseWriter, r *http.Request) {
	var input struct {
		Email    string `json:"email"`
		Password string `json:"password"`
	}
	if !decodeJSON(w, r, &input) {
		return
	}
	user, err := s.repo.GetUserByEmail(r.Context(), strings.TrimSpace(strings.ToLower(input.Email)))
	if err != nil || !auth.CheckPassword(user.PasswordHash, input.Password) {
		writeError(w, http.StatusUnauthorized, "invalid email or password")
		return
	}
	token, err := auth.CreateToken(s.cfg.JWTSecret, user.ID, user.Email, s.cfg.AccessTokenTTL)
	if err != nil {
		s.internalError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{"access_token": token, "user": publicUser(user)})
}

func (s *Server) me(w http.ResponseWriter, r *http.Request) {
	user, err := s.repo.GetUserByID(r.Context(), userIDFromContext(r.Context()))
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, publicUser(user))
}

func publicUser(user *domain.User) map[string]any {
	return map[string]any{
		"id":         user.ID,
		"email":      user.Email,
		"name":       user.Name,
		"created_at": user.CreatedAt,
	}
}

func (s *Server) createOrganization(w http.ResponseWriter, r *http.Request) {
	var input struct {
		Name string `json:"name"`
	}
	if !decodeJSON(w, r, &input) {
		return
	}
	if strings.TrimSpace(input.Name) == "" {
		writeError(w, http.StatusBadRequest, "name is required")
		return
	}
	org, err := s.repo.CreateOrganization(r.Context(), userIDFromContext(r.Context()), input.Name)
	if err != nil {
		s.internalError(w, err)
		return
	}
	writeJSON(w, http.StatusCreated, org)
}

func (s *Server) listOrganizations(w http.ResponseWriter, r *http.Request) {
	orgs, err := s.repo.ListOrganizations(r.Context(), userIDFromContext(r.Context()))
	if err != nil {
		s.internalError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, orgs)
}

func (s *Server) createApiary(w http.ResponseWriter, r *http.Request) {
	var input domain.Apiary
	if !decodeJSON(w, r, &input) {
		return
	}
	if input.OrganizationID == "" || input.Name == "" {
		writeError(w, http.StatusBadRequest, "organization_id and name are required")
		return
	}
	apiary, err := s.repo.CreateApiary(r.Context(), userIDFromContext(r.Context()), input)
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusCreated, apiary)
}

func (s *Server) listApiaries(w http.ResponseWriter, r *http.Request) {
	apiaries, err := s.repo.ListApiaries(r.Context(), userIDFromContext(r.Context()), r.URL.Query().Get("organization_id"))
	if err != nil {
		s.internalError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, apiaries)
}

func (s *Server) deleteApiary(w http.ResponseWriter, r *http.Request) {
	var input struct {
		ConfirmName string `json:"confirm_name"`
	}
	if !decodeJSON(w, r, &input) {
		return
	}
	if strings.TrimSpace(input.ConfirmName) == "" {
		writeError(w, http.StatusBadRequest, "confirm_name is required")
		return
	}
	if err := s.repo.DeleteApiary(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "apiaryID"), input.ConfirmName); err != nil {
		s.handleRepoError(w, err)
		return
	}
	w.WriteHeader(http.StatusNoContent)
}

func (s *Server) createHive(w http.ResponseWriter, r *http.Request) {
	var input domain.Hive
	if !decodeJSON(w, r, &input) {
		return
	}
	input.ApiaryID = chi.URLParam(r, "apiaryID")
	if input.Name == "" {
		writeError(w, http.StatusBadRequest, "name is required")
		return
	}
	hive, err := s.repo.CreateHive(r.Context(), userIDFromContext(r.Context()), input)
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusCreated, hive)
}

func (s *Server) listHives(w http.ResponseWriter, r *http.Request) {
	hives, err := s.repo.ListHives(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "apiaryID"))
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, hives)
}

func (s *Server) deleteHive(w http.ResponseWriter, r *http.Request) {
	if err := s.repo.DeleteHive(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "hiveID")); err != nil {
		s.handleRepoError(w, err)
		return
	}
	w.WriteHeader(http.StatusNoContent)
}

func (s *Server) listUnassignedDevices(w http.ResponseWriter, r *http.Request) {
	devices, err := s.repo.ListUnassignedDevices(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "apiaryID"))
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, devices)
}

func (s *Server) getDevice(w http.ResponseWriter, r *http.Request) {
	device, err := s.repo.GetDevice(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "apiaryID"), chi.URLParam(r, "deviceUUID"))
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, device)
}

func (s *Server) deleteDevice(w http.ResponseWriter, r *http.Request) {
	if err := s.repo.DeleteDevice(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "apiaryID"), chi.URLParam(r, "deviceUUID")); err != nil {
		s.handleRepoError(w, err)
		return
	}
	w.WriteHeader(http.StatusNoContent)
}

func (s *Server) assignDevice(w http.ResponseWriter, r *http.Request) {
	var input struct {
		HiveID          string `json:"hive_id"`
		ImportMode      string `json:"import_mode"`
		ReplaceExisting bool   `json:"replace_existing"`
	}
	if !decodeJSON(w, r, &input) {
		return
	}
	if input.HiveID == "" {
		writeError(w, http.StatusBadRequest, "hive_id is required")
		return
	}
	assignment, err := s.repo.AssignDeviceToHive(
		r.Context(),
		userIDFromContext(r.Context()),
		chi.URLParam(r, "apiaryID"),
		chi.URLParam(r, "deviceUUID"),
		input.HiveID,
		input.ImportMode,
		input.ReplaceExisting,
	)
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusCreated, assignment)
}

func (s *Server) listDeviceCommands(w http.ResponseWriter, r *http.Request) {
	limit, _ := strconv.Atoi(r.URL.Query().Get("limit"))
	commands, err := s.repo.ListDeviceCommands(
		r.Context(),
		userIDFromContext(r.Context()),
		chi.URLParam(r, "apiaryID"),
		chi.URLParam(r, "deviceUUID"),
		limit,
	)
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, commands)
}

func (s *Server) createDeviceCommand(w http.ResponseWriter, r *http.Request) {
	var input domain.CreateDeviceCommandInput
	if !decodeJSON(w, r, &input) {
		return
	}
	input.Command = strings.TrimSpace(input.Command)
	if !allowedDeviceCommand(input.Command) {
		writeError(w, http.StatusBadRequest, "unsupported command")
		return
	}
	if len(input.Payload) == 0 {
		input.Payload = json.RawMessage(`{}`)
	}
	if !json.Valid(input.Payload) {
		writeError(w, http.StatusBadRequest, "payload must be valid JSON")
		return
	}

	command, err := s.repo.CreateDeviceCommand(
		r.Context(),
		userIDFromContext(r.Context()),
		chi.URLParam(r, "apiaryID"),
		chi.URLParam(r, "deviceUUID"),
		input,
	)
	if err != nil {
		s.handleRepoError(w, err)
		return
	}

	if s.commands == nil {
		updated, updateErr := s.repo.MarkDeviceCommandFailed(r.Context(), command.ID, "mqtt command publisher is not configured", nil)
		if updateErr != nil {
			s.internalError(w, updateErr)
			return
		}
		writeJSON(w, http.StatusCreated, updated)
		return
	}

	publishCtx, cancel := context.WithTimeout(r.Context(), 5*time.Second)
	defer cancel()
	topics, err := s.commands.PublishDeviceCommand(publishCtx, *command)
	if err != nil {
		updated, updateErr := s.repo.MarkDeviceCommandFailed(r.Context(), command.ID, err.Error(), topics)
		if updateErr != nil {
			s.internalError(w, updateErr)
			return
		}
		writeJSON(w, http.StatusCreated, updated)
		return
	}
	updated, err := s.repo.MarkDeviceCommandPublished(r.Context(), command.ID, topics)
	if err != nil {
		s.internalError(w, err)
		return
	}
	writeJSON(w, http.StatusCreated, updated)
}

func allowedDeviceCommand(command string) bool {
	switch command {
	case "reboot", "restart", "firmware_update", "config_update", "hold_config_session", "capture_weight", "finish_config_session":
		return true
	default:
		return false
	}
}

func (s *Server) latestTelemetry(w http.ResponseWriter, r *http.Request) {
	readings, err := s.repo.LatestTelemetryForHive(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "hiveID"))
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, readings)
}

func (s *Server) telemetryHistory(w http.ResponseWriter, r *http.Request) {
	now := time.Now().UTC()
	from := now.Add(-24 * time.Hour)
	to := now
	if value := r.URL.Query().Get("from"); value != "" {
		parsed, err := time.Parse(time.RFC3339, value)
		if err != nil {
			writeError(w, http.StatusBadRequest, "from must be RFC3339")
			return
		}
		from = parsed
	}
	if value := r.URL.Query().Get("to"); value != "" {
		parsed, err := time.Parse(time.RFC3339, value)
		if err != nil {
			writeError(w, http.StatusBadRequest, "to must be RFC3339")
			return
		}
		to = parsed
	}
	limit, _ := strconv.Atoi(r.URL.Query().Get("limit"))
	readings, err := s.repo.TelemetryHistoryForHive(
		r.Context(),
		userIDFromContext(r.Context()),
		chi.URLParam(r, "hiveID"),
		from,
		to,
		r.URL.Query().Get("metric"),
		limit,
	)
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, readings)
}

func (s *Server) hiveScaleProfile(w http.ResponseWriter, r *http.Request) {
	profile, err := s.repo.HiveScaleProfile(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "hiveID"))
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, profile)
}

func (s *Server) saveHiveTare(w http.ResponseWriter, r *http.Request) {
	var input domain.SaveHiveTareInput
	if !decodeJSON(w, r, &input) {
		return
	}
	if len(input.Metadata) == 0 {
		input.Metadata = json.RawMessage(`{}`)
	}
	event, err := s.repo.SaveHiveTare(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "hiveID"), input)
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusCreated, event)
}

func (s *Server) removeHiveSuperTare(w http.ResponseWriter, r *http.Request) {
	var input domain.RemoveHiveSuperInput
	if !decodeJSON(w, r, &input) {
		return
	}
	if len(input.Metadata) == 0 {
		input.Metadata = json.RawMessage(`{}`)
	}
	event, err := s.repo.RemoveHiveSuperTare(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "hiveID"), input)
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusCreated, event)
}

func (s *Server) apiaryEvents(w http.ResponseWriter, r *http.Request) {
	limit, _ := strconv.Atoi(r.URL.Query().Get("limit"))
	events, err := s.repo.EventsForApiary(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "apiaryID"), limit)
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, events)
}

func (s *Server) apiaryAdvice(w http.ResponseWriter, r *http.Request) {
	date := time.Now().UTC()
	if value := r.URL.Query().Get("date"); value != "" {
		parsed, err := parseDate(value)
		if err != nil {
			writeError(w, http.StatusBadRequest, "date must be YYYY-MM-DD or RFC3339")
			return
		}
		date = parsed
	}
	includeHidden := r.URL.Query().Get("include_hidden") == "true"
	items, err := s.calendar.ActiveAdvice(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "apiaryID"), date, includeHidden)
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{"items": items})
}

func (s *Server) updateAdviceState(w http.ResponseWriter, r *http.Request) {
	var input domain.AdviceStateInput
	if !decodeJSON(w, r, &input) {
		return
	}
	if err := s.calendar.SetAdviceState(
		r.Context(),
		userIDFromContext(r.Context()),
		chi.URLParam(r, "apiaryID"),
		chi.URLParam(r, "adviceCode"),
		input,
	); err != nil {
		if strings.Contains(err.Error(), "unsupported") {
			writeError(w, http.StatusBadRequest, err.Error())
			return
		}
		s.handleRepoError(w, err)
		return
	}
	w.WriteHeader(http.StatusNoContent)
}

func (s *Server) apiaryCalendarTasks(w http.ResponseWriter, r *http.Request) {
	now := time.Now().UTC()
	from := time.Date(now.Year(), now.Month(), 1, 0, 0, 0, 0, time.UTC)
	to := from.AddDate(0, 1, 0).Add(-time.Nanosecond)
	if value := r.URL.Query().Get("from"); value != "" {
		parsed, err := parseDate(value)
		if err != nil {
			writeError(w, http.StatusBadRequest, "from must be YYYY-MM-DD or RFC3339")
			return
		}
		from = parsed
	}
	if value := r.URL.Query().Get("to"); value != "" {
		parsed, err := parseDate(value)
		if err != nil {
			writeError(w, http.StatusBadRequest, "to must be YYYY-MM-DD or RFC3339")
			return
		}
		to = parsed
	}
	items, err := s.calendar.CalendarTasks(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "apiaryID"), from, to)
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{"items": items})
}

func (s *Server) createApiaryTask(w http.ResponseWriter, r *http.Request) {
	var input domain.CreateApiaryTaskInput
	if !decodeJSON(w, r, &input) {
		return
	}
	task, err := s.calendar.CreateTask(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "apiaryID"), input)
	if err != nil {
		if strings.Contains(err.Error(), "required") {
			writeError(w, http.StatusBadRequest, err.Error())
			return
		}
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusCreated, task)
}

func (s *Server) updateApiaryTask(w http.ResponseWriter, r *http.Request) {
	var input domain.UpdateApiaryTaskInput
	if !decodeJSON(w, r, &input) {
		return
	}
	task, err := s.calendar.UpdateTask(
		r.Context(),
		userIDFromContext(r.Context()),
		chi.URLParam(r, "apiaryID"),
		chi.URLParam(r, "taskID"),
		input,
	)
	if err != nil {
		if strings.Contains(err.Error(), "unsupported") {
			writeError(w, http.StatusBadRequest, err.Error())
			return
		}
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, task)
}

func (s *Server) hiveEvents(w http.ResponseWriter, r *http.Request) {
	limit, _ := strconv.Atoi(r.URL.Query().Get("limit"))
	events, err := s.repo.EventsForHive(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "hiveID"), limit)
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, events)
}

func decodeJSON(w http.ResponseWriter, r *http.Request, dst any) bool {
	decoder := json.NewDecoder(r.Body)
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(dst); err != nil {
		writeError(w, http.StatusBadRequest, "invalid json")
		return false
	}
	return true
}

func parseDate(value string) (time.Time, error) {
	if parsed, err := time.Parse("2006-01-02", value); err == nil {
		return parsed, nil
	}
	return time.Parse(time.RFC3339, value)
}

func writeJSON(w http.ResponseWriter, status int, value any) {
	if value != nil {
		rv := reflect.ValueOf(value)
		if rv.Kind() == reflect.Slice && rv.IsNil() {
			value = reflect.MakeSlice(rv.Type(), 0, 0).Interface()
		}
	}
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	_ = json.NewEncoder(w).Encode(value)
}

func writeError(w http.ResponseWriter, status int, message string) {
	writeJSON(w, status, map[string]string{"error": message})
}

func (s *Server) internalError(w http.ResponseWriter, err error) {
	s.logger.Error("api error", "error", err)
	writeError(w, http.StatusInternalServerError, "internal error")
}

func (s *Server) handleRepoError(w http.ResponseWriter, err error) {
	if errors.Is(err, repository.ErrNotFound) {
		writeError(w, http.StatusNotFound, "not found")
		return
	}
	if errors.Is(err, repository.ErrInvalidInput) {
		writeError(w, http.StatusBadRequest, err.Error())
		return
	}
	s.internalError(w, err)
}
