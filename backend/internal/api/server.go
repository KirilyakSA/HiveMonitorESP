package api

import (
	"context"
	"encoding/json"
	"errors"
	"log/slog"
	"net/http"
	"strconv"
	"strings"
	"time"

	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/auth"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/config"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/domain"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/events"
	"github.com/KirilyakSA/HiveMonitorESP/backend/internal/repository"
	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
)

type Server struct {
	cfg    config.Config
	repo   *repository.Repository
	bus    *events.Bus
	logger *slog.Logger
}

func NewServer(cfg config.Config, repo *repository.Repository, bus *events.Bus, logger *slog.Logger) *Server {
	return &Server{cfg: cfg, repo: repo, bus: bus, logger: logger}
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
			r.Get("/{apiaryID}/hives", s.listHives)
			r.Post("/{apiaryID}/hives", s.createHive)
			r.Get("/{apiaryID}/devices/unassigned", s.listUnassignedDevices)
			r.Post("/{apiaryID}/devices/{deviceUUID}/assign", s.assignDevice)
			r.Get("/{apiaryID}/events", s.apiaryEvents)
		})

		r.Route("/hives", func(r chi.Router) {
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

func (s *Server) listUnassignedDevices(w http.ResponseWriter, r *http.Request) {
	devices, err := s.repo.ListUnassignedDevices(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "apiaryID"))
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, devices)
}

func (s *Server) assignDevice(w http.ResponseWriter, r *http.Request) {
	var input struct {
		HiveID     string `json:"hive_id"`
		ImportMode string `json:"import_mode"`
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
	)
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusCreated, assignment)
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

func (s *Server) apiaryEvents(w http.ResponseWriter, r *http.Request) {
	limit, _ := strconv.Atoi(r.URL.Query().Get("limit"))
	events, err := s.repo.EventsForApiary(r.Context(), userIDFromContext(r.Context()), chi.URLParam(r, "apiaryID"), limit)
	if err != nil {
		s.handleRepoError(w, err)
		return
	}
	writeJSON(w, http.StatusOK, events)
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

func writeJSON(w http.ResponseWriter, status int, value any) {
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
	s.internalError(w, err)
}
