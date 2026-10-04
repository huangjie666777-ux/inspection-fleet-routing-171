// Package httpapi exposes the planner over HTTP using chi.
package httpapi

import (
	"context"
	"encoding/json"
	"errors"
	"log"
	"net/http"

	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"

	"inspection-fleet-routing/internal/cbs"
	"inspection-fleet-routing/internal/model"
	"inspection-fleet-routing/internal/validate"
)

// maxBodyBytes bounds a single planning request payload.
const maxBodyBytes = 1 << 20

// Router builds the application mux. Each request gets an independent
// constraint tree and budget; the request context drives cancellation.
func Router() http.Handler {
	r := chi.NewRouter()
	r.Use(middleware.RealIP)
	r.Use(middleware.RequestID)
	r.Use(middleware.Recoverer)
	r.Get("/healthz", health)
	r.Post("/plan", plan)
	return r
}

func health(w http.ResponseWriter, _ *http.Request) {
	writeJSON(w, http.StatusOK, map[string]string{"status": "ok"})
}

func plan(w http.ResponseWriter, r *http.Request) {
	r.Body = http.MaxBytesReader(w, r.Body, maxBodyBytes)
	var req model.PlanRequest
	dec := json.NewDecoder(r.Body)
	dec.DisallowUnknownFields()
	if err := dec.Decode(&req); err != nil {
		writePlanError(w, http.StatusBadRequest, decodeErrorMessage(err))
		return
	}
	// Only one JSON object is accepted.
	if dec.More() {
		writePlanError(w, http.StatusBadRequest, "request body must contain a single JSON object")
		return
	}

	grid, err := validate.Request(req)
	if err != nil {
		writePlanError(w, http.StatusBadRequest, err.Error())
		return
	}

	res := cbs.Solve(r.Context(), grid, req.Vehicles, req.MaxSteps, req.Budget)
	resp := model.PlanResponse{
		Budget:          req.Budget,
		ExpandedNodes:   res.ExpandedNodes,
		RemainingBudget: req.Budget - res.ExpandedNodes,
	}
	switch res.Outcome {
	case cbs.Found:
		resp.Status = model.StatusOptimal
		resp.TotalCost = res.TotalCost
		resp.Vehicles = make([]model.VehicleResult, len(req.Vehicles))
		for i, v := range req.Vehicles {
			arrival := res.Arrivals[i]
			resp.Vehicles[i] = model.VehicleResult{
				ID:          v.ID,
				Path:        res.Paths[i],
				ArrivalTime: arrival,
				Cost:        arrival,
			}
		}
		writeJSON(w, http.StatusOK, resp)
	case cbs.NoSolution:
		resp.Status = model.StatusInfeasible
		writeJSON(w, http.StatusOK, resp)
	case cbs.Undecided:
		if errors.Is(r.Context().Err(), context.Canceled) {
			resp.Status = model.StatusCancelled
		} else {
			resp.Status = model.StatusUndecided
		}
		writeJSON(w, http.StatusOK, resp)
	default:
		writePlanError(w, http.StatusInternalServerError, "unknown planner outcome")
	}
}

func decodeErrorMessage(err error) string {
	// Keep the net/http max-bytes message readable.
	var mbErr *http.MaxBytesError
	if errors.As(err, &mbErr) {
		return "request body too large"
	}
	return "invalid JSON: " + err.Error()
}

func writePlanError(w http.ResponseWriter, code int, msg string) {
	writeJSON(w, code, model.PlanResponse{Status: "error", Error: msg})
}

func writeJSON(w http.ResponseWriter, code int, v any) {
	w.Header().Set("Content-Type", "application/json; charset=utf-8")
	w.WriteHeader(code)
	if err := json.NewEncoder(w).Encode(v); err != nil {
		log.Printf("write response: %v", err)
	}
}
