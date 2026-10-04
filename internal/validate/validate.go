// Package validate checks plan requests against the API limits and builds the
// validated grid used by the planner.
package validate

import (
	"fmt"
	"strings"

	"inspection-fleet-routing/internal/model"
)

const (
	MaxWidth  = 20
	MaxHeight = 20
	MinSize   = 1
	MinAgents = 1
	MaxAgents = 5
	MinSteps  = 1
	MaxSteps  = 64
	MinBudget = 1
	MaxBudget = 20000
)

// Request validates req and returns the constructed grid. The returned error
// lists every problem found; callers surface its text verbatim.
func Request(req model.PlanRequest) (model.Grid, error) {
	var errs []string
	add := func(format string, args ...any) {
		errs = append(errs, fmt.Sprintf(format, args...))
	}

	if req.Width < MinSize || req.Width > MaxWidth {
		add("width must be in [%d,%d], got %d", MinSize, MaxWidth, req.Width)
	}
	if req.Height < MinSize || req.Height > MaxHeight {
		add("height must be in [%d,%d], got %d", MinSize, MaxHeight, req.Height)
	}
	if req.MaxSteps < MinSteps || req.MaxSteps > MaxSteps {
		add("max_steps must be in [%d,%d], got %d", MinSteps, MaxSteps, req.MaxSteps)
	}
	if req.Budget < MinBudget || req.Budget > MaxBudget {
		add("budget must be in [%d,%d], got %d", MinBudget, MaxBudget, req.Budget)
	}
	if len(req.Vehicles) < MinAgents || len(req.Vehicles) > MaxAgents {
		add("vehicles count must be in [%d,%d], got %d", MinAgents, MaxAgents, len(req.Vehicles))
	}

	gridOK := req.Width >= MinSize && req.Width <= MaxWidth && req.Height >= MinSize && req.Height <= MaxHeight
	grid := model.NewGrid(req.Width, req.Height, req.Obstacles)
	if gridOK {
		for i, o := range req.Obstacles {
			if !grid.Inside(o) {
				add("obstacles[%d] (%d,%d) is outside the grid", i, o.X, o.Y)
			}
		}
	}

	ids := make(map[string]struct{}, len(req.Vehicles))
	starts := make(map[model.Cell]struct{}, len(req.Vehicles))
	goals := make(map[model.Cell]struct{}, len(req.Vehicles))
	for i, v := range req.Vehicles {
		pref := fmt.Sprintf("vehicles[%d]", i)
		if strings.TrimSpace(v.ID) == "" {
			add("%s.id must be a non-empty string", pref)
		} else if _, dup := ids[v.ID]; dup {
			add("%s.id %q is duplicated", pref, v.ID)
		} else {
			ids[v.ID] = struct{}{}
		}
		if gridOK {
			if !grid.Inside(v.Start) {
				add("%s.start (%d,%d) is outside the grid", pref, v.Start.X, v.Start.Y)
			} else if !grid.Passable(v.Start) {
				add("%s.start (%d,%d) is on an obstacle", pref, v.Start.X, v.Start.Y)
			}
			if !grid.Inside(v.Goal) {
				add("%s.goal (%d,%d) is outside the grid", pref, v.Goal.X, v.Goal.Y)
			} else if !grid.Passable(v.Goal) {
				add("%s.goal (%d,%d) is on an obstacle", pref, v.Goal.X, v.Goal.Y)
			}
		}
		if _, dup := starts[v.Start]; dup {
			add("%s.start (%d,%d) duplicates another start", pref, v.Start.X, v.Start.Y)
		}
		starts[v.Start] = struct{}{}
		if _, dup := goals[v.Goal]; dup {
			add("%s.goal (%d,%d) duplicates another goal", pref, v.Goal.X, v.Goal.Y)
		}
		goals[v.Goal] = struct{}{}
	}

	if len(errs) > 0 {
		return grid, fmt.Errorf("invalid request: %s", strings.Join(errs, "; "))
	}
	return grid, nil
}
