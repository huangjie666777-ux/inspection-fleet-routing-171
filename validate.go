package main

import (
	"errors"
	"fmt"
)

// validateRequest checks all request-level invariants and returns a Grid.
func validateRequest(req *PlanRequest) (*Grid, error) {
	if req.Width < 1 || req.Width > maxGridDim || req.Height < 1 || req.Height > maxGridDim {
		return nil, fmt.Errorf("width and height must be within 1..%d", maxGridDim)
	}
	if req.MaxSteps < 1 || req.MaxSteps > maxStepsLimit {
		return nil, fmt.Errorf("max_steps must be within 1..%d", maxStepsLimit)
	}
	if req.NodeBudget < 1 || req.NodeBudget > maxBudget {
		return nil, fmt.Errorf("node_budget must be within 1..%d", maxBudget)
	}
	if len(req.Vehicles) < 1 || len(req.Vehicles) > maxVehicles {
		return nil, fmt.Errorf("vehicles must contain 1..%d entries", maxVehicles)
	}

	grid := &Grid{W: req.Width, H: req.Height, Blocked: map[Cell]bool{}}
	for _, o := range req.Obstacles {
		if o.X < 0 || o.X >= req.Width || o.Y < 0 || o.Y >= req.Height {
			return nil, fmt.Errorf("obstacle (%d,%d) out of bounds", o.X, o.Y)
		}
		grid.Blocked[o] = true
	}

	ids := map[string]bool{}
	starts := map[Cell]bool{}
	goals := map[Cell]bool{}
	for _, v := range req.Vehicles {
		if v.ID == "" {
			return nil, errors.New("vehicle id must not be empty")
		}
		if ids[v.ID] {
			return nil, fmt.Errorf("duplicate vehicle id %q", v.ID)
		}
		ids[v.ID] = true
		if !grid.free(v.Start) {
			return nil, fmt.Errorf("vehicle %q start (%d,%d) out of bounds or on obstacle", v.ID, v.Start.X, v.Start.Y)
		}
		if !grid.free(v.Goal) {
			return nil, fmt.Errorf("vehicle %q goal (%d,%d) out of bounds or on obstacle", v.ID, v.Goal.X, v.Goal.Y)
		}
		if starts[v.Start] {
			return nil, fmt.Errorf("duplicate start cell (%d,%d)", v.Start.X, v.Start.Y)
		}
		starts[v.Start] = true
		if goals[v.Goal] {
			return nil, fmt.Errorf("duplicate goal cell (%d,%d)", v.Goal.X, v.Goal.Y)
		}
		goals[v.Goal] = true
	}
	return grid, nil
}
