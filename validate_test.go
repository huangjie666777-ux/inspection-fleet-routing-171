package main

import "testing"

func validReq() *PlanRequest {
	return &PlanRequest{
		Width: 4, Height: 4, MaxSteps: 32, NodeBudget: 1000,
		Obstacles: []Cell{{1, 1}},
		Vehicles: []VehicleSpec{
			{ID: "a", Start: Cell{0, 0}, Goal: Cell{3, 3}},
			{ID: "b", Start: Cell{3, 0}, Goal: Cell{0, 3}},
		},
	}
}

func TestValidateOK(t *testing.T) {
	if _, err := validateRequest(validReq()); err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
}

func TestValidateRejects(t *testing.T) {
	cases := map[string]func(*PlanRequest){
		"grid too large": func(r *PlanRequest) { r.Width = 21 },
		"zero height":    func(r *PlanRequest) { r.Height = 0 },
		"too many cars": func(r *PlanRequest) {
			r.Vehicles = append(r.Vehicles, r.Vehicles[0], r.Vehicles[1], r.Vehicles[0], r.Vehicles[1])
		},
		"zero vehicles":     func(r *PlanRequest) { r.Vehicles = nil },
		"steps too big":     func(r *PlanRequest) { r.MaxSteps = 65 },
		"steps zero":        func(r *PlanRequest) { r.MaxSteps = 0 },
		"budget too big":    func(r *PlanRequest) { r.NodeBudget = 20001 },
		"budget zero":       func(r *PlanRequest) { r.NodeBudget = 0 },
		"obstacle oob":      func(r *PlanRequest) { r.Obstacles = []Cell{{9, 9}} },
		"start on obstacle": func(r *PlanRequest) { r.Vehicles[0].Start = Cell{1, 1} },
		"goal on obstacle":  func(r *PlanRequest) { r.Vehicles[0].Goal = Cell{1, 1} },
		"start oob":         func(r *PlanRequest) { r.Vehicles[0].Start = Cell{-1, 0} },
		"dup id":            func(r *PlanRequest) { r.Vehicles[1].ID = "a" },
		"empty id":          func(r *PlanRequest) { r.Vehicles[0].ID = "" },
		"dup start":         func(r *PlanRequest) { r.Vehicles[1].Start = r.Vehicles[0].Start },
		"dup goal":          func(r *PlanRequest) { r.Vehicles[1].Goal = r.Vehicles[0].Goal },
	}
	for name, mutate := range cases {
		req := validReq()
		mutate(req)
		if _, err := validateRequest(req); err == nil {
			t.Errorf("%s: expected error, got nil", name)
		}
	}
}
