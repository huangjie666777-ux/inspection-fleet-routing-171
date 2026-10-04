package validate

import (
	"strings"
	"testing"

	"inspection-fleet-routing/internal/model"
)

func TestValidRequest(t *testing.T) {
	req := model.PlanRequest{
		Width: 3, Height: 2,
		Obstacles: []model.Cell{{X: 0, Y: 0}},
		Vehicles: []model.Vehicle{
			{ID: "a", Start: model.Cell{X: 1, Y: 0}, Goal: model.Cell{X: 2, Y: 0}},
		},
		MaxSteps: 5, Budget: 100,
	}
	if _, err := Request(req); err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
}

func TestInvalidRequests(t *testing.T) {
	base := func() model.PlanRequest {
		return model.PlanRequest{
			Width: 20, Height: 20, MaxSteps: 64, Budget: 20000,
			Vehicles: []model.Vehicle{{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 1, Y: 1}}},
		}
	}
	cases := []struct {
		name string
		mut  func(*model.PlanRequest)
		want string
	}{
		{"width zero", func(r *model.PlanRequest) { r.Width = 0 }, "width"},
		{"width huge", func(r *model.PlanRequest) { r.Width = 21 }, "width"},
		{"height neg", func(r *model.PlanRequest) { r.Height = -1 }, "height"},
		{"steps zero", func(r *model.PlanRequest) { r.MaxSteps = 0 }, "max_steps"},
		{"steps huge", func(r *model.PlanRequest) { r.MaxSteps = 65 }, "max_steps"},
		{"budget zero", func(r *model.PlanRequest) { r.Budget = 0 }, "budget"},
		{"budget huge", func(r *model.PlanRequest) { r.Budget = 20001 }, "budget"},
		{"no vehicles", func(r *model.PlanRequest) { r.Vehicles = nil }, "vehicles count"},
		{"six vehicles", func(r *model.PlanRequest) {
			for i := 0; i < 6; i++ {
				r.Vehicles = append(r.Vehicles, model.Vehicle{ID: string(rune('a' + i)), Start: model.Cell{X: i, Y: 0}, Goal: model.Cell{X: i, Y: 1}})
			}
		}, "vehicles count"},
		{"empty id", func(r *model.PlanRequest) { r.Vehicles[0].ID = " " }, "id"},
		{"dup id", func(r *model.PlanRequest) {
			r.Vehicles = append(r.Vehicles, model.Vehicle{ID: "a", Start: model.Cell{X: 2, Y: 2}, Goal: model.Cell{X: 3, Y: 3}})
		}, "duplicated"},
		{"start out", func(r *model.PlanRequest) { r.Vehicles[0].Start = model.Cell{X: 20, Y: 0} }, "outside"},
		{"goal neg", func(r *model.PlanRequest) { r.Vehicles[0].Goal = model.Cell{X: -1, Y: 0} }, "outside"},
		{"start on obstacle", func(r *model.PlanRequest) {
			r.Obstacles = []model.Cell{{X: 0, Y: 0}}
		}, "obstacle"},
		{"dup start", func(r *model.PlanRequest) {
			r.Vehicles = append(r.Vehicles, model.Vehicle{ID: "b", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 2, Y: 2}})
		}, "duplicates another start"},
		{"dup goal", func(r *model.PlanRequest) {
			r.Vehicles = append(r.Vehicles, model.Vehicle{ID: "b", Start: model.Cell{X: 2, Y: 2}, Goal: model.Cell{X: 1, Y: 1}})
		}, "duplicates another goal"},
		{"obstacle out", func(r *model.PlanRequest) {
			r.Obstacles = []model.Cell{{X: -1, Y: 5}}
		}, "outside the grid"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			req := base()
			tc.mut(&req)
			_, err := Request(req)
			if err == nil {
				t.Fatal("expected validation error")
			}
			if !strings.Contains(err.Error(), tc.want) {
				t.Fatalf("error %q does not mention %q", err.Error(), tc.want)
			}
		})
	}
}
