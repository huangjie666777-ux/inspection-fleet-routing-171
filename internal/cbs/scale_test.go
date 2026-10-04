package cbs

import (
	"context"
	"fmt"
	"testing"
	"time"

	"inspection-fleet-routing/internal/model"
)

func TestFiveVehiclesOnFullGrid(t *testing.T) {
	g := model.NewGrid(20, 20, nil)
	veh := []model.Vehicle{
		{ID: "v0", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 19, Y: 19}},
		{ID: "v1", Start: model.Cell{X: 19, Y: 0}, Goal: model.Cell{X: 0, Y: 19}},
		{ID: "v2", Start: model.Cell{X: 0, Y: 19}, Goal: model.Cell{X: 19, Y: 0}},
		{ID: "v3", Start: model.Cell{X: 19, Y: 19}, Goal: model.Cell{X: 0, Y: 0}},
		{ID: "v4", Start: model.Cell{X: 10, Y: 0}, Goal: model.Cell{X: 10, Y: 19}},
	}
	start := time.Now()
	res := Solve(context.Background(), g, veh, 64, 20000)
	t.Logf("outcome=%d cost=%d expanded=%d elapsed=%s", res.Outcome, res.TotalCost, res.ExpandedNodes, time.Since(start))
	if res.Outcome == Found {
		verifySim(t, g, veh, res, 64)
		bfA := 19 + 19
		bfV4 := 19
		lower := 4*bfA + bfV4
		if res.TotalCost < lower {
			t.Fatalf("cost %d below Manhattan lower bound %d", res.TotalCost, lower)
		}
		fmt.Printf("5-vehicle plan cost=%d expanded=%d\n", res.TotalCost, res.ExpandedNodes)
	}
}
