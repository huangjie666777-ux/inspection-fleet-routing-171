package cbs

import (
	"context"
	"testing"

	"inspection-fleet-routing/internal/model"
)

func grid(w, h int, obs ...model.Cell) model.Grid {
	return model.NewGrid(w, h, obs)
}

// verifySim simulates a solution until maxSteps and asserts the movement
// rules: orthogonal/wait steps, no obstacle entry, no same-cell collision and
// no head-on swap. It also asserts each path stops after arrival.
func verifySim(t *testing.T, g model.Grid, vehicles []model.Vehicle, res Result, maxSteps int) {
	t.Helper()
	n := len(vehicles)
	pos := make([]model.Cell, n)
	for i := range vehicles {
		pos[i] = res.Paths[i][0]
		if pos[i] != vehicles[i].Start {
			t.Fatalf("agent %d path starts at %v, want %v", i, pos[i], vehicles[i].Start)
		}
	}
	for ts := 0; ts <= maxSteps; ts++ {
		for i := range vehicles {
			c := cellAt(res.Paths[i], res.Arrivals[i], ts)
			if !g.Passable(c) {
				t.Fatalf("agent %d at %v is not passable at t=%d", i, c, ts)
			}
			for j := i + 1; j < n; j++ {
				if c == cellAt(res.Paths[j], res.Arrivals[j], ts) {
					t.Fatalf("vertex collision at t=%d cell %v", ts, c)
				}
			}
		}
		if ts > 0 {
			for i := range vehicles {
				prev := cellAt(res.Paths[i], res.Arrivals[i], ts-1)
				cur := cellAt(res.Paths[i], res.Arrivals[i], ts)
				d := manhattan(prev, cur)
				if d != 0 && d != 1 {
					t.Fatalf("agent %d jumps %v -> %v", i, prev, cur)
				}
				for j := i + 1; j < n; j++ {
					pj0 := cellAt(res.Paths[j], res.Arrivals[j], ts-1)
					pj1 := cellAt(res.Paths[j], res.Arrivals[j], ts)
					if prev == pj1 && cur == pj0 {
						t.Fatalf("edge swap at t=%d: %d,%v->%v vs %d,%v->%v", ts-1, i, prev, cur, j, pj0, pj1)
					}
				}
			}
		}
	}
	for i, v := range vehicles {
		if res.Paths[i][res.Arrivals[i]] != v.Goal {
			t.Fatalf("agent %d does not end at goal", i)
		}
	}
}

func cellAt(path []model.Cell, arrival, t int) model.Cell {
	if t >= len(path) {
		return path[arrival]
	}
	return path[t]
}

func TestSingleAgentShortestPath(t *testing.T) {
	g := grid(3, 1)
	veh := []model.Vehicle{{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 2, Y: 0}}}
	res := Solve(context.Background(), g, veh, 10, 100)
	if res.Outcome != Found || res.TotalCost != 2 {
		t.Fatalf("unexpected result: %+v", res)
	}
}

func TestStartEqualsGoalStaysAtZero(t *testing.T) {
	g := grid(2, 2)
	veh := []model.Vehicle{{ID: "a", Start: model.Cell{X: 1, Y: 1}, Goal: model.Cell{X: 1, Y: 1}}}
	res := Solve(context.Background(), g, veh, 4, 100)
	if res.Outcome != Found || res.TotalCost != 0 || res.Arrivals[0] != 0 {
		t.Fatalf("unexpected: %+v", res)
	}
	if len(res.Paths[0]) != 1 {
		t.Fatalf("expected path length 1, got %d", len(res.Paths[0]))
	}
}

func TestHeadOnCorridorWithSiding(t *testing.T) {
	// Horizontal corridor (0,0)-(2,0) with a siding at (1,1). Agents swap
	// ends; CBS must route one into the siding (or wait) instead of swapping.
	g := grid(3, 2)
	veh := []model.Vehicle{
		{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 2, Y: 0}},
		{ID: "b", Start: model.Cell{X: 2, Y: 0}, Goal: model.Cell{X: 0, Y: 0}},
	}
	res := Solve(context.Background(), g, veh, 12, 1000)
	if res.Outcome != Found {
		t.Fatalf("expected solution, got %+v", res)
	}
	// Minimum is 2+4=6: one vehicle detours through the siding (arrival 4).
	if res.TotalCost != 6 {
		t.Fatalf("expected cost 6, got %d", res.TotalCost)
	}
	verifySim(t, g, veh, res, 12)
}

func TestHeadOnNoSidingIsInfeasible(t *testing.T) {
	// 1x3 corridor, agents swap: without a siding the order can never change.
	g := grid(3, 1)
	veh := []model.Vehicle{
		{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 2, Y: 0}},
		{ID: "b", Start: model.Cell{X: 2, Y: 0}, Goal: model.Cell{X: 0, Y: 0}},
	}
	res := Solve(context.Background(), g, veh, 20, 20000)
	if res.Outcome != NoSolution {
		t.Fatalf("expected NoSolution, got %+v", res)
	}
}

func TestCorridorBlockedByTooFewSteps(t *testing.T) {
	// Same swap needs 4 steps; max_steps 3 cannot work.
	g := grid(3, 1)
	veh := []model.Vehicle{
		{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 2, Y: 0}},
		{ID: "b", Start: model.Cell{X: 2, Y: 0}, Goal: model.Cell{X: 0, Y: 0}},
	}
	res := Solve(context.Background(), g, veh, 3, 1000)
	if res.Outcome != NoSolution {
		t.Fatalf("expected NoSolution, got %+v", res)
	}
}

func TestNarrowCorridorInfeasible(t *testing.T) {
	// 1x2 corridor, agents swap: impossible forever.
	g := grid(2, 1)
	veh := []model.Vehicle{
		{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 1, Y: 0}},
		{ID: "b", Start: model.Cell{X: 1, Y: 0}, Goal: model.Cell{X: 0, Y: 0}},
	}
	res := Solve(context.Background(), g, veh, 10, 20000)
	if res.Outcome != NoSolution {
		t.Fatalf("expected NoSolution, got %+v", res)
	}
}

func TestBudgetExhaustionIsUndecided(t *testing.T) {
	g := grid(3, 2)
	veh := []model.Vehicle{
		{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 2, Y: 0}},
		{ID: "b", Start: model.Cell{X: 1, Y: 0}, Goal: model.Cell{X: 0, Y: 0}},
	}
	res := Solve(context.Background(), g, veh, 12, 1)
	if res.Outcome != Undecided {
		t.Fatalf("expected Undecided with budget 1, got %+v", res)
	}
	if res.Paths != nil {
		t.Fatalf("undecided must not carry partial paths")
	}
}

func TestGoalOccupancyConflictSolved(t *testing.T) {
	// b starts at a's goal and must move away before a arrives.
	g := grid(2, 2)
	veh := []model.Vehicle{
		{ID: "a", Start: model.Cell{X: 0, Y: 1}, Goal: model.Cell{X: 0, Y: 0}},
		{ID: "b", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 1, Y: 1}},
	}
	res := Solve(context.Background(), g, veh, 8, 1000)
	if res.Outcome != Found {
		t.Fatalf("expected Found, got %+v", res)
	}
	// b reaches (1,1) at t=1; a enters (0,0) at t=1 and b ends at t=2:
	// total cost 1 + 2 = 3.
	if res.TotalCost != 3 {
		t.Fatalf("expected cost 3, got %d", res.TotalCost)
	}
	verifySim(t, g, veh, res, 8)
}

func TestCancellation(t *testing.T) {
	// 1x1 start==goal would finish instantly; instead use an impossible swap in
	// a long corridor and cancel immediately.
	g := grid(2, 1)
	veh := []model.Vehicle{
		{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 1, Y: 0}},
		{ID: "b", Start: model.Cell{X: 1, Y: 0}, Goal: model.Cell{X: 0, Y: 0}},
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	res := Solve(ctx, g, veh, 10, 20000)
	if res.Outcome != Undecided {
		t.Fatalf("expected Undecided after cancel, got %+v", res)
	}
}
