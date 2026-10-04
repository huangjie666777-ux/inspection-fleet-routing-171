package cbs

import (
	"context"
	"testing"

	"inspection-fleet-routing/internal/model"
)

// bruteForceBFS computes the exact optimum sum of first-arrival times up to T
// steps with a joint space-time search, or reports no solution. Supports up to
// four agents; finished agents remain at their goal forever.
func bruteForceBFS(t *testing.T, g model.Grid, veh []model.Vehicle, T int) (bool, int) {
	t.Helper()
	n := len(veh)
	if n > 4 {
		t.Skip("brute force supports up to 4 agents")
	}

	type joint struct {
		pos     [4]model.Cell
		done    uint8
		arrival [4]int
	}
	key := func(j joint) string {
		b := make([]byte, 0, 2*n+n+1)
		for i := 0; i < n; i++ {
			b = append(b, byte(j.pos[i].X), byte(j.pos[i].Y))
		}
		b = append(b, byte(j.done))
		for i := 0; i < n; i++ {
			b = append(b, byte(j.arrival[i]))
		}
		return string(b)
	}
	allDone := uint8(1<<uint(n) - 1)
	var start joint
	for i, v := range veh {
		start.pos[i] = v.Start
		if v.Start == v.Goal {
			start.done |= 1 << uint(i)
		}
	}

	best := -1
	eval := func(j joint) {
		if j.done != allDone {
			return
		}
		c := 0
		for i := 0; i < n; i++ {
			c += j.arrival[i]
		}
		if best == -1 || c < best {
			best = c
		}
	}
	eval(start)

	frontier := []joint{start}
	for step := 0; step < T && len(frontier) > 0; step++ {
		seen := map[string]struct{}{}
		var nxt []joint
		emit := func(j joint) {
			// Reject vertex collisions and head-on swaps (caller provides the
			// previous state via closure variable prev below).
			k := key(j)
			if _, ok := seen[k]; ok {
				return
			}
			seen[k] = struct{}{}
			eval(j)
			if j.done != allDone {
				nxt = append(nxt, j)
			}
		}
		for _, prev := range frontier {
			var dfs func(int, joint)
			dfs = func(agent int, cur joint) {
				if agent == n {
					for i := 0; i < n; i++ {
						for j := i + 1; j < n; j++ {
							if cur.pos[i] == cur.pos[j] {
								return
							}
							if cur.pos[i] == prev.pos[j] && cur.pos[j] == prev.pos[i] {
								return
							}
						}
					}
					emit(cur)
					return
				}
				if prev.done&(1<<uint(agent)) != 0 {
					cur.pos[agent] = prev.pos[agent]
					dfs(agent+1, cur)
					return
				}
				old := prev.pos[agent]
				for _, d := range dirs {
					c := model.Cell{X: old.X + d.X, Y: old.Y + d.Y}
					if !g.Passable(c) {
						continue
					}
					np := cur
					np.pos[agent] = c
					if c == veh[agent].Goal {
						np.done |= 1 << uint(agent)
						np.arrival[agent] = step + 1
					}
					dfs(agent+1, np)
				}
			}
			dfs(0, prev)
		}
		frontier = nxt
	}
	if best < 0 {
		return false, 0
	}
	return true, best
}

// TestCBSMatchesJointBFS compares CBS status/optimum against exact joint
// search over a battery of small instances.
func TestCBSMatchesJointBFS(t *testing.T) {
	type scenario struct {
		w, h int
		obs  []model.Cell
		veh  []model.Vehicle
		T    int
	}
	scenarios := []scenario{
		{2, 2, nil, []model.Vehicle{
			{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 1, Y: 1}},
			{ID: "b", Start: model.Cell{X: 1, Y: 1}, Goal: model.Cell{X: 0, Y: 0}},
		}, 6},
		{3, 2, nil, []model.Vehicle{
			{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 2, Y: 0}},
			{ID: "b", Start: model.Cell{X: 2, Y: 0}, Goal: model.Cell{X: 0, Y: 0}},
		}, 8},
		{3, 2, []model.Cell{{X: 2, Y: 1}}, []model.Vehicle{
			{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 1, Y: 1}},
			{ID: "b", Start: model.Cell{X: 1, Y: 0}, Goal: model.Cell{X: 0, Y: 0}},
		}, 8},
		{2, 2, []model.Cell{{X: 1, Y: 0}}, []model.Vehicle{
			{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 1, Y: 1}},
			{ID: "b", Start: model.Cell{X: 0, Y: 1}, Goal: model.Cell{X: 1, Y: 1}},
		}, 8},
		{3, 1, nil, []model.Vehicle{
			{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 2, Y: 0}},
			{ID: "b", Start: model.Cell{X: 1, Y: 0}, Goal: model.Cell{X: 0, Y: 0}},
		}, 6},
		{2, 3, []model.Cell{{X: 0, Y: 1}}, []model.Vehicle{
			{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 0, Y: 2}},
			{ID: "b", Start: model.Cell{X: 1, Y: 2}, Goal: model.Cell{X: 1, Y: 0}},
		}, 8},
		{2, 2, nil, []model.Vehicle{
			{ID: "a", Start: model.Cell{X: 0, Y: 0}, Goal: model.Cell{X: 0, Y: 0}},
			{ID: "b", Start: model.Cell{X: 1, Y: 0}, Goal: model.Cell{X: 1, Y: 1}},
		}, 6},
		{3, 3, []model.Cell{{X: 1, Y: 1}}, []model.Vehicle{
			{ID: "a", Start: model.Cell{X: 0, Y: 1}, Goal: model.Cell{X: 2, Y: 1}},
			{ID: "b", Start: model.Cell{X: 1, Y: 0}, Goal: model.Cell{X: 1, Y: 2}},
		}, 8},
	}
	for idx, sc := range scenarios {
		g := model.NewGrid(sc.w, sc.h, sc.obs)
		bfOK, bfCost := bruteForceBFS(t, g, sc.veh, sc.T)
		res := Solve(context.Background(), g, sc.veh, sc.T, 20000)
		switch {
		case !bfOK && res.Outcome != NoSolution:
			t.Fatalf("scenario %d: BFS infeasible but CBS=%+v", idx, res)
		case bfOK && res.Outcome != Found:
			t.Fatalf("scenario %d: BFS feasible (%d) but CBS=%+v", idx, bfCost, res)
		case bfOK && res.TotalCost != bfCost:
			t.Fatalf("scenario %d: CBS cost %d != BFS cost %d", idx, res.TotalCost, bfCost)
		}
		if res.Outcome == Found {
			verifySim(t, g, sc.veh, res, sc.T)
		}
	}
}
