package main

import (
	"container/heap"
	"context"
	"fmt"
	"strings"
)

// cbsNode is one node of the CBS high-level search tree.
type cbsNode struct {
	constraints []*constraintSet // per agent
	paths       [][]Cell         // per agent, step 0..arrival
	arrivals    []int
	cost        int // sum of arrival times
	seq         int
}

type cbsHeap []cbsNode

func (h cbsHeap) Len() int { return len(h) }
func (h cbsHeap) Less(i, j int) bool {
	return h[i].cost < h[j].cost || (h[i].cost == h[j].cost && h[i].seq < h[j].seq)
}
func (h cbsHeap) Swap(i, j int)       { h[i], h[j] = h[j], h[i] }
func (h *cbsHeap) Push(x interface{}) { *h = append(*h, x.(cbsNode)) }
func (h *cbsHeap) Pop() interface{} {
	old := *h
	n := old[len(old)-1]
	*h = old[:len(old)-1]
	return n
}

// posAt returns the cell occupied by path p at time t; after arrival the
// vehicle stays at its goal forever.
func posAt(p []Cell, t int) Cell {
	if t >= len(p) {
		return p[len(p)-1]
	}
	return p[t]
}

// conflict describes the first collision between two paths.
type conflict struct {
	a, b         int
	cell         Cell
	fromA, fromB Cell // positions one step earlier (edge conflicts)
	time         int
	edge         bool
}

// firstConflict scans all vehicle pairs over the full occupancy horizon
// (including post-arrival stays) and returns the earliest conflict.
// Beyond the last arrival every vehicle is parked at its own goal and
// goals are validated distinct, so no conflict can occur later.
func firstConflict(paths [][]Cell) *conflict {
	horizon := 0
	for _, p := range paths {
		if len(p)-1 > horizon {
			horizon = len(p) - 1
		}
	}
	for t := 0; t <= horizon; t++ {
		for i := 0; i < len(paths); i++ {
			for j := i + 1; j < len(paths); j++ {
				pi, pj := posAt(paths[i], t), posAt(paths[j], t)
				if pi == pj {
					return &conflict{a: i, b: j, cell: pi, time: t}
				}
				if t > 0 {
					qi, qj := posAt(paths[i], t-1), posAt(paths[j], t-1)
					if qi == pj && qj == pi && qi != pi {
						return &conflict{a: i, b: j, cell: pi, fromA: qi, fromB: qj, time: t, edge: true}
					}
				}
			}
		}
	}
	return nil
}

// solveCBS runs conflict-based search. It returns the joint plan, or a
// status explaining why none was returned:
//   - StatusNoSolution: the search space within maxSteps was exhausted.
//   - StatusUndecided:  the node budget ran out, or ctx was cancelled.
func solveCBS(ctx context.Context, grid *Grid, vehicles []VehicleSpec, maxSteps, budget int) (paths [][]Cell, arrivals []int, cost int, status string) {
	n := len(vehicles)
	root := cbsNode{constraints: make([]*constraintSet, n), paths: make([][]Cell, n), arrivals: make([]int, n)}
	for i, v := range vehicles {
		root.constraints[i] = newConstraintSet()
		p, arr, ok := lowLevel(grid, v.Start, v.Goal, root.constraints[i], maxSteps)
		if !ok {
			return nil, nil, 0, StatusNoSolution
		}
		root.paths[i] = p
		root.arrivals[i] = arr
		root.cost += arr
	}

	open := &cbsHeap{root}
	heap.Init(open)
	expanded := 0
	seq := 1
	seen := map[string]bool{constraintKey(root.constraints): true}

	for open.Len() > 0 {
		if err := ctx.Err(); err != nil {
			return nil, nil, 0, StatusUndecided
		}
		node := heap.Pop(open).(cbsNode)
		conf := firstConflict(node.paths)
		if conf == nil {
			return node.paths, node.arrivals, node.cost, StatusSuccess
		}
		if expanded >= budget {
			return nil, nil, 0, StatusUndecided
		}
		expanded++

		// Branch: constrain each of the two agents involved.
		branch := []struct {
			agent int
			c     Constraint
		}{}
		if conf.edge {
			branch = append(branch,
				struct {
					agent int
					c     Constraint
				}{conf.a, Constraint{Agent: conf.a, Cell: conf.cell, From: conf.fromA, Time: conf.time - 1, Edge: true}},
				struct {
					agent int
					c     Constraint
				}{conf.b, Constraint{Agent: conf.b, Cell: conf.fromA, From: conf.fromB, Time: conf.time - 1, Edge: true}},
			)
		} else {
			branch = append(branch,
				struct {
					agent int
					c     Constraint
				}{conf.a, Constraint{Agent: conf.a, Cell: conf.cell, Time: conf.time}},
				struct {
					agent int
					c     Constraint
				}{conf.b, Constraint{Agent: conf.b, Cell: conf.cell, Time: conf.time}},
			)
		}

		for _, br := range branch {
			child := cbsNode{
				constraints: make([]*constraintSet, n),
				paths:       make([][]Cell, n),
				arrivals:    make([]int, n),
				seq:         seq,
			}
			seq++
			copy(child.constraints, node.constraints)
			copy(child.paths, node.paths)
			copy(child.arrivals, node.arrivals)
			child.cost = node.cost
			child.constraints[br.agent] = node.constraints[br.agent].clone(br.c)
			v := vehicles[br.agent]
			p, arr, ok := lowLevel(grid, v.Start, v.Goal, child.constraints[br.agent], maxSteps)
			if !ok {
				continue
			}
			child.paths[br.agent] = p
			child.cost += arr - child.arrivals[br.agent]
			child.arrivals[br.agent] = arr
			key := constraintKey(child.constraints)
			if seen[key] {
				continue
			}
			seen[key] = true
			heap.Push(open, child)
		}
	}
	return nil, nil, 0, StatusNoSolution
}

// constraintKey fingerprints the joint constraint state so identical
// high-level nodes reached via different conflict orders are pruned.
func constraintKey(sets []*constraintSet) string {
	var sb strings.Builder
	for i, cs := range sets {
		fmt.Fprintf(&sb, "|%d:", i)
		for cell, ts := range cs.vertex {
			for t := range ts {
				fmt.Fprintf(&sb, "v%d,%d@%d;", cell.X, cell.Y, t)
			}
		}
		for k, ts := range cs.edge {
			for t := range ts {
				fmt.Fprintf(&sb, "e%d,%d>%d,%d@%d;", k[0].X, k[0].Y, k[1].X, k[1].Y, t)
			}
		}
	}
	return sb.String()
}
