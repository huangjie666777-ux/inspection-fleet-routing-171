package cbs

import (
	"container/heap"
	"context"

	"inspection-fleet-routing/internal/model"
)

// Outcome is the high-level search result.
type Outcome int

const (
	Found Outcome = iota
	NoSolution
	Undecided
)

// agentPath is one vehicle's plan: positions for times 0..arrival. Occupancy
// after arrival stays at the goal indefinitely.
type agentPath struct {
	path    []model.Cell
	arrival int
}

func (p agentPath) at(t int) model.Cell {
	if t >= len(p.path) {
		return p.path[p.arrival]
	}
	return p.path[t]
}

// conflictKind identifies which split a found conflict needs.
type conflictKind int

const (
	kindVertex conflictKind = iota
	kindEdge
)

type conflict struct {
	kind       conflictKind
	a, b       int // vehicles
	t          int
	cell       model.Cell // vertex conflict cell
	fromA, toA model.Cell // edge swap cells, from a's direction
}

// ctNode is a constraint-tree node.
type ctNode struct {
	paths []agentPath
	cost  int
	cs    constraints
	depth int
	index int
}

type ctQueue []*ctNode

func (q ctQueue) Len() int { return len(q) }
func (q ctQueue) Less(i, j int) bool {
	if q[i].cost != q[j].cost {
		return q[i].cost < q[j].cost
	}
	return q[i].depth < q[j].depth
}
func (q ctQueue) Swap(i, j int) { q[i], q[j] = q[j], q[i]; q[i].index = i; q[j].index = j }
func (q *ctQueue) Push(x any)   { n := x.(*ctNode); n.index = len(*q); *q = append(*q, n) }
func (q *ctQueue) Pop() any {
	old := *q
	n := old[len(old)-1]
	*q = old[:len(old)-1]
	return n
}

// Result is returned by Solve.
type Result struct {
	Outcome       Outcome
	Paths         [][]model.Cell
	Arrivals      []int
	TotalCost     int
	ExpandedNodes int
}

// Solve runs CBS. budget bounds the number of constraint-tree node
// expansions (the root counts as one). Each call uses independent constraints
// and budget; ctx cancellation stops the search.
func Solve(ctx context.Context, grid model.Grid, vehicles []model.Vehicle, maxSteps, budget int) Result {
	n := len(vehicles)
	base := newConstraints()
	rootPaths := make([]agentPath, n)
	for i, v := range vehicles {
		path, arrival, ok := lowLevelPath(ctx, grid, i, v.Start, v.Goal, base, maxSteps)
		if !ok {
			if ctx.Err() != nil {
				return Result{Outcome: Undecided}
			}
			return Result{Outcome: NoSolution, ExpandedNodes: 0}
		}
		rootPaths[i] = agentPath{path: path, arrival: arrival}
	}

	root := &ctNode{paths: rootPaths, cs: base, cost: sumCost(rootPaths)}
	open := &ctQueue{root}
	heap.Init(open)
	expanded := 0
	for open.Len() > 0 {
		select {
		case <-ctx.Done():
			return Result{Outcome: Undecided, ExpandedNodes: expanded}
		default:
		}
		node := heap.Pop(open).(*ctNode)
		expanded++
		if expanded > budget {
			return Result{Outcome: Undecided, ExpandedNodes: expanded - 1}
		}

		cf := firstConflict(node.paths, maxSteps)
		if cf == nil {
			res := Result{Outcome: Found, TotalCost: node.cost, ExpandedNodes: expanded}
			res.Paths = make([][]model.Cell, n)
			res.Arrivals = make([]int, n)
			for i, p := range node.paths {
				res.Paths[i] = p.path
				res.Arrivals[i] = p.arrival
			}
			return res
		}

		for _, branch := range splitAgents(cf) {
			child := &ctNode{paths: clonePaths(node.paths), cs: node.cs.clone(), depth: node.depth + 1}
			agent := branch
			addConflictConstraint(child.cs, cf, agent)
			v := vehicles[agent]
			path, arrival, ok := lowLevelPath(ctx, grid, agent, v.Start, v.Goal, child.cs, maxSteps)
			if !ok {
				if ctx.Err() != nil {
					return Result{Outcome: Undecided, ExpandedNodes: expanded}
				}
				continue // pruned branch
			}
			child.paths[agent] = agentPath{path: path, arrival: arrival}
			child.cost = sumCost(child.paths)
			heap.Push(open, child)
		}
	}
	return Result{Outcome: NoSolution, ExpandedNodes: expanded}
}

func sumCost(paths []agentPath) int {
	c := 0
	for _, p := range paths {
		c += p.arrival
	}
	return c
}

func clonePaths(paths []agentPath) []agentPath {
	out := make([]agentPath, len(paths))
	copy(out, paths)
	return out
}

// firstConflict scans t=0..maxSteps. Vertex conflicts at t=0 would mean
// duplicate starts and are rejected at validation, but are checked anyway.
func firstConflict(paths []agentPath, maxSteps int) *conflict {
	n := len(paths)
	for t := 0; t <= maxSteps; t++ {
		occupied := make(map[model.Cell]int, n)
		for i := range paths {
			c := paths[i].at(t)
			if j, clash := occupied[c]; clash {
				return &conflict{kind: kindVertex, a: j, b: i, t: t, cell: c}
			}
			occupied[c] = i
		}
		if t < maxSteps {
			for i := 0; i < n; i++ {
				for j := i + 1; j < n; j++ {
					pi0, pi1 := paths[i].at(t), paths[i].at(t+1)
					pj0, pj1 := paths[j].at(t), paths[j].at(t+1)
					if pi0 == pj1 && pi1 == pj0 {
						return &conflict{kind: kindEdge, a: i, b: j, t: t, fromA: pi0, toA: pi1}
					}
				}
			}
		}
	}
	return nil
}

// splitAgents returns the two agents that must independently avoid cf.
func splitAgents(cf *conflict) [2]int {
	return [2]int{cf.a, cf.b}
}

func addConflictConstraint(cs constraints, cf *conflict, agent int) {
	switch cf.kind {
	case kindVertex:
		cs.vertex[vertexConstraint{agent: agent, cell: cf.cell, t: cf.t}] = struct{}{}
	case kindEdge:
		a := pathsCellFor(cf, agent, true)
		b := pathsCellFor(cf, agent, false)
		cs.edge[edgeConstraint{agent: agent, a: a, b: b, t: cf.t}] = struct{}{}
	}
}

// pathsCellFor returns the from/to cells of the forbidden directed edge for the
// constrained agent. cf describes the swap from a's perspective: a moves
// fromA->toA while b moves toA->fromA.
func pathsCellFor(cf *conflict, agent int, from bool) model.Cell {
	if agent == cf.a {
		if from {
			return cf.fromA
		}
		return cf.toA
	}
	if from {
		return cf.toA
	}
	return cf.fromA
}
