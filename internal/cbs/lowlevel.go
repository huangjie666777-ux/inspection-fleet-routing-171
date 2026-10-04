// Package cbs implements Conflict-Based Search for multi-vehicle
// collision-free path planning on a rectangular grid.
//
// Time is discrete: all vehicles occupy a cell at t=0, and each transition
// covers one time step. Moves are to the four orthogonal neighbours or a wait.
package cbs

import (
	"container/heap"
	"context"

	"inspection-fleet-routing/internal/model"
)

// vertexConstraint forbids agent from occupying cell at time t.
type vertexConstraint struct {
	agent int
	cell  model.Cell
	t     int
}

// edgeConstraint forbids agent moving from a to b between time t and t+1.
// A wait edge (a==b) is never constrained by CBS conflicts.
type edgeConstraint struct {
	agent int
	a, b  model.Cell
	t     int
}

// constraints is the constraint set accumulated along one CBS branch.
type constraints struct {
	vertex map[vertexConstraint]struct{}
	edge   map[edgeConstraint]struct{}
}

func newConstraints() constraints {
	return constraints{
		vertex: map[vertexConstraint]struct{}{},
		edge:   map[edgeConstraint]struct{}{},
	}
}

func (cs constraints) clone() constraints {
	next := constraints{
		vertex: make(map[vertexConstraint]struct{}, len(cs.vertex)+1),
		edge:   make(map[edgeConstraint]struct{}, len(cs.edge)+1),
	}
	for k := range cs.vertex {
		next.vertex[k] = struct{}{}
	}
	for k := range cs.edge {
		next.edge[k] = struct{}{}
	}
	return next
}

func (cs constraints) forbidsVertex(agent int, c model.Cell, t int) bool {
	_, bad := cs.vertex[vertexConstraint{agent, c, t}]
	return bad
}

func (cs constraints) forbidsEdge(agent int, a, b model.Cell, t int) bool {
	_, bad := cs.edge[edgeConstraint{agent, a, b, t}]
	return bad
}

// state is a position-time node in the low-level A* search.
type state struct {
	cell model.Cell
	t    int
}

func manhattan(a, b model.Cell) int {
	dx := a.X - b.X
	if dx < 0 {
		dx = -dx
	}
	dy := a.Y - b.Y
	if dy < 0 {
		dy = -dy
	}
	return dx + dy
}

var dirs = []model.Cell{{X: 0, Y: 0}, {X: 1, Y: 0}, {X: -1, Y: 0}, {X: 0, Y: 1}, {X: 0, Y: -1}}

// llNode is an A* open-set entry.
type llNode struct {
	state  state
	g      int // elapsed time
	f      int // g + heuristic
	parent *llNode
	index  int
}

type llQueue []*llNode

func (q llQueue) Len() int           { return len(q) }
func (q llQueue) Less(i, j int) bool { return q[i].f < q[j].f || (q[i].f == q[j].f && q[i].g > q[j].g) }
func (q llQueue) Swap(i, j int)      { q[i], q[j] = q[j], q[i]; q[i].index = i; q[j].index = j }
func (q *llQueue) Push(x any)        { n := x.(*llNode); n.index = len(*q); *q = append(*q, n) }
func (q *llQueue) Pop() any {
	old := *q
	n := old[len(old)-1]
	*q = old[:len(old)-1]
	return n
}

// lowLevelPath finds a minimum-arrival-time path for one agent that obeys cs
// and reaches goal within maxSteps steps. The returned path covers times
// 0..arrival inclusive; after arrival the vehicle stays at goal forever.
//
// ok is false if no such path exists.
func lowLevelPath(ctx context.Context, grid model.Grid, agent int, start, goal model.Cell, cs constraints, maxSteps int) (path []model.Cell, arrival int, ok bool) {
	if cs.forbidsVertex(agent, start, 0) {
		return nil, 0, false
	}
	startNode := &llNode{state: state{start, 0}, g: 0, f: manhattan(start, goal)}
	open := &llQueue{startNode}
	heap.Init(open)
	// closed records expanded space-time states. bestAt records the smallest
	// arrival time at each cell; that earlier state dominates a later one only
	// when it can safely wait at the cell until the later time, i.e. no vertex
	// constraint (away from goal) forbids occupying the cell in between. Edge
	// constraints never apply to waits, so they do not affect waiting.
	closed := map[state]struct{}{}
	bestAt := map[model.Cell]int{start: 0}

	checkEvery := 256
	iter := 0
	for open.Len() > 0 {
		iter++
		if iter%checkEvery == 0 {
			select {
			case <-ctx.Done():
				return nil, 0, false
			default:
			}
		}
		n := heap.Pop(open).(*llNode)
		s := n.state
		if _, seen := closed[s]; seen {
			continue
		}
		if best, exists := bestAt[s.cell]; exists && best < s.t && canWaitAt(cs, agent, s.cell, best, s.t, goal) {
			continue
		}
		closed[s] = struct{}{}
		if best, exists := bestAt[s.cell]; !exists || s.t < best {
			bestAt[s.cell] = s.t
		}

		if s.cell == goal && s.t > latestGoalForbidden(cs, agent, goal) {
			return reconstruct(n), s.t, true
		}
		if s.t >= maxSteps {
			continue
		}
		nt := s.t + 1
		for _, d := range dirs {
			nb := model.Cell{X: s.cell.X + d.X, Y: s.cell.Y + d.Y}
			if !grid.Passable(nb) {
				continue
			}
			if cs.forbidsVertex(agent, nb, nt) {
				continue
			}
			if cs.forbidsEdge(agent, s.cell, nb, s.t) {
				continue
			}
			ns := state{nb, nt}
			if _, seen := closed[ns]; seen {
				continue
			}
			if best, exists := bestAt[nb]; exists && best <= nt && canWaitAt(cs, agent, nb, best, nt, goal) {
				continue
			}
			heap.Push(open, &llNode{
				state:  ns,
				g:      nt,
				f:      nt + manhattan(nb, goal),
				parent: n,
			})
		}
	}
	return nil, 0, false
}

// canWaitAt reports whether the agent may stay at c from time from+1 through
// to inclusive (vertex constraints at the endpoints themselves are already
// checked on the states). At the goal the entire future up to the latest
// relevant constraint matters, handled by the goal acceptance test; waiting
// dominance at the goal is therefore disabled.
func canWaitAt(cs constraints, agent int, c model.Cell, from, to int, goal model.Cell) bool {
	if c == goal {
		return false
	}
	for t := from + 1; t <= to; t++ {
		if cs.forbidsVertex(agent, c, t) {
			return false
		}
	}
	return true
}

// latestGoalForbidden returns the largest time of a vertex constraint that
// forbids the agent occupying goal. A plan may only stop at goal after that
// time, since finished vehicles remain there forever.
func latestGoalForbidden(cs constraints, agent int, goal model.Cell) int {
	latest := -1
	for k := range cs.vertex {
		if k.agent == agent && k.cell == goal && k.t > latest {
			latest = k.t
		}
	}
	return latest
}

func reconstruct(n *llNode) []model.Cell {
	path := make([]model.Cell, n.g+1)
	for cur := n; cur != nil; cur = cur.parent {
		path[cur.state.t] = cur.state.cell
	}
	return path
}
