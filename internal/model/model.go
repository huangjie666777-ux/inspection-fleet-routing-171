package model

// Cell is an integer grid coordinate. Coordinates are zero-based: x in
// [0,width), y in [0,height).
type Cell struct {
	X int `json:"x"`
	Y int `json:"y"`
}

// Vehicle describes one inspection vehicle with a unique id and integer
// start/goal coordinates.
type Vehicle struct {
	ID    string `json:"id"`
	Start Cell   `json:"start"`
	Goal  Cell   `json:"goal"`
}

// PlanRequest is the JSON body accepted by POST /plan.
type PlanRequest struct {
	Width     int       `json:"width"`
	Height    int       `json:"height"`
	Obstacles []Cell    `json:"obstacles"`
	Vehicles  []Vehicle `json:"vehicles"`
	MaxSteps  int       `json:"max_steps"`
	Budget    int       `json:"budget"` // CBS high-level node expansion budget
}

// Status values returned in PlanResponse.Status.
const (
	StatusOptimal    = "optimal"    // optimal-cost plan found and proved
	StatusInfeasible = "infeasible" // search exhausted within max_steps
	StatusUndecided  = "undecided"  // CBS node budget exhausted before proof
	StatusCancelled  = "cancelled"  // request context was cancelled
)

// VehicleResult is the per-vehicle fragment of a successful plan.
type VehicleResult struct {
	ID          string `json:"id"`
	Path        []Cell `json:"path"`         // positions from time 0 to arrival
	ArrivalTime int    `json:"arrival_time"` // first time the goal is reached
	Cost        int    `json:"cost"`         // arrival time (waiting is counted)
}

// PlanResponse is the JSON body returned by POST /plan.
type PlanResponse struct {
	Status          string          `json:"status"`
	TotalCost       int             `json:"total_cost,omitempty"`
	Vehicles        []VehicleResult `json:"vehicles,omitempty"`
	ExpandedNodes   int             `json:"expanded_nodes"`
	Budget          int             `json:"budget"`
	RemainingBudget int             `json:"remaining_budget"`
	Error           string          `json:"error,omitempty"`
}

// Grid is the validated rectangular world.
type Grid struct {
	Width   int
	Height  int
	blocked map[Cell]struct{}
}

// NewGrid builds a grid and marks blocked cells. Duplicate obstacle cells are
// collapsed.
func NewGrid(width, height int, obstacles []Cell) Grid {
	g := Grid{Width: width, Height: height, blocked: make(map[Cell]struct{}, len(obstacles))}
	for _, o := range obstacles {
		g.blocked[o] = struct{}{}
	}
	return g
}

// Inside reports whether c is inside the rectangle.
func (g Grid) Inside(c Cell) bool {
	return c.X >= 0 && c.X < g.Width && c.Y >= 0 && c.Y < g.Height
}

// Passable reports whether c is inside and free of obstacles.
func (g Grid) Passable(c Cell) bool {
	if !g.Inside(c) {
		return false
	}
	_, bad := g.blocked[c]
	return !bad
}
