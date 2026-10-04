package httpapi

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"

	"inspection-fleet-routing/internal/model"
)

func postJSON(t *testing.T, body string) (int, model.PlanResponse) {
	t.Helper()
	req := httptest.NewRequest(http.MethodPost, "/plan", bytes.NewBufferString(body))
	req.Header.Set("Content-Type", "application/json")
	rec := httptest.NewRecorder()
	Router().ServeHTTP(rec, req)
	var resp model.PlanResponse
	if err := json.Unmarshal(rec.Body.Bytes(), &resp); err != nil {
		t.Fatalf("response is not JSON: %v body=%s", err, rec.Body.String())
	}
	return rec.Code, resp
}

func TestPlanOptimal(t *testing.T) {
	body := `{
	  "width":3,"height":2,
	  "vehicles":[
	    {"id":"a","start":{"x":0,"y":0},"goal":{"x":2,"y":0}},
	    {"id":"b","start":{"x":2,"y":0},"goal":{"x":0,"y":0}}
	  ],
	  "max_steps":12,"budget":1000
	}`
	code, resp := postJSON(t, body)
	if code != http.StatusOK || resp.Status != model.StatusOptimal {
		t.Fatalf("code=%d resp=%+v", code, resp)
	}
	if resp.TotalCost != 6 || len(resp.Vehicles) != 2 {
		t.Fatalf("unexpected payload: %+v", resp)
	}
	if resp.Vehicles[0].ArrivalTime != resp.Vehicles[0].Cost {
		t.Fatal("cost must equal arrival time")
	}
}

func TestPlanInfeasible(t *testing.T) {
	body := `{
	  "width":2,"height":1,
	  "vehicles":[
	    {"id":"a","start":{"x":0,"y":0},"goal":{"x":1,"y":0}},
	    {"id":"b","start":{"x":1,"y":0},"goal":{"x":0,"y":0}}
	  ],
	  "max_steps":10,"budget":20000
	}`
	code, resp := postJSON(t, body)
	if code != http.StatusOK || resp.Status != model.StatusInfeasible {
		t.Fatalf("code=%d resp=%+v", code, resp)
	}
	if resp.Vehicles != nil {
		t.Fatalf("infeasible response must omit routes")
	}
}

func TestPlanUndecided(t *testing.T) {
	body := `{
	  "width":3,"height":2,
	  "vehicles":[
	    {"id":"a","start":{"x":0,"y":0},"goal":{"x":2,"y":0}},
	    {"id":"b","start":{"x":1,"y":0},"goal":{"x":0,"y":0}}
	  ],
	  "max_steps":12,"budget":1
	}`
	code, resp := postJSON(t, body)
	if code != http.StatusOK || resp.Status != model.StatusUndecided {
		t.Fatalf("code=%d resp=%+v", code, resp)
	}
}

func TestPlanValidationError(t *testing.T) {
	body := `{"width":0,"height":1,"vehicles":[],"max_steps":0,"budget":0}`
	code, resp := postJSON(t, body)
	if code != http.StatusBadRequest || resp.Status != "error" || resp.Error == "" {
		t.Fatalf("code=%d resp=%+v", code, resp)
	}
}

func TestPlanMalformedJSON(t *testing.T) {
	body := `{"width":`
	code, resp := postJSON(t, body)
	if code != http.StatusBadRequest || resp.Status != "error" {
		t.Fatalf("code=%d resp=%+v", code, resp)
	}
}

func TestHealth(t *testing.T) {
	req := httptest.NewRequest(http.MethodGet, "/healthz", nil)
	rec := httptest.NewRecorder()
	Router().ServeHTTP(rec, req)
	if rec.Code != http.StatusOK {
		t.Fatalf("health code %d", rec.Code)
	}
}
