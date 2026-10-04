package main

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"
)

func post(t *testing.T, body string) (int, map[string]interface{}) {
	t.Helper()
	req := httptest.NewRequest(http.MethodPost, "/plan", bytes.NewBufferString(body))
	rec := httptest.NewRecorder()
	newRouter().ServeHTTP(rec, req)
	var out map[string]interface{}
	_ = json.Unmarshal(rec.Body.Bytes(), &out)
	return rec.Code, out
}

func TestHTTPSuccess(t *testing.T) {
	code, out := post(t, `{
		"width": 3, "height": 2, "max_steps": 16, "node_budget": 20000,
		"vehicles": [
			{"id": "a", "start": {"x": 0, "y": 0}, "goal": {"x": 2, "y": 0}},
			{"id": "b", "start": {"x": 2, "y": 0}, "goal": {"x": 0, "y": 0}}
		]
	}`)
	if code != http.StatusOK || out["status"] != "success" {
		t.Fatalf("code=%d out=%v", code, out)
	}
	if out["total_cost"].(float64) != 6 {
		t.Fatalf("total_cost = %v", out["total_cost"])
	}
	vehicles := out["vehicles"].([]interface{})
	if len(vehicles) != 2 {
		t.Fatalf("vehicles = %v", vehicles)
	}
}

func TestHTTPBadRequest(t *testing.T) {
	code, out := post(t, `{"width": 99, "height": 2, "max_steps": 16, "node_budget": 10,
		"vehicles": [{"id": "a", "start": {"x": 0, "y": 0}, "goal": {"x": 1, "y": 0}}]}`)
	if code != http.StatusBadRequest || out["error"] == nil {
		t.Fatalf("code=%d out=%v", code, out)
	}
}

func TestHTTPMalformedJSON(t *testing.T) {
	code, _ := post(t, `{"width":`)
	if code != http.StatusBadRequest {
		t.Fatalf("code=%d", code)
	}
}

func TestHTTPNoSolution(t *testing.T) {
	code, out := post(t, `{
		"width": 2, "height": 1, "max_steps": 12, "node_budget": 20000,
		"vehicles": [
			{"id": "a", "start": {"x": 0, "y": 0}, "goal": {"x": 1, "y": 0}},
			{"id": "b", "start": {"x": 1, "y": 0}, "goal": {"x": 0, "y": 0}}
		]
	}`)
	if code != http.StatusOK || out["status"] != "no_solution" {
		t.Fatalf("code=%d out=%v", code, out)
	}
	if _, ok := out["vehicles"]; ok {
		t.Fatalf("no_solution must not carry partial routes: %v", out)
	}
}
