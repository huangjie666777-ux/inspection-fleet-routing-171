package httpapi

import (
	"context"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"
)

// TestPlanCancelled runs a search bound by a tiny step limit in a tight
// corridor and cancels the request immediately. The solver must report the
// cancelled status rather than infeasible or a partial plan.
func TestPlanCancelled(t *testing.T) {
	body := strings.NewReader(`{
	  "width":2,"height":1,
	  "vehicles":[
	    {"id":"a","start":{"x":0,"y":0},"goal":{"x":1,"y":0}},
	    {"id":"b","start":{"x":1,"y":0},"goal":{"x":0,"y":0}}
	  ],
	  "max_steps":64,"budget":20000
	}`)
	req := httptest.NewRequest(http.MethodPost, "/plan", body).WithContext(context.Background())
	ctx, cancel := context.WithCancel(req.Context())
	req = req.WithContext(ctx)
	cancel()
	rec := httptest.NewRecorder()
	done := make(chan struct{})
	go func() {
		Router().ServeHTTP(rec, req)
		close(done)
	}()
	select {
	case <-done:
	case <-time.After(5 * time.Second):
		t.Fatal("handler did not stop after cancellation")
	}
	if !strings.Contains(rec.Body.String(), "\"cancelled\"") {
		t.Fatalf("expected cancelled, got %s", rec.Body.String())
	}
}
