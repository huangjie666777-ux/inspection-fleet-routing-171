// Command server runs the inspection-fleet routing HTTP API.
package main

import (
	"flag"
	"fmt"
	"log"
	"net/http"
	"os"
	"strconv"
	"time"

	"inspection-fleet-routing/internal/httpapi"
)

func main() {
	defaultPort := 8080
	if p := os.Getenv("PORT"); p != "" {
		if v, err := strconv.Atoi(p); err == nil {
			defaultPort = v
		}
	}
	port := flag.Int("port", defaultPort, "TCP port to listen on (env PORT also supported)")
	flag.Parse()

	srv := &http.Server{
		Addr:              fmt.Sprintf(":%d", *port),
		Handler:           httpapi.Router(),
		ReadHeaderTimeout: 10 * time.Second,
	}
	log.Printf("inspection-fleet routing API listening on %s (POST /plan)", srv.Addr)
	if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
		log.Fatal(err)
	}
}
