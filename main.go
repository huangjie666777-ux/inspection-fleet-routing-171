package main

import (
	"log"
	"net/http"
	"os"
)

func main() {
	port := os.Getenv("PORT")
	if port == "" {
		port = "8080"
	}
	addr := ":" + port
	log.Printf("inspection-fleet-routing listening on %s", addr)
	log.Fatal(http.ListenAndServe(addr, newRouter()))
}
