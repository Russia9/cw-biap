package main

import (
	"fmt"
	"os"
	"traj"
	"traj/aero"
)

// args[1] = rocket.json path
// args[2] = aero averages.csv path
func main() {
	if len(os.Args) != 3 {
		panic("wrong number of arguments")
	}

	// Loading rocket.json
	rocket, err := traj.LoadRocketJSON(os.Args[1])
	if err != nil {
		panic(fmt.Errorf("rocket.json: %w", err))
	}

	// Loading aero table
	aero, err := aero.LoadCSV(os.Args[2])
	if err != nil {
		panic(fmt.Errorf("aero: %w", err))
	}

	// Init model
	_ = traj.InitModel(rocket, aero)
}
