package main

import (
	"flag"
	"fmt"
	"os"

	"traj"
)

func main() {
	aeroPath := flag.String("aero", "../openfoam/results/averages.csv", "path to openfoam averages.csv")
	outPath := flag.String("out", "out/traj.csv", "trajectory CSV output path")
	step := flag.Float64("h", 0.1, "integration step [s]")
	decimate := flag.Int("decimate", 10, "write every Nth trajectory row")
	flag.Parse()

	at, err := traj.LoadAero(*aeroPath)
	if err != nil {
		fmt.Fprintln(os.Stderr, "load aero:", err)
		os.Exit(1)
	}

	rocket := traj.DefaultRocket()
	rows, diag, err := traj.Simulate(rocket, at, *step)
	if err != nil {
		fmt.Fprintln(os.Stderr, "simulate:", err)
		os.Exit(1)
	}

	if err := traj.WriteCSV(rows, *outPath, *decimate); err != nil {
		fmt.Fprintln(os.Stderr, "write csv:", err)
		os.Exit(1)
	}
	fmt.Printf("wrote %d rows -> %s\n\n", len(rows), *outPath)

	traj.PrintDiagnostics(diag, at)
}
