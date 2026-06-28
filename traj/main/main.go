package main

import (
	"flag"
	"fmt"
	"os"

	"traj"
)

func main() {
	aeroPath := flag.String("aero", "../openfoam/results/averages.csv", "path to openfoam averages.csv")
	configPath := flag.String("config", "", "rocket JSON config path (default: built-in rocket)")
	outPath := flag.String("out", "out/traj.csv", "trajectory CSV output path")
	step := flag.Float64("h", 0.1, "integration step [s]")
	decimate := flag.Int("decimate", 1, "write every Nth trajectory row")
	metrics := flag.Bool("metrics", false, "print one JSON metrics line and skip CSV/diagnostics (for the optimizer)")
	flag.Parse()

	at, err := traj.LoadAero(*aeroPath)
	if err != nil {
		fmt.Fprintln(os.Stderr, "load aero:", err)
		os.Exit(1)
	}

	var (
		rocket traj.Rocket
		limits traj.Limits
	)
	if *configPath != "" {
		rocket, limits, err = traj.LoadConfig(*configPath)
		if err != nil {
			fmt.Fprintln(os.Stderr, "load config:", err)
			os.Exit(1)
		}
	} else {
		rocket, limits = traj.DefaultConfig()
	}

	rows, diag, err := traj.Simulate(rocket, at, *step)
	if err != nil {
		fmt.Fprintln(os.Stderr, "simulate:", err)
		os.Exit(1)
	}

	if *metrics {
		fmt.Println(traj.MetricsJSON(diag, limits))
		return
	}

	if err := traj.WriteCSV(rows, *outPath, *decimate); err != nil {
		fmt.Fprintln(os.Stderr, "write csv:", err)
		os.Exit(1)
	}
	fmt.Printf("wrote %d rows -> %s\n\n", len(rows), *outPath)

	traj.PrintDiagnostics(diag, limits, at)
}
