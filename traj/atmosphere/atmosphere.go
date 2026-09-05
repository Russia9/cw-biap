package atmosphere

import "math"

type inputParam struct {
	H    float64
	T    float64
	Beta float64
	M    float64
	P    float64
}

var input = []inputParam{
	{-2000, 301.150, -0.0065, 28.964420, 127774},
	{0, 288.150, -0.0065, 28.964420, 101325},
	{11000, 216.650, 0, 28.964420, 22632.0},
	{20000, 216.650, +0.0010, 28.964420, 5474.87},
	{32000, 228.65, +0.0028, 28.964420, 868.014},
	{47000, 270.65, 0, 28.964420, 110.906},
	{51000, 270.65, -0.0028, 28.964420, 66.9384},
	{71000, 214.65, -0.0020, 28.964420, 3.95639},
	{85000, 186.65, 0, 28.964420, 0.363409},
	{94000, 186.650, 0, 28.964420, 0.089925},
}

const HBoundary = 94000.0

const REarth = 6356767.0

// h	[м] 		- Геометрическая высота
// H 	[м']  		- Геопотенциальная высота
// rho	[кг/м^3] 	- Плотность
// p	[Па] 		- Давление
// T	[К]			- Температура
// g	[м/с^2]		- Ускорение свободного падения
// a	[м/с]		- Скорость звука
func Atmosphere(h float64) (H, rho, p, T, g, a float64) {
	const g_c = 9.80665
	const R = 287.05287
	const k = 1.4

	if h > HBoundary { // vacuum
		top := input[len(input)-1]
		H = REarth * h / (REarth + h)
		T = top.T
		g = g_c * (REarth / (REarth + h)) * (REarth / (REarth + h))
		a = math.Sqrt(k * R * T)
		return H, 0, 0, T, g, a
	}
	if h < -2000 {
		h = -2000
	}

	// Geopotential height: H = rh/(r+h)
	H = REarth * h / (REarth + h)

	// Determine the layer
	var layer int = -1
	for i, v := range input {
		if H <= v.H {
			layer = i - 1
			break
		}
	}
	if layer < 0 {
		layer = 0
	}

	// Temperature: T = TStar + Beta(H - HStar)
	T = input[layer].T + input[layer].Beta*(H-input[layer].H)

	// Gravity
	g = g_c * (REarth / (REarth + h)) * (REarth / (REarth + h))

	// Pressure
	if input[layer].Beta == 0 {
		p = math.Pow(10, math.Log10(input[layer].P)-((0.434294*g_c)/(R*T))*(H-input[layer].H))
	} else {
		p = math.Pow(10, math.Log10(input[layer].P)-(g_c/(input[layer].Beta*R))*math.Log10((input[layer].T+input[layer].Beta*(H-input[layer].H))/(input[layer].T)))
	}

	// Density
	rho = p / (R * T)

	// Speed of sound
	a = math.Sqrt(k * R * T)

	return H, rho, p, T, g, a
}
