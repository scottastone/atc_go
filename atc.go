package main

import (
	"fmt"
	"math"
	"math/rand"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/gdamore/tcell/v2"
)

// --- Constants ---
const (
	StatusBoardWidth  = 80               // Width of the right-hand status board
	FooterHeight      = 10               // Height of the log/input/help area
	SimRate           = time.Second / 60 // 60Hz simulation update rate
	RenderRate        = time.Second / 30 // 30Hz render update rate
	BaseSpeed         = 0.25             // "Units" per second at 1x speed. Slower for more realism.
	TurnRate          = 15.0             // Degrees per second. A 90-degree turn now takes 6 seconds.
	AltitudeRate      = 100.0            // Feet per second. A 1000ft change takes 10 seconds.
	AltitudeMin       = 10000.0
	AltitudeMax       = 30000.0
	VerticalSep       = 1000.0 // Minimum vertical separation (float)
	LateralSep        = 3.0    // Minimum horizontal separation (in "units")
	GameOverConflict  = 20     // Number of conflicts to end the game (Increased)
	PredictionHorizon = 1200.0 // 20 minutes into the future to predict conflicts
	// MaxAircraft is now part of Difficulty
)

// --- Styles ---
var (
	styleDefault           = tcell.StyleDefault.Background(tcell.ColorBlack).Foreground(tcell.ColorWhite)
	styleHeader            = styleDefault.Foreground(tcell.ColorAqua).Bold(true)
	styleWarning           = styleDefault.Foreground(tcell.ColorYellow)
	styleClimb             = styleDefault.Foreground(tcell.ColorPlum)
	styleDescend           = styleDefault.Foreground(tcell.ColorSilver)
	styleConflict          = styleDefault.Foreground(tcell.ColorBlack).Background(tcell.ColorRed).Bold(true)
	styleBorder            = styleDefault.Foreground(tcell.ColorDarkGray)
	styleScore             = styleDefault.Foreground(tcell.ColorLime)
	styleLog               = styleDefault.Foreground(tcell.ColorGray)
	styleInput             = styleDefault.Foreground(tcell.ColorWhite).Bold(true)
	stylePaused            = styleDefault.Foreground(tcell.ColorBlack).Background(tcell.ColorWhite)
	styleDebug             = styleDefault.Foreground(tcell.ColorMediumVioletRed)
	styleCPA               = styleDefault.Foreground(tcell.ColorOrangeRed)
	stylePredictedConflict = styleDefault.Foreground(tcell.ColorHotPink).Bold(true)

	// Airline-specific styles
	airlineStyles = []tcell.Style{
		tcell.StyleDefault.Foreground(tcell.ColorLime),
		tcell.StyleDefault.Foreground(tcell.ColorAqua),
		tcell.StyleDefault.Foreground(tcell.ColorYellow),
		tcell.StyleDefault.Foreground(tcell.ColorBlue),
		tcell.StyleDefault.Foreground(tcell.ColorOrange),
		tcell.StyleDefault.Foreground(tcell.ColorWhite),
		tcell.StyleDefault.Foreground(tcell.ColorFuchsia),
		tcell.StyleDefault.Foreground(tcell.ColorSilver),
		tcell.StyleDefault.Foreground(tcell.ColorMediumVioletRed),
		tcell.StyleDefault.Foreground(tcell.ColorSpringGreen),
		tcell.StyleDefault.Foreground(tcell.ColorSkyblue), // Fixed capitalization
	}
)

// --- Game State & Difficulty ---

type GameState int

const (
	StateMenu GameState = iota
	StatePlaying
	StatePaused
	StateGameOver
)

type Difficulty struct {
	Name        string
	MaxAircraft int
	SpawnRate   float64 // Base spawn time, lower is faster
}

var difficulties = []Difficulty{
	{Name: "Easy", MaxAircraft: 8, SpawnRate: 2.0},
	{Name: "Normal", MaxAircraft: 15, SpawnRate: 1.0},
	{Name: "Hard", MaxAircraft: 25, SpawnRate: 0.5},
}

// PredictedConflict stores information about a potential future conflict.
type PredictedConflict struct {
	P1        *Aircraft
	P2        *Aircraft
	TimeToCPA float64
	MinSep    float64
	CPA       Vector // Midpoint of closest approach
}

// --- Structs ---

// Vector is a 2D float64 vector
type Vector struct {
	X, Y float64
}

// headingToVector converts a heading in degrees (0-359) to a normalized movement vector
func headingToVector(hdg int) Vector {
	rad := float64(hdg) * (math.Pi / 180.0)
	return Vector{
		X: math.Sin(rad),
		Y: -math.Cos(rad), // -Y is Up
	}
}

// Aircraft represents a single blip
type Aircraft struct {
	ID                   rune // Unique character for the blip
	Callsign             string
	X, Y                 float64 // Position is now float64
	Altitude             float64 // Altitude is now float64
	Heading              int
	TargetAltitude       float64 // Target altitude is now float64
	TargetHeading        int
	Status               string
	StatusStyle          tcell.Style // Color of the status line (changes with status)
	BaseStyle            tcell.Style // Base color of the airline (for the blip)
	dx, dy               float64     // Movement vector is now float64
	isConflicting        bool
	Speed                float64 // Individual speed multiplier
	isPredictingConflict bool
}

// NewAircraft creates a new plane
func NewAircraft(id rune, callsign string, x, y, alt float64, hdg, targetHdg, targetAlt float64) *Aircraft {
	vec := headingToVector(int(hdg))

	// Find airline index to get consistent color
	code := callsign[:3]
	colorIndex := 0 // Default to first color
	for i, c := range airlineCodes {
		if c == code {
			colorIndex = i
			break
		}
	}
	baseStyle := airlineStyles[colorIndex%len(airlineStyles)] // Use modulo for safety

	return &Aircraft{
		ID:                   id,
		Callsign:             callsign,
		X:                    x,
		Y:                    y,
		Altitude:             alt,
		Heading:              int(hdg),
		TargetAltitude:       targetAlt,
		TargetHeading:        int(targetHdg),
		Status:               "CRUISING",
		StatusStyle:          baseStyle, // Cruising style is base style
		BaseStyle:            baseStyle, // Store base style
		dx:                   vec.X,     // dx/dy are now floats
		dy:                   vec.Y,
		isConflicting:        false,
		isPredictingConflict: false,
	}
}

// Update ticks the aircraft's logic, takes deltaTime
func (a *Aircraft) Update(deltaTime float64) {
	// 1. Update Altitude
	altDiff := a.TargetAltitude - a.Altitude
	altStep := AltitudeRate * deltaTime
	if math.Abs(altDiff) < altStep {
		a.Altitude = a.TargetAltitude
	} else if altDiff > 0 {
		a.Altitude += altStep
		a.Status = "CLIMBING"
		a.StatusStyle = styleClimb
	} else if altDiff < 0 {
		a.Altitude -= altStep
		a.Status = "DESCENDING"
		a.StatusStyle = styleDescend
	}

	if a.Altitude == a.TargetAltitude {
		a.Status = "CRUISING"
		a.StatusStyle = a.BaseStyle // Use base color when cruising
	}

	// 2. Update Heading (now with turn rate)
	if a.Heading != a.TargetHeading {
		turnStep := TurnRate * deltaTime // Degrees to turn this tick

		// Calculate shortest turn direction
		diff := a.TargetHeading - a.Heading
		if diff > 180 {
			diff -= 360
		} else if diff < -180 {
			diff += 360
		}

		if math.Abs(float64(diff)) < turnStep {
			a.Heading = a.TargetHeading // Snap to target
		} else if diff > 0 {
			// Turn right (clockwise)
			a.Heading = (a.Heading + int(turnStep)) % 360
		} else {
			// Turn left (counter-clockwise)
			a.Heading = (a.Heading - int(turnStep) + 360) % 360
		}
	}

	// 3. Update Position based on *current* heading
	vec := headingToVector(a.Heading)
	a.dx = vec.X
	a.dy = vec.Y
	a.X += a.dx * BaseSpeed * deltaTime
	a.Y += a.dy * BaseSpeed * deltaTime

	// Reset conflict status (will be set by game logic if still true)
	a.isConflicting = false
	a.isPredictingConflict = false
}

// GetStatusLine returns the formatted string for the status board
func (a *Aircraft) GetStatusLine() (string, tcell.Style) {
	status := a.Status
	style := a.StatusStyle // This is now BaseStyle, styleClimb, or styleDescend
	if a.isConflicting {
		status = "**CONFLICT**"
		style = styleConflict
	}

	// Format: ID CALLSIGN | Pos: (XX.X, YY.X) | Hdg: XXX° (Tgt: XXX°) | Alt: XXXXX (Tgt: XXXXX) | STATUS
	line := fmt.Sprintf("%c %-7s| Pos:(%4.1f,%4.1f) | Hdg:%3d°(Tgt:%3d°) | Alt:%5.0f(Tgt:%5.0f) | %s",
		a.ID, a.Callsign, a.X, a.Y, a.Heading, a.TargetHeading, a.Altitude, a.TargetAltitude, status)
	return line, style
}

// Game holds the entire simulation state
type Game struct {
	state              GameState
	difficulty         Difficulty
	screen             tcell.Screen
	aircraftList       []*Aircraft
	gameRunning        bool
	showDebug          bool
	spawnTimer         float64 // Now float64
	score              int
	conflicts          int
	commandInput       string
	messageLog         []string
	mutex              sync.Mutex
	airspaceWidth      int
	airspaceHeight     int
	r                  *rand.Rand
	speedMultiplier    float64      // Now float64
	simTicker          *time.Ticker // For simulation logic
	renderTicker       *time.Ticker // For drawing
	nextAircraftID     rune
	conflictFlash      bool
	simTime            time.Duration
	renderTime         time.Duration
	predictedConflicts []*PredictedConflict
	menuSelection      int // 0 for Restart, 1 for Quit
	actionChan         chan string
} // Added closing brace

var airlineCodes = []string{"ACA", "DAL", "UAL", "AAL", "SWA", "BAW", "AFR", "KLM", "JAL", "DLH", "QFA"}
var commonAltitudes = []float64{
	18000, 19000, 20000, 21000, 22000, 23000, 24000, 25000,
	26000, 27000, 28000, 29000, 30000,
}

// --- Game Logic ---

// Reset re-initializes the game state for a new game
func (g *Game) Reset() {
	g.mutex.Lock()
	defer g.mutex.Unlock()

	g.aircraftList = []*Aircraft{}
	g.state = StatePlaying
	g.spawnTimer = g.difficulty.SpawnRate * 2
	g.score = 0
	g.conflicts = 0
	g.commandInput = ""
	g.messageLog = make([]string, 5)
	g.r = rand.New(rand.NewSource(time.Now().UnixNano()))
	g.speedMultiplier = 1.0 // Reset to 1x on new game
	g.nextAircraftID = 'a'
	g.predictedConflicts = make([]*PredictedConflict, 0)
	g.menuSelection = 0
}

// NewGame initializes the game
func NewGame(s tcell.Screen) *Game {
	g := &Game{
		screen:             s,
		state:              StateMenu,
		difficulty:         difficulties[1], // Default to Normal
		aircraftList:       []*Aircraft{},
		gameRunning:        true,
		showDebug:          false,
		spawnTimer:         2.5, // float64 (start spawning sooner)
		messageLog:         make([]string, 5),
		r:                  rand.New(rand.NewSource(time.Now().UnixNano())),
		speedMultiplier:    1.0, // float64, start at 1x
		nextAircraftID:     'a',
		conflictFlash:      false,
		predictedConflicts: make([]*PredictedConflict, 0),
		menuSelection:      0,
		actionChan:         make(chan string, 1),
	}
	g.updateScreenSize() // Set initial size
	return g
}

// updateScreenSize recalculates dimensions based on terminal size
func (g *Game) updateScreenSize() {
	w, h := g.screen.Size()
	g.airspaceWidth = w - StatusBoardWidth
	g.airspaceHeight = h - FooterHeight

	// Enforce minimum size
	if g.airspaceWidth < 20 {
		g.airspaceWidth = 20
	}
	if g.airspaceHeight < 10 {
		g.airspaceHeight = 10
	}
}

// AddMessage adds a status message to the log
func (g *Game) AddMessage(msg string) {
	// Shift log up
	for i := 0; i < len(g.messageLog)-1; i++ {
		g.messageLog[i] = g.messageLog[i+1]
	}
	g.messageLog[len(g.messageLog)-1] = msg
}

// getRandomAltitude picks a random flight level
func (g *Game) getRandomAltitude() float64 {
	return commonAltitudes[g.r.Intn(len(commonAltitudes))]
}

// GenerateAircraft creates a new plane at a random edge
func (g *Game) GenerateAircraft() {
	// 1. Generate random callsign
	airline := airlineCodes[g.r.Intn(len(airlineCodes))]
	flightNum := g.r.Intn(900) + 100 // e.g., 100-999
	callsign := fmt.Sprintf("%s%d", airline, flightNum)

	altitude := g.getRandomAltitude()

	var x, y float64
	var heading int
	edge := g.r.Intn(4)

	switch edge {
	case 0: // North
		x, y = float64(g.r.Intn(g.airspaceWidth-2)+1), 1.0
		heading = 180
	case 1: // South
		x, y = float64(g.r.Intn(g.airspaceWidth-2)+1), float64(g.airspaceHeight-2)
		heading = 0
	case 2: // West
		x, y = 1.0, float64(g.r.Intn(g.airspaceHeight-2)+1)
		heading = 90
	case 3: // East
		x, y = float64(g.airspaceWidth-2), float64(g.r.Intn(g.airspaceHeight-2)+1)
		heading = 270
	}

	// Give it a new random target heading and altitude to create action
	newTargetHeading := g.r.Intn(360)
	newTargetAltitude := g.getRandomAltitude()
	// Ensure new altitude is different
	for newTargetAltitude == altitude {
		newTargetAltitude = g.getRandomAltitude()
	}

	// Assign and increment the unique ID
	id := g.nextAircraftID
	g.nextAircraftID++
	if g.nextAircraftID > 'z' {
		g.nextAircraftID = '0'
	} else if g.nextAircraftID > '9' && g.nextAircraftID < 'a' {
		g.nextAircraftID = 'a' // Skip over symbols between '9' and 'a'
	}

	newPlane := NewAircraft(id, callsign, x, y, altitude, float64(heading), float64(newTargetHeading), newTargetAltitude)

	g.aircraftList = append(g.aircraftList, newPlane)
	g.AddMessage(fmt.Sprintf("New(%c): %s, Hdg %d° Alt %.0f, Tgt Hdg %d° Tgt Alt %.0f",
		id, callsign, heading, altitude, newTargetHeading, newTargetAltitude))
}

// FindAircraft looks up a plane by its callsign
func (g *Game) FindAircraft(callsign string) *Aircraft {
	for _, plane := range g.aircraftList {
		if plane.Callsign == callsign {
			return plane
		}
	}
	return nil
}

// CheckForConflicts iterates all pairs and flags conflicts
func (g *Game) CheckForConflicts() {
	conflictFoundThisTick := false
	for i := 0; i < len(g.aircraftList); i++ {
		for j := i + 1; j < len(g.aircraftList); j++ {
			p1 := g.aircraftList[i]
			p2 := g.aircraftList[j]

			latDist := math.Sqrt(math.Pow(p1.X-p2.X, 2) + math.Pow(p1.Y-p2.Y, 2))
			vertDist := math.Abs(p1.Altitude - p2.Altitude)

			if latDist < LateralSep && vertDist < VerticalSep {
				p1.isConflicting = true
				p2.isConflicting = true
				if !conflictFoundThisTick {
					g.conflicts++ // Only increment once per tick
					conflictFoundThisTick = true
				}
			}
		}
	}
}

// PredictConflict calculates the time and distance of closest approach for two aircraft.
// It returns the time to closest approach in seconds, the minimum separation distance,
// and a boolean indicating if a conflict is predicted.
func (g *Game) PredictConflict(a1, a2 *Aircraft, maxTime float64) (timeToCPA float64, minSeparation float64, isConflict bool) {
	// This is a simplified 2D prediction that does not account for turns.
	// It projects the current velocity vectors forward in time.

	// Aircraft velocity is BaseSpeed units per second.
	v1 := Vector{X: a1.dx * BaseSpeed, Y: a1.dy * BaseSpeed}
	v2 := Vector{X: a2.dx * BaseSpeed, Y: a2.dy * BaseSpeed}
	relVel := Vector{X: v1.X - v2.X, Y: v1.Y - v2.Y}

	// Relative position vector
	relPos := Vector{X: a1.X - a2.X, Y: a1.Y - a2.Y}

	// Time to closest point of approach (CPA)
	dotRelVel := relVel.X*relVel.X + relVel.Y*relVel.Y
	if dotRelVel < 0.001 { // Effectively zero, avoid division by zero
		return -1, 0, false // Parallel or stationary, no change in separation
	}

	dotPosVel := relPos.X*relVel.X + relPos.Y*relVel.Y
	timeToCPA = -dotPosVel / dotRelVel

	if timeToCPA <= 0 || timeToCPA > maxTime {
		return -1, 0, false // CPA is in the past or too far in the future
	}

	// Calculate lateral separation at CPA
	futurePos1 := Vector{X: a1.X + v1.X*timeToCPA, Y: a1.Y + v1.Y*timeToCPA}
	futurePos2 := Vector{X: a2.X + v2.X*timeToCPA, Y: a2.Y + v2.Y*timeToCPA}
	minSeparation = math.Sqrt(math.Pow(futurePos1.X-futurePos2.X, 2) + math.Pow(futurePos1.Y-futurePos2.Y, 2))

	// Predict altitude at CPA, assuming constant climb/descent rate
	alt1_at_cpa := a1.Altitude
	altStep1 := AltitudeRate * timeToCPA
	if a1.TargetAltitude > a1.Altitude {
		alt1_at_cpa = math.Min(a1.TargetAltitude, a1.Altitude+altStep1)
	} else if a1.TargetAltitude < a1.Altitude {
		alt1_at_cpa = math.Max(a1.TargetAltitude, a1.Altitude-altStep1)
	}

	alt2_at_cpa := a2.Altitude
	altStep2 := AltitudeRate * timeToCPA
	if a2.TargetAltitude > a2.Altitude {
		alt2_at_cpa = math.Min(a2.TargetAltitude, a2.Altitude+altStep2)
	} else if a2.TargetAltitude < a2.Altitude {
		alt2_at_cpa = math.Max(a2.TargetAltitude, a2.Altitude-altStep2)
	}

	vertDistAtCPA := math.Abs(alt1_at_cpa - alt2_at_cpa)

	if minSeparation < LateralSep && vertDistAtCPA < VerticalSep {
		return timeToCPA, minSeparation, true
	}

	return -1, 0, false
}

// CheckForPredictedConflicts iterates all pairs and finds all predicted conflicts where planes are getting closer.
func (g *Game) CheckForPredictedConflicts() {
	g.predictedConflicts = make([]*PredictedConflict, 0) // Clear previous predictions

	for i := 0; i < len(g.aircraftList); i++ {
		for j := i + 1; j < len(g.aircraftList); j++ {
			p1 := g.aircraftList[i]
			p2 := g.aircraftList[j]

			timeToCPA, minSep, isConflict := g.PredictConflict(p1, p2, PredictionHorizon)

			if isConflict {
				// Only show if they are projected to get closer than they are now
				currentSep := math.Sqrt(math.Pow(p1.X-p2.X, 2) + math.Pow(p1.Y-p2.Y, 2))
				if minSep < currentSep {
					// Calculate the midpoint of the CPA for drawing
					futurePos1 := Vector{X: p1.X + p1.dx*BaseSpeed*timeToCPA, Y: p1.Y + p1.dy*BaseSpeed*timeToCPA}
					futurePos2 := Vector{X: p2.X + p2.dx*BaseSpeed*timeToCPA, Y: p2.Y + p2.dy*BaseSpeed*timeToCPA}
					cpaMidpoint := Vector{X: (futurePos1.X + futurePos2.X) / 2, Y: (futurePos1.Y + futurePos2.Y) / 2}

					g.predictedConflicts = append(g.predictedConflicts, &PredictedConflict{
						P1: p1, P2: p2, TimeToCPA: timeToCPA, MinSep: minSep, CPA: cpaMidpoint,
					})

				}
			}
		}
	}
}

// UpdateSimulation is the main logic tick
func (g *Game) UpdateSimulation(deltaTime float64) {
	start := time.Now() // Start debug timer
	g.mutex.Lock()
	defer g.mutex.Unlock()

	// Check if paused
	switch g.state {
	case StatePlaying:
		// Continue
	default:
		// Any other state (Paused, Menu, GameOver), do nothing.
		return
	}

	// 1. Spawn new aircraft
	g.spawnTimer -= deltaTime
	if len(g.aircraftList) < g.difficulty.MaxAircraft && g.spawnTimer <= 0 {
		g.GenerateAircraft()
		g.spawnTimer = g.r.Float64()*2.0 + g.difficulty.SpawnRate
	}

	// 2. Update all aircraft
	var planesToKeep []*Aircraft
	for _, plane := range g.aircraftList {
		plane.Update(deltaTime)
		// Check if plane has left the airspace (use int cast for grid boundary)
		if int(plane.X) > 0 && int(plane.X) < g.airspaceWidth-1 && int(plane.Y) > 0 && int(plane.Y) < g.airspaceHeight-1 {
			planesToKeep = append(planesToKeep, plane)
		} else {
			g.score += 10
			g.AddMessage(fmt.Sprintf("%s has left the airspace. Score +10", plane.Callsign))
		}
	}
	g.aircraftList = planesToKeep

	// 3. Check for conflicts
	g.CheckForConflicts()

	// 3.5 Check for predicted conflicts
	g.CheckForPredictedConflicts()

	for _, pc := range g.predictedConflicts {
		pc.P1.isPredictingConflict = true
		pc.P2.isPredictingConflict = true
	}
	// 4. Update conflict flash status
	activeConflict := false
	for _, plane := range g.aircraftList {
		if plane.isConflicting {
			activeConflict = true
			break
		}
	}
	if activeConflict {
		g.conflictFlash = !g.conflictFlash
	} else {
		g.conflictFlash = false
	}

	// 5. Check for game over
	if g.conflicts > GameOverConflict {
		g.state = StateGameOver
	}

	// Store sim time for debug
	g.simTime = time.Since(start)
}

// ProcessCommand parses and executes user input
func (g *Game) ProcessCommand() {
	g.mutex.Lock()
	defer g.mutex.Unlock()

	command := strings.TrimSpace(g.commandInput)
	g.commandInput = "" // Clear input buffer

	if command == "" {
		return
	}

	cmdUpper := strings.ToUpper(command)
	parts := strings.Split(cmdUpper, " ")
	if len(parts) != 3 {
		g.AddMessage("Invalid command. Use: [CALLSIGN] [A/H] [VALUE]")
		return
	}

	callsign, action, valueStr := parts[0], parts[1], parts[2]
	plane := g.FindAircraft(callsign)
	if plane == nil {
		g.AddMessage(fmt.Sprintf("Flight %s not found.", callsign))
		return
	}

	value, err := strconv.ParseFloat(valueStr, 64)
	if err != nil {
		g.AddMessage(fmt.Sprintf("Invalid number: %s", valueStr))
		return
	}

	switch action {
	case "A": // Altitude
		if value < AltitudeMin || value > AltitudeMax {
			g.AddMessage(fmt.Sprintf("Alt must be %.0f-%.0f", AltitudeMin, AltitudeMax))
			return
		}
		plane.TargetAltitude = value
		g.AddMessage(fmt.Sprintf("%s, new altitude %.0f", callsign, value))
	case "H": // Heading
		hdg := int(value)
		if hdg < 0 || hdg > 359 {
			g.AddMessage(fmt.Sprintf("Hdg must be 0-359"))
			return
		}
		plane.TargetHeading = hdg
		g.AddMessage(fmt.Sprintf("%s, new heading %d°", callsign, hdg))
	default:
		g.AddMessage("Invalid action. Use 'A' (Altitude) or 'H' (Heading).")
	}
}

// --- Main Loops (Input, Update, Render) ---

// Run is the main game loop
func (g *Game) Run() {
	// Start the input handler in a goroutine
	go g.HandleInput()

	// Start the game logic ticker in a goroutine
	g.simTicker = time.NewTicker(SimRate)
	g.renderTicker = time.NewTicker(RenderRate)

	for g.gameRunning {
		select {
		case <-g.simTicker.C:
			// Calculate delta time and apply speed multiplier
			deltaTime := SimRate.Seconds() * g.speedMultiplier
			g.UpdateSimulation(deltaTime)
		case <-g.renderTicker.C:
			g.Render()
		}
	}

	// Stop tickers when the main loop exits
	g.simTicker.Stop()
	g.renderTicker.Stop()
}

// HandleInput polls tcell for key events
func (g *Game) HandleInput() {
	for {
		ev := g.screen.PollEvent()
		switch ev := ev.(type) {
		case *tcell.EventResize:
			g.updateScreenSize() // Recalculate dimensions
			g.screen.Sync()
		case *tcell.EventKey:
			if ev.Key() == tcell.KeyCtrlC {
				g.gameRunning = false
				return
			}
			if ev.Key() == tcell.KeyCtrlD {
				g.showDebug = !g.showDebug
				g.Render() // Re-render to show/hide debug info immediately
				continue
			}

			// --- State-based Input Handling ---
			switch g.state {
			case StateMenu:
				switch ev.Key() {
				case tcell.KeyUp:
					g.menuSelection = (g.menuSelection - 1 + len(difficulties)) % len(difficulties)
				case tcell.KeyDown:
					g.menuSelection = (g.menuSelection + 1) % len(difficulties)
				case tcell.KeyEnter:
					g.difficulty = difficulties[g.menuSelection]
					g.Reset() // This also sets state to StatePlaying
				}
				g.Render()
				continue

			case StateGameOver:
				switch ev.Key() {
				case tcell.KeyUp, tcell.KeyDown:
					g.menuSelection = (g.menuSelection + 1) % 2 // Toggle between 0 and 1
				case tcell.KeyEnter:
					if g.menuSelection == 0 { // Restart
						g.state = StateMenu // Go back to difficulty selection
						g.menuSelection = 1 // Default to Normal
					} else { // Quit
						g.gameRunning = false
					}
				}
				g.Render()
				continue

			case StatePlaying, StatePaused:
				if ev.Key() == tcell.KeyEscape {
					if g.state == StatePlaying {
						g.state = StatePaused
						g.AddMessage("Simulation PAUSED. Press 'ESC' again to resume.")
					} else {
						g.state = StatePaused
						g.state = StatePlaying
						g.AddMessage("Simulation RESUMED.")
					}
					g.Render()
					continue
				}
			}

			// --- Normal Game Input ---

			if ev.Key() == tcell.KeyEnter {
				g.ProcessCommand()
				continue
			}
			if ev.Key() == tcell.KeyRune && (ev.Rune() == 'q' || ev.Rune() == 'Q') && len(g.commandInput) == 0 {
				// Allow 'q' to quit if input is empty
				g.gameRunning = false
				g.AddMessage("Quit command received.")
				return
			}
			if ev.Key() == tcell.KeyBackspace || ev.Key() == tcell.KeyBackspace2 {
				if len(g.commandInput) > 0 {
					g.commandInput = g.commandInput[:len(g.commandInput)-1]
				}
				continue
			}
			if ev.Key() == tcell.KeyRune {
				g.mutex.Lock()
				r := ev.Rune()
				switch {
				case r == '+':
					if g.speedMultiplier < 4 {
						g.speedMultiplier += 0.5
						g.AddMessage(fmt.Sprintf("Game speed set to %.1fx", g.speedMultiplier))
					}
				case r == '-':
					if g.speedMultiplier > 0.51 { // Use 0.51 to avoid float precision issues
						g.speedMultiplier -= 0.5
						g.AddMessage(fmt.Sprintf("Game speed set to %.1fx", g.speedMultiplier))
					}
				default:
					g.commandInput += string(r)
				}
				g.mutex.Unlock()
			}
		}
	}
}

// --- Drawing ---

// DrawText is a helper to put strings on the screen
func DrawText(s tcell.Screen, x, y int, text string, style tcell.Style) {
	for i, r := range text {
		s.SetContent(x+i, y, r, nil, style)
	}
}

// Render draws the entire game state to the screen
func (g *Game) Render() { // Fixed: Changed from (g) to (g *Game)
	start := time.Now() // Start debug timer
	g.mutex.Lock()
	defer g.mutex.Unlock()
	g.screen.Clear()

	// 0. Check for minimum screen size
	minW, minH := StatusBoardWidth+23, FooterHeight+12 // 20(min air) + 80(status) + 3(pad), 10(min air) + 10(footer) + 2(pad)
	w, h := g.screen.Size()
	if w < minW || h < minH {
		DrawText(g.screen, 1, 1, "SCREEN TOO SMALL", styleConflict)
		DrawText(g.screen, 1, 2, fmt.Sprintf("Please resize to at least %d x %d", minW, minH), styleWarning)
		g.screen.Show()
		return
	}

	// --- State-based Rendering ---
	switch g.state {
	case StateMenu:
		title := "--- GO ATC SIMULATOR ---"
		DrawText(g.screen, w/2-len(title)/2, h/2-5, title, styleHeader)
		prompt := "Select Difficulty:"
		DrawText(g.screen, w/2-len(prompt)/2, h/2-3, prompt, styleDefault)

		for i, d := range difficulties {
			style := styleDefault
			msg := fmt.Sprintf("  %s  ", d.Name)
			if i == g.menuSelection {
				style = styleInput
				msg = fmt.Sprintf("> %s <", d.Name)
			}
			DrawText(g.screen, w/2-len(msg)/2, h/2-1+i, msg, style)
		}
		DrawText(g.screen, w/2-len("Use Arrow Keys and Enter")/2, h/2+3, "Use Arrow Keys and Enter", styleLog)

		g.screen.Show()
		return

	case StateGameOver:
		msg1 := "GAME OVER"
		DrawText(g.screen, w/2-len(msg1)/2, h/2-3, msg1, styleConflict)

		msg2 := fmt.Sprintf("Final Score: %d", g.score)
		DrawText(g.screen, w/2-len(msg2)/2, h/2-2, msg2, styleScore)

		restartStyle, quitStyle := styleDefault, styleDefault
		if g.menuSelection == 0 {
			restartStyle = styleInput
		} else {
			quitStyle = styleInput
		}

		restartMsg := "> RESTART <"
		quitMsg := "  QUIT   "
		DrawText(g.screen, w/2-len(restartMsg)/2, h/2, restartMsg, restartStyle)
		DrawText(g.screen, w/2-len(quitMsg)/2, h/2+1, quitMsg, quitStyle)

		g.screen.Show()
		return // Don't draw the rest of the game
	}
	// For StatePlaying and StatePaused, we draw the main game screen.

	// 1. Draw Headers
	DrawText(g.screen, 1, 0, "--- GO ATC SIMULATOR (v1.7) ---", styleHeader)
	header := fmt.Sprintf("SCORE:%-5d|CONFLICTS:%d/%d|AC:%-2d/%-2d|DIFF:%s", g.score, g.conflicts, GameOverConflict, len(g.aircraftList), g.difficulty.MaxAircraft, g.difficulty.Name)
	DrawText(g.screen, g.airspaceWidth+3, 0, header, styleInput)

	// 2. Draw Airspace Borders (with conflict flash)
	currentBorderStyle := styleBorder
	if g.conflictFlash {
		currentBorderStyle = styleConflict
	}
	for x := 0; x < g.airspaceWidth; x++ {
		g.screen.SetContent(x, 1, tcell.RuneHLine, nil, currentBorderStyle)
		g.screen.SetContent(x, g.airspaceHeight, tcell.RuneHLine, nil, currentBorderStyle)
	}
	for y := 1; y <= g.airspaceHeight; y++ {
		g.screen.SetContent(0, y, tcell.RuneVLine, nil, currentBorderStyle)
		g.screen.SetContent(g.airspaceWidth-1, y, tcell.RuneVLine, nil, currentBorderStyle)
	}
	// Corners
	g.screen.SetContent(0, 1, tcell.RuneULCorner, nil, currentBorderStyle)
	g.screen.SetContent(g.airspaceWidth-1, 1, tcell.RuneURCorner, nil, currentBorderStyle)
	g.screen.SetContent(0, g.airspaceHeight, tcell.RuneLLCorner, nil, currentBorderStyle)
	g.screen.SetContent(g.airspaceWidth-1, g.airspaceHeight, tcell.RuneLRCorner, nil, currentBorderStyle)

	// 3. Draw Aircraft
	// 3a. Draw CPA lines first, so they are underneath aircraft blips
	for _, conflict := range g.predictedConflicts {
		// Draw line from plane 1 to CPA
		DrawLine(g.screen, int(conflict.P1.X), int(conflict.P1.Y), int(conflict.CPA.X), int(conflict.CPA.Y), '.', styleCPA)
		// Draw line from plane 2 to CPA
		DrawLine(g.screen, int(conflict.P2.X), int(conflict.P2.Y), int(conflict.CPA.X), int(conflict.CPA.Y), '.', styleCPA)
		// Draw an 'X' at the CPA point
		g.screen.SetContent(int(conflict.CPA.X), int(conflict.CPA.Y), 'X', nil, styleCPA.Bold(true))
	}

	for _, plane := range g.aircraftList {
		style := plane.BaseStyle // Use the base airline color for the blip
		if plane.isConflicting {
			style = styleConflict
		} else if plane.isPredictingConflict {
			style = stylePredictedConflict
		}
		// Cast float64 position to int for rendering on the grid
		g.screen.SetContent(int(plane.X), int(plane.Y), plane.ID, nil, style)
	}

	// 4. Draw Status Board (right side)
	y := 2
	if len(g.predictedConflicts) > 0 {
		DrawText(g.screen, g.airspaceWidth+3, y, "--- PREDICTED CONFLICTS ---", stylePredictedConflict)
		y++
		for i, conflict := range g.predictedConflicts {
			msg := fmt.Sprintf("CPA %ds: %s & %s (%.1f units)",
				int(conflict.TimeToCPA), conflict.P1.Callsign, conflict.P2.Callsign, conflict.MinSep)

			if i >= 3 { // Limit to showing 3 predictions
				DrawText(g.screen, g.airspaceWidth+3, y, "...", styleWarning)
				y++
				break
			}
			DrawText(g.screen, g.airspaceWidth+3, y, msg, stylePredictedConflict)
			y++
		}
		y++ // Spacer
	}

	DrawText(g.screen, g.airspaceWidth+3, y, "--- AIRCRAFT STATUS ---", styleHeader)
	y++
	for _, plane := range g.aircraftList {
		line, style := plane.GetStatusLine()
		DrawText(g.screen, g.airspaceWidth+3, y, line, style)
		y++
	}

	// 5. Draw Message Log (below airspace)
	y = g.airspaceHeight + 1
	DrawText(g.screen, 1, y, "--- SYSTEM LOG ---", styleHeader)
	for _, msg := range g.messageLog {
		y++
		DrawText(g.screen, 1, y, msg, styleLog)
	}

	// 6. Draw Command Input
	y += 2
	prompt := "COMMAND> " + g.commandInput
	DrawText(g.screen, 1, y, prompt, styleInput)
	if g.state == StatePlaying {
		g.screen.ShowCursor(1+len(prompt), y) // Show cursor at end of input
	}

	// 7. Draw Instructions
	y += 2
	DrawText(g.screen, 1, y, "CMDS: [CALLSIGN] [A/H] [VALUE] | 'ESC' to Pause | 'Ctrl+D' for Debug", styleDefault)
	DrawText(g.screen, 1, y+1, "Type 'Q' (in empty cmd) or 'CtrlC' to quit. | Use '+' and '-' to change speed.", styleDefault)

	// 8. Draw Speed
	speedStr := fmt.Sprintf("SPEED: %.1fx", g.speedMultiplier)
	DrawText(g.screen, g.airspaceWidth-len(speedStr)-2, g.airspaceHeight-1, speedStr, styleDefault)

	// 9. Draw PAUSED overlay
	if g.state == StatePaused {
		msg := " PAUSED "
		DrawText(g.screen, g.airspaceWidth/2-len(msg)/2, g.airspaceHeight/2, msg, stylePaused)
	}

	// 10. Draw DEBUG overlay
	g.renderTime = time.Since(start) // Store render time
	if g.showDebug {
		debugStr1 := fmt.Sprintf("SimTime: %s", g.simTime)
		debugStr2 := fmt.Sprintf("RenderTime: %s", g.renderTime)
		DrawText(g.screen, g.airspaceWidth-len(debugStr1)-2, g.airspaceHeight-3, debugStr1, styleDebug)
		DrawText(g.screen, g.airspaceWidth-len(debugStr2)-2, g.airspaceHeight-2, debugStr2, styleDebug)
	}

	// 11. Push buffer to screen
	g.screen.Show()
}

// DrawLine uses Bresenham's line algorithm to draw a line between two points
func DrawLine(s tcell.Screen, x1, y1, x2, y2 int, r rune, style tcell.Style) {
	dx := math.Abs(float64(x2 - x1))
	dy := -math.Abs(float64(y2 - y1))
	sx, sy := 1, 1
	if x1 > x2 {
		sx = -1
	}
	if y1 > y2 {
		sy = -1
	}
	err := dx + dy

	for {
		// Don't draw over the start and end points
		if (x1 != x2 || y1 != y2) && (x1 != x1 || y1 != y1) {
			s.SetContent(x1, y1, r, nil, style)
		}
		if x1 == x2 && y1 == y2 {
			break
		}
		e2 := 2 * err
		if e2 >= dy {
			err += dy
			x1 += sx
		}
		if e2 <= dx {
			err += dx
			y1 += sy
		}
	}
}

// --- Main Function ---
func main() {
	s, err := tcell.NewScreen()
	if err != nil {
		fmt.Fprintf(os.Stderr, "Failed to create screen: %v", err)
		os.Exit(1)
	}
	if err := s.Init(); err != nil {
		fmt.Fprintf(os.Stderr, "Failed to initialize screen: %v", err)
		os.Exit(1)
	}
	s.SetStyle(styleDefault)

	game := NewGame(s)
	defer s.Fini()

	// This outer loop allows for restarting the game.
	// The inner `game.Run()` will exit when gameRunning is false.
	for {
		game.Run()
		// The loop will break if game.gameRunning is set to false,
		// which happens on Ctrl-C, 'q', or selecting "Quit" from the game over menu.
		if !game.gameRunning {
			break
		}
	}

	fmt.Println("--- SIMULATION ENDED ---")
	fmt.Printf("Final Score: %d\n", game.score)
	fmt.Printf("Total Conflicts: %d\n", game.conflicts)
}
