package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"math"
	"math/rand"
	"net/http"
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
	BaseSpeed         = 0.125            // "Units" per second at 1x speed. Slower for more realism.
	TurnRate          = 10.0             // Degrees per second. A 90-degree turn now takes 9 seconds.
	AltitudeRate      = 50.0             // Feet per second. A 1000ft change takes 20 seconds.
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
	styleCommandAck        = styleDefault.Background(tcell.ColorYellow).Foreground(tcell.ColorBlack).Bold(true)
	styleSelected          = styleDefault.Background(tcell.ColorDarkSlateGray)

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
	StateGameOverPending // New state for the grace period
)

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
	ID                   rune        `json:"id"`
	Callsign             string      `json:"callsign"`
	X                    float64     `json:"x"`
	Y                    float64     `json:"y"`
	Altitude             float64     `json:"altitude"`
	Heading              float64     `json:"heading"`
	TargetAltitude       float64     `json:"target_altitude"`
	TargetHeading        float64     `json:"target_heading"`
	Status               string      `json:"status"`
	StatusStyle          tcell.Style // Color of the status line (changes with status)
	BaseStyle            tcell.Style // Base color of the airline (for the blip)
	dx, dy               float64     // Movement vector is now float64 (internal)
	isConflicting        bool
	isPredictingConflict bool
	commandAckTime       float64 // Timer for command acknowledgment flash
	commandHistory       []string
	turnRate             float64 // Degrees per second, per-aircraft
	altitudeRate         float64 // Feet per second, per-aircraft
}

// NewAircraft creates a new plane
func NewAircraft(id rune, callsign string, x, y, alt, hdg, targetHdg, targetAlt float64) *Aircraft {
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

	// Add some variability to performance
	// Turn rate: +/- 10% of base
	// Altitude rate: +/- 15% of base
	turnRateVar := (rand.Float64() - 0.5) * 0.2 * TurnRate    // rand between -0.1 and +0.1
	altRateVar := (rand.Float64() - 0.5) * 0.3 * AltitudeRate // rand between -0.15 and +0.15

	return &Aircraft{
		ID:                   id,
		Callsign:             callsign,
		X:                    x,
		Y:                    y,
		Altitude:             alt,
		Heading:              hdg,
		TargetAltitude:       targetAlt,
		TargetHeading:        targetHdg,
		Status:               "CRUISING",
		StatusStyle:          baseStyle, // Cruising style is base style
		BaseStyle:            baseStyle, // Store base style
		dx:                   vec.X,     // dx/dy are now floats
		dy:                   vec.Y,
		isConflicting:        false,
		isPredictingConflict: false,
		turnRate:             TurnRate + turnRateVar,
		altitudeRate:         AltitudeRate + altRateVar,
		commandHistory:       make([]string, 0),
	}
}

// Update ticks the aircraft's logic, takes deltaTime
func (a *Aircraft) Update(deltaTime float64, baseSpeed float64) {
	// 1. Update Position based on heading from *last* tick to create an arc.
	// The movement vector (dx, dy) is based on the heading before it's updated.
	a.X += a.dx * baseSpeed * deltaTime
	a.Y += a.dy * baseSpeed * deltaTime

	// 1. Update Altitude
	altDiff := a.TargetAltitude - a.Altitude
	altStep := a.altitudeRate * deltaTime
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
		turnStep := a.turnRate * deltaTime // Degrees to turn this tick

		// Calculate shortest turn direction (float64)
		diff := a.TargetHeading - a.Heading
		if diff > 180 {
			diff -= 360
		} else if diff < -180 {
			diff += 360
		}

		if math.Abs(diff) < turnStep {
			a.Heading = a.TargetHeading // Snap to target
		} else if diff > 0 {
			// Turn right (clockwise)
			a.Heading += turnStep
		} else {
			// Turn left (counter-clockwise)
			a.Heading -= turnStep
		}
		a.Heading = math.Mod(a.Heading+360, 360) // Normalize to 0-359
	}

	// 3. Update movement vector for the *next* tick based on the *new* heading.
	vec := headingToVector(int(a.Heading)) // Recalculate vector from current heading
	a.dx, a.dy = vec.X, vec.Y              // Update the movement vector

	// Update command ack timer
	if a.commandAckTime > 0 {
		a.commandAckTime -= deltaTime
	}

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
	line := fmt.Sprintf("%c %-7s| Pos:(%4.1f,%4.1f) | Hdg:%3.0f°(Tgt:%3.0f°) | Alt:%5.0f(Tgt:%5.0f) | %s",
		a.ID, a.Callsign, a.X, a.Y, a.Heading, a.TargetHeading, a.Altitude, a.TargetAltitude, status)
	return line, style
}

// addCommandToHistory adds a command string to the aircraft's history, keeping it to a max length.
func (a *Aircraft) addCommandToHistory(cmd string) {
	const maxHistory = 5
	a.commandHistory = append(a.commandHistory, cmd)
	if len(a.commandHistory) > maxHistory {
		a.commandHistory = a.commandHistory[len(a.commandHistory)-maxHistory:]
	}
}

func (a *Aircraft) IsCommandAck() bool {
	return a.commandAckTime > 0
}

// Game holds the entire simulation state
type Game struct {
	state                     GameState
	screen                    tcell.Screen
	aircraftList              []*Aircraft
	gameRunning               bool
	showDebug                 bool
	spawnTimer                float64 // Now float64
	score                     int
	conflicts                 int
	commandInput              string
	messageLog                []string
	commandHistory            []string // For command history
	historyIndex              int      // For command history
	mutex                     sync.Mutex
	airspaceWidth             int
	airspaceHeight            int
	r                         *rand.Rand
	speedMultiplier           float64      // Now float64
	simTicker                 *time.Ticker // For simulation logic
	renderTicker              *time.Ticker // For drawing
	nextAircraftID            rune
	conflictFlash             bool
	simTime                   time.Duration
	renderTime                time.Duration
	predictedConflicts        []*PredictedConflict
	menuSelection             int // 0 for Restart, 1 for Quit
	pauseMenuSelection        int // For the pause menu
	gameOverTimer             float64
	selectedAircraftCallsign  string // For highlighting in status board
	tabCompletionPrefix       string // To remember the original prefix for cycling
	tabCompletionStartedEmpty bool   // To handle cycling when starting with an empty input
	actionChan                chan string
	apiEnabled                bool
	// Settings
	maxAircraftSetting           int
	spawnRateSetting             float64
	baseSpeedSetting             float64
	flowTrafficProportionSetting float64
	simRateHz                    int
	renderRateHz                 int
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
	g.spawnTimer = g.spawnRateSetting * 2
	g.score = 0
	g.conflicts = 0
	g.commandInput = ""
	g.messageLog = make([]string, 5)
	g.commandHistory = []string{}
	g.historyIndex = -1
	g.r = rand.New(rand.NewSource(time.Now().UnixNano()))
	g.speedMultiplier = 1.0 // Reset to 1x on new game
	g.nextAircraftID = 'a'
	g.predictedConflicts = make([]*PredictedConflict, 0)
	g.menuSelection = 0
	g.pauseMenuSelection = 0
	g.gameOverTimer = 0
	g.tabCompletionPrefix = ""
	g.tabCompletionStartedEmpty = false
	g.selectedAircraftCallsign = ""
}

// NewGame initializes the game
func NewGame(s tcell.Screen, apiEnabled bool) *Game {
	g := &Game{
		screen:                    s,
		state:                     StateMenu,
		aircraftList:              []*Aircraft{},
		gameRunning:               true,
		showDebug:                 false,
		spawnTimer:                2.5, // float64 (start spawning sooner)
		messageLog:                make([]string, 5),
		commandHistory:            []string{},
		historyIndex:              -1,
		r:                         rand.New(rand.NewSource(time.Now().UnixNano())),
		speedMultiplier:           1.0, // float64, start at 1x
		nextAircraftID:            'a',
		conflictFlash:             false,
		predictedConflicts:        make([]*PredictedConflict, 0),
		menuSelection:             0,
		pauseMenuSelection:        0,
		gameOverTimer:             0,
		tabCompletionPrefix:       "",
		tabCompletionStartedEmpty: false,
		selectedAircraftCallsign:  "",
		actionChan:                make(chan string, 1),
		apiEnabled:                apiEnabled,
		// Default settings (Normal)
		maxAircraftSetting:           15,
		spawnRateSetting:             1.0,
		baseSpeedSetting:             0.125,
		flowTrafficProportionSetting: 0.7, // 70% of traffic will be in a predictable flow
		simRateHz:                    60,
		renderRateHz:                 30,
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

// isSpawnLocationSafe checks if a new aircraft at (x, y, alt) would conflict with existing aircraft.
func (g *Game) isSpawnLocationSafe(x, y, alt float64) bool {
	for _, plane := range g.aircraftList {
		latDist := math.Sqrt(math.Pow(plane.X-x, 2) + math.Pow(plane.Y-y, 2))
		vertDist := math.Abs(plane.Altitude - alt)

		if latDist < LateralSep && vertDist < VerticalSep {
			return false // Conflict detected
		}
	}
	return true // Location is safe
}

// GenerateAircraft creates a new plane at a random edge
func (g *Game) GenerateAircraft() {
	// Try up to 10 times to find a safe spawn location to prevent infinite loops
	for i := 0; i < 10; i++ {
		if g.tryGenerateAircraft() {
			return // Successfully generated
		}
	}
	// If we failed 10 times, log it and give up for this cycle.
	g.AddMessage("Failed to find safe spawn point.")
}

// tryGenerateAircraft attempts to create a single aircraft. Returns true on success.
func (g *Game) tryGenerateAircraft() bool {
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

	// Check if this spawn point is safe. If not, abort this attempt.
	if !g.isSpawnLocationSafe(x, y, altitude) {
		// This location is not safe, try again.
		return false
	}

	// Give it a new random target heading and altitude to create action
	var targetHeading float64
	// Check if this aircraft should be part of a predictable flow
	if g.r.Float64() < g.flowTrafficProportionSetting {
		// Flow traffic: Set heading to cross the airspace
		targetHeading = float64(heading)
	} else {
		// Random traffic: Give a random new heading
		targetHeading = float64(g.r.Intn(360))
	}

	// The initial heading is always straight in from the edge.
	// The target heading determines its flight plan.
	initialHeading := float64(heading)
	// For flow traffic, we want it to fly straight across, so initial and target are the same.
	// For random traffic, we'll let it turn to its random target heading.

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

	newPlane := NewAircraft(id, callsign, x, y, altitude, initialHeading, targetHeading, newTargetAltitude)

	g.aircraftList = append(g.aircraftList, newPlane)
	g.AddMessage(fmt.Sprintf("New(%c): %s, Hdg %.0f° Alt %.0f, Tgt Hdg %.0f° Tgt Alt %.0f",
		id, callsign, initialHeading, altitude, targetHeading, newTargetAltitude))
	return true
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

// FindAircraftByIdentifier looks up a plane by its callsign or its single-character ID.
func (g *Game) FindAircraftByIdentifier(identifier string) *Aircraft {
	// The input identifier is already uppercased.

	// Try to find by single-character ID first.
	if len(identifier) == 1 {
		idRune := rune(identifier[0])
		// Convert uppercase letter from command to lowercase for matching.
		if idRune >= 'A' && idRune <= 'Z' {
			idRune = idRune - 'A' + 'a'
		}

		for _, plane := range g.aircraftList {
			if plane.ID == idRune {
				return plane
			}
		}
	}

	// If not found by ID, fall back to searching by full callsign.
	return g.FindAircraft(identifier)
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
	case StatePlaying, StateGameOverPending: // Allow simulation to run in both states
		if g.state == StateGameOverPending {
			g.gameOverTimer -= deltaTime
			if g.gameOverTimer <= 0 {
				g.state = StateGameOver // Transition to the final game over screen
			}
		}
	default:
		// Any other state (Paused, Menu, final GameOver), do not update simulation.
		return
	}

	// 1. Spawn new aircraft
	g.spawnTimer -= deltaTime
	if len(g.aircraftList) < g.maxAircraftSetting && g.spawnTimer <= 0 {
		g.GenerateAircraft()
		g.spawnTimer = g.r.Float64()*2.0 + g.spawnRateSetting
	}

	// 2. Update all aircraft
	var planesToKeep []*Aircraft
	for _, plane := range g.aircraftList {
		plane.Update(deltaTime, g.baseSpeedSetting)
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
	// Only trigger this once, when moving from Playing state
	if g.conflicts > GameOverConflict && g.state == StatePlaying {
		g.state = StateGameOverPending
		g.gameOverTimer = 4.0 // 4-second grace period
	}

	// Store sim time for debug
	g.simTime = time.Since(start)
}

// ProcessCommand parses and executes a command string.
// It's now separate from UI input handling so it can be called by the API.
func (g *Game) ProcessCommand(command string) {
	command = strings.TrimSpace(command)

	if command == "" {
		return
	}

	cmdUpper := strings.ToUpper(command)
	parts := strings.Split(cmdUpper, " ")

	// Handle single-word commands first
	if len(parts) == 1 {
		switch cmdUpper {
		case "ALTCHK":
			altGroups := make(map[int][]string)
			// Group by truncating to the nearest 1000ft level to catch conflicts.
			for _, plane := range g.aircraftList {
				roundedAlt := int(plane.Altitude/1000) * 1000
				altGroups[roundedAlt] = append(altGroups[roundedAlt], plane.Callsign)
			}

			found := false
			for alt, callsigns := range altGroups {
				if len(callsigns) > 1 {
					g.AddMessage(fmt.Sprintf("ALT %d: %s", alt, strings.Join(callsigns, ", ")))
					found = true
				}
			}
			if !found {
				g.AddMessage("No aircraft sharing altitudes.")
			}
			return

		case "HDGCHK":
			hdgGroups := make(map[int][]string)
			// Group by rounding to the nearest 10 degrees.
			for _, plane := range g.aircraftList {
				// Round to nearest 10, e.g., 273 -> 270, 278 -> 280
				roundedHdg := int(math.Round(plane.Heading/10.0)) * 10
				// Handle the 355-359 range rounding to 360
				if roundedHdg == 360 {
					roundedHdg = 0
				}
				hdgGroups[roundedHdg] = append(hdgGroups[roundedHdg], plane.Callsign)
			}

			found := false
			for hdg, callsigns := range hdgGroups {
				if len(callsigns) > 1 {
					g.AddMessage(fmt.Sprintf("HDG %d°: %s", hdg, strings.Join(callsigns, ", ")))
					found = true
				}
			}
			if !found {
				g.AddMessage("No aircraft on similar headings.")
			}
			return

		case "HELP":
			g.AddMessage("--- HELP ---")
			g.AddMessage("Commands:")
			g.AddMessage("  [CS] A [ALT]     - Set target altitude (e.g., DAL123 A 25000)")
			g.AddMessage("  [CS] H [HDG]     - Set target heading (e.g., DAL123 H 270)")
			g.AddMessage("  HO [CS]          - Handoff aircraft, scoring points")
			g.AddMessage("  ALTCHK           - Check for aircraft at same altitude levels")
			g.AddMessage("  HDGCHK           - Check for aircraft on similar headings")
			g.AddMessage("Controls:")
			g.AddMessage("  ESC                      - Pause/Resume simulation")
			g.AddMessage("  TAB                      - Autocomplete/Cycle callsigns and commands")
			g.AddMessage("  Arrow Up/Down            - Cycle through command history")
			g.AddMessage("  + / -                    - Adjust simulation speed")
			g.AddMessage("  Ctrl+D                   - Toggle debug info")
			g.AddMessage("  Ctrl+C                   - Quit the game")
			g.AddMessage("Goal: Guide aircraft out of your airspace (borders) without conflicts.")
			g.AddMessage("Conflicts: Occur when two aircraft are too close laterally AND vertically.")
			g.AddMessage("Predicted Conflicts: Show potential future conflicts based on current trajectories.")
			g.AddMessage("--- END HELP ---")
			return

		case "EXIT":
			g.gameRunning = false
			return

		}
	}

	// Handle 2-part commands like "HO [CALLSIGN]"
	if len(parts) == 2 {
		action, callsign := parts[0], parts[1]
		if action == "HO" {
			plane := g.FindAircraftByIdentifier(callsign)
			if plane == nil {
				g.AddMessage(fmt.Sprintf("Flight %s not found for handoff.", parts[1]))
				return
			}

			// Remove the aircraft from the list
			var newAircraftList []*Aircraft
			for _, p := range g.aircraftList {
				if p.Callsign != plane.Callsign {
					newAircraftList = append(newAircraftList, p)
				}
			}
			g.aircraftList = newAircraftList

			g.score += 10 // Same score as a successful exit
			g.AddMessage(fmt.Sprintf("%s handed off. Score +10", callsign))
			return
		}
	}

	if len(parts) != 3 {
		g.AddMessage("Invalid command. Use HELP for a list of commands.")
		return
	}

	identifier, action, valueStr := parts[0], parts[1], parts[2]
	plane := g.FindAircraftByIdentifier(identifier)
	if plane == nil {
		g.AddMessage(fmt.Sprintf("Flight %s not found.", identifier))
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
		plane.commandAckTime = 2.0 // Flash for 2 seconds
		plane.addCommandToHistory(fmt.Sprintf("ALT %.0f", value))
		g.AddMessage(fmt.Sprintf("%s, new altitude %.0f", plane.Callsign, value))
	case "H": // Heading
		hdg := value
		if hdg < 0 || hdg > 359 {
			g.AddMessage(fmt.Sprintf("Hdg must be 0-359"))
			return
		}
		plane.TargetHeading = hdg
		plane.commandAckTime = 2.0 // Flash for 2 seconds
		plane.addCommandToHistory(fmt.Sprintf("HDG %.0f°", hdg))
		g.AddMessage(fmt.Sprintf("%s, new heading %.0f°", plane.Callsign, hdg))
	default:
		g.AddMessage("Invalid action. Use 'A' (Altitude) or 'H' (Heading).")
	}
}

// handleUICommand is called when the user presses Enter in the UI.
func (g *Game) handleUICommand() {
	g.mutex.Lock()
	defer g.mutex.Unlock()

	command := g.commandInput
	// Add to command history if it's not empty and not a duplicate of the last one
	if command != "" && (len(g.commandHistory) == 0 || g.commandHistory[len(g.commandHistory)-1] != command) {
		g.commandHistory = append(g.commandHistory, command)
	}
	g.tabCompletionPrefix = ""             // Clear tab prefix on command entry
	g.tabCompletionStartedEmpty = false    // Clear tab state on command entry
	g.selectedAircraftCallsign = ""        // Clear selection on command entry
	g.historyIndex = len(g.commandHistory) // Reset history index
	g.commandInput = ""                    // Clear input buffer
	g.ProcessCommand(command)
}

// handlePauseMenu processes input from the pause menu.
func (g *Game) handlePauseMenu() {
	switch g.pauseMenuSelection {
	case 0: // Resume
		g.state = StatePlaying
		g.AddMessage("Simulation RESUMED.")
	case 1: // Restart
		g.state = StateMenu
		g.menuSelection = 0 // Reset main menu selection
	case 2: // Quit
		g.gameRunning = false
	}
	g.tabCompletionStartedEmpty = false
	g.tabCompletionPrefix = ""
	g.selectedAircraftCallsign = ""
}

// --- Main Loops (Input, Update, Render) ---

// Run is the main game loop
func (g *Game) Run() {
	simRate := time.Second / time.Duration(g.simRateHz)
	renderRate := time.Second / time.Duration(g.renderRateHz)

	// Start the input handler in a goroutine
	go g.HandleInput()

	// Start the game logic ticker in a goroutine
	g.simTicker = time.NewTicker(simRate)
	g.renderTicker = time.NewTicker(renderRate)

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
				numSettings := 4              // maxAircraft, spawnRate, baseSpeed, flowTraffic
				numOptions := numSettings + 2 // settings + Start + Quit
				switch ev.Key() {
				case tcell.KeyUp:
					g.menuSelection = (g.menuSelection - 1 + numOptions) % numOptions
				case tcell.KeyDown:
					g.menuSelection = (g.menuSelection + 1) % numOptions
				case tcell.KeyLeft:
					switch g.menuSelection {
					case 0: // Max Aircraft
						if g.maxAircraftSetting > 1 {
							g.maxAircraftSetting--
						}
					case 1: // Spawn Rate
						g.spawnRateSetting = math.Max(0.1, g.spawnRateSetting-0.1)
					case 2: // Base Speed
						g.baseSpeedSetting = math.Max(0.01, g.baseSpeedSetting-0.01)
					case 3: // Flow Traffic Proportion
						g.flowTrafficProportionSetting = math.Max(0.0, g.flowTrafficProportionSetting-0.1)
					}
				case tcell.KeyRight:
					switch g.menuSelection {
					case 0: // Max Aircraft
						g.maxAircraftSetting++
					case 1: // Spawn Rate
						g.spawnRateSetting += 0.1
					case 2: // Base Speed
						g.baseSpeedSetting += 0.01
					case 3: // Flow Traffic Proportion
						g.flowTrafficProportionSetting = math.Min(1.0, g.flowTrafficProportionSetting+0.1)
					}
				case tcell.KeyEnter:
					if g.menuSelection == numSettings { // Start Game
						g.state = StatePlaying
						g.Reset()
					} else if g.menuSelection == numSettings+1 { // Quit
						g.gameRunning = false
					}
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

			case StateGameOverPending:
				// Disable all input except for quitting during the grace period
				// Ctrl+C and Ctrl+D are handled globally above.
				continue

			case StatePlaying, StatePaused:
				if ev.Key() == tcell.KeyEscape {
					if g.state == StatePlaying {
						g.pauseMenuSelection = 0 // Default to Resume
						g.state = StatePaused
					} else {
						// In pause menu, ESC resumes
						g.state = StatePlaying
					}
					g.Render()
					continue
				}

				if g.state == StatePaused {
					switch ev.Key() {
					case tcell.KeyUp:
						g.pauseMenuSelection = (g.pauseMenuSelection - 1 + 3) % 3
					case tcell.KeyDown:
						g.pauseMenuSelection = (g.pauseMenuSelection + 1) % 3
					case tcell.KeyEnter:
						g.handlePauseMenu()
					}
					g.Render()
					continue
				}

				// Update selection based on current input
				g.mutex.Lock()
				callsignPart := strings.Split(g.commandInput, " ")[0]
				g.selectedAircraftCallsign = callsignPart
				g.mutex.Unlock()

				// --- Command History and Completion ---
				if ev.Key() == tcell.KeyUp {
					g.mutex.Lock()
					if len(g.commandHistory) > 0 {
						if g.historyIndex > 0 {
							g.historyIndex--
						}
						g.commandInput = g.commandHistory[g.historyIndex]
						g.tabCompletionPrefix = ""          // Clear tab prefix
						g.tabCompletionStartedEmpty = false // Clear tab state
						g.selectedAircraftCallsign = ""     // Clear selection when using history
					}
					g.mutex.Unlock()
					continue
				}
				if ev.Key() == tcell.KeyDown {
					g.mutex.Lock()
					if g.historyIndex != -1 && g.historyIndex < len(g.commandHistory)-1 {
						g.historyIndex++
						g.commandInput = g.commandHistory[g.historyIndex]
					} else if g.historyIndex == len(g.commandHistory)-1 {
						g.historyIndex = len(g.commandHistory) // Go to a "new" command
						g.commandInput = ""
						g.tabCompletionPrefix = ""          // Clear tab prefix
						g.tabCompletionStartedEmpty = false // Clear tab state
						g.selectedAircraftCallsign = ""     // Clear selection
					}
					g.mutex.Unlock()
					continue
				}
				if ev.Key() == tcell.KeyTab {
					g.mutex.Lock()

					hasSpace := strings.Contains(g.commandInput, " ")
					parts := strings.Fields(g.commandInput)

					// --- Stage 2: Cycle Commands (A, H) ---
					// This happens if there's a space and the callsign is valid.
					if hasSpace && len(parts) > 0 && g.FindAircraft(parts[0]) != nil {
						callsign := parts[0]
						if len(parts) == 1 { // e.g., "DAL123 "
							g.commandInput = callsign + " A "
						} else if parts[1] == "A" {
							g.commandInput = callsign + " H "
						} else if parts[1] == "H" {
							g.commandInput = callsign + " " // Cycle back to just callsign + space
						}
					} else {
						// --- Stage 1: Cycle Callsigns ---
						// This happens if there's no space in the input.

						// If this is the first time pressing tab for this command, store the state.
						if g.tabCompletionPrefix == "" {
							if g.commandInput == "" {
								g.tabCompletionStartedEmpty = true
							}
							g.tabCompletionPrefix = strings.ToUpper(g.commandInput) // This will be "" if input was empty
						}

						var matches []string
						for _, ac := range g.aircraftList {
							if g.tabCompletionPrefix == "" || strings.HasPrefix(ac.Callsign, g.tabCompletionPrefix) {
								matches = append(matches, ac.Callsign)
							}
						}

						if len(matches) > 0 {
							currentIndex := -1
							var currentInputUpper string
							// If we started with an empty input, the "current" input is the one we just completed.
							// Otherwise, it's whatever is in the text box.
							currentInputUpper = strings.ToUpper(g.commandInput)

							// Find where the current input is in the list of matches.
							for i, match := range matches {
								if match == currentInputUpper && g.FindAircraft(currentInputUpper) != nil {
									currentIndex = i
									break
								}
							}

							// Move to the next index, wrapping around.
							nextIndex := (currentIndex + 1) % len(matches)
							g.commandInput = matches[nextIndex]
							g.selectedAircraftCallsign = g.commandInput
						}
					}
					g.mutex.Unlock()
					continue
				}
			}

			if ev.Key() == tcell.KeyEnter {
				g.handleUICommand()
				continue
			}
			if ev.Key() == tcell.KeyBackspace || ev.Key() == tcell.KeyBackspace2 {
				if len(g.commandInput) > 0 {
					g.commandInput = g.commandInput[:len(g.commandInput)-1]
					g.tabCompletionStartedEmpty = false // Clear tab state
					g.tabCompletionPrefix = ""          // Clear tab prefix
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
					g.tabCompletionPrefix = "" // Clear tab prefix when typing a character
					g.tabCompletionStartedEmpty = false
					// Add a space, but only if there isn't one already and input is not empty
					if r == ' ' {
						// Allow space if input is not empty and doesn't already end with a space
						if len(g.commandInput) > 0 && !strings.HasSuffix(g.commandInput, " ") {
							g.commandInput += " "
						}
					} else {
						g.commandInput += string(r)
					}
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
		title := "--- ATC SIMULATOR - SETTINGS ---"
		DrawText(g.screen, w/2-len(title)/2, h/2-5, title, styleHeader)

		// Settings
		settings := []string{
			fmt.Sprintf("Max Aircraft : < %2d >", g.maxAircraftSetting),
			fmt.Sprintf("Spawn Rate   : < %4.2f >", g.spawnRateSetting),
			fmt.Sprintf("Base Speed   : < %5.3f >", g.baseSpeedSetting),
			fmt.Sprintf("Flow Traffic : < %4.2f >", g.flowTrafficProportionSetting),
		}
		for i, s := range settings {
			style := styleDefault
			if i == g.menuSelection {
				style = styleInput
			}
			DrawText(g.screen, w/2-len(s)/2, h/2-2+i, s, style)
		}

		// Options
		options := []string{"Start Game", "Quit"}
		for i, o := range options {
			style := styleDefault
			msg := fmt.Sprintf("  %s  ", o)
			if i+len(settings) == g.menuSelection {
				style = styleInput
				msg = fmt.Sprintf("> %s <", o)
			}
			DrawText(g.screen, w/2-len(msg)/2, h/2+2+i, msg, style)
		}
		DrawText(g.screen, w/2-len("Use Arrows to change values, Enter to select")/2, h/2+5, "Use Arrows to change values, Enter to select", styleLog)

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

		restartMsg := " RESTART "
		quitMsg := "  QUIT  "
		if g.menuSelection == 0 {
			restartMsg = "> RESTART <"
		} else {
			quitMsg = ">  QUIT  <"
		}
		DrawText(g.screen, w/2-len(restartMsg)/2, h/2, restartMsg, restartStyle)
		DrawText(g.screen, w/2-len(quitMsg)/2, h/2+1, quitMsg, quitStyle)

		g.screen.Show()
		return // Don't draw the rest of the game
	}
	// For StatePlaying and StatePaused, we draw the main game screen.

	// 1. Draw Headers
	DrawText(g.screen, 1, 0, "--- ATC SIMULATOR ---", styleHeader)
	header := fmt.Sprintf("SCORE:%-5d|CONFLICTS:%d/%d|AC:%-2d/%-2d", g.score, g.conflicts, GameOverConflict, len(g.aircraftList), g.maxAircraftSetting)
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

	// 2b. Draw projected path for selected aircraft
	if g.selectedAircraftCallsign != "" {
		// Use FindAircraftByIdentifier to allow selection by 'a' or 'DAL123'
		if plane := g.FindAircraftByIdentifier(strings.ToUpper(g.selectedAircraftCallsign)); plane != nil {
			// Draw current trajectory
			projectionTime := 120.0 // 2 minutes
			endX := plane.X + plane.dx*g.baseSpeedSetting*projectionTime
			endY := plane.Y + plane.dy*g.baseSpeedSetting*projectionTime
			DrawLine(g.screen, int(plane.X), int(plane.Y), int(endX), int(endY), '.', styleLog)

			// Check if a new heading command is being typed and draw a preview trajectory
			parts := strings.Fields(g.commandInput)
			if len(parts) == 3 && strings.ToUpper(parts[1]) == "H" {
				// Ensure the command is for the currently selected plane
				if strings.ToUpper(parts[0]) == strings.ToUpper(plane.Callsign) || strings.ToUpper(parts[0]) == string(plane.ID) {
					newHeading, err := strconv.ParseFloat(parts[2], 64)
					if err == nil && newHeading >= 0 && newHeading <= 359 {
						// This is a valid heading command in progress, draw the preview
						newVec := headingToVector(int(newHeading))
						previewEndX := plane.X + newVec.X*g.baseSpeedSetting*projectionTime
						previewEndY := plane.Y + newVec.Y*g.baseSpeedSetting*projectionTime
						DrawLine(g.screen, int(plane.X), int(plane.Y), int(previewEndX), int(previewEndY), '.', styleWarning)
					}
				}
			}
		}
	}

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
		// Order of precedence: Command Ack > Conflict > Predicted Conflict > Base
		if plane.commandAckTime > 0 {
			style = styleCommandAck
		} else if plane.isConflicting {
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
		// Highlight if selected via tab or if the typed identifier matches
		selectedPlane := g.FindAircraftByIdentifier(strings.ToUpper(g.selectedAircraftCallsign))
		if selectedPlane != nil && plane.Callsign == selectedPlane.Callsign {
			style = styleSelected
		} else if plane.IsCommandAck() {
			style = styleCommandAck
		}
		DrawText(g.screen, g.airspaceWidth+3, y, line, style)
		y++
	}

	// Draw command history for selected aircraft
	if g.selectedAircraftCallsign != "" {
		// Use FindAircraftByIdentifier to show history when 'a' is typed
		if plane := g.FindAircraftByIdentifier(strings.ToUpper(g.selectedAircraftCallsign)); plane != nil && len(plane.commandHistory) > 0 {
			y++ // Spacer
			header := fmt.Sprintf("--- CMD HISTORY: %s ---", plane.Callsign)
			DrawText(g.screen, g.airspaceWidth+3, y, header, styleHeader)
			y++
			// Show last 5 commands
			for _, cmd := range plane.commandHistory {
				DrawText(g.screen, g.airspaceWidth+3, y, "  - "+cmd, styleLog)
				y++
			}
		}
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
	DrawText(g.screen, 1, y, "CMDS: [CS] [A/H] [VAL] | HO [CS] | ALTCHK | HDGCHK | 'ESC' to Pause | 'Ctrl+D' for Debug", styleDefault)
	DrawText(g.screen, 1, y+1, "Use 'Ctrl+C' to quit. | Use '+' and '-' to change speed.", styleDefault)

	// 8. Draw Speed
	speedStr := fmt.Sprintf("SPEED: %.1fx", g.speedMultiplier)
	DrawText(g.screen, g.airspaceWidth-len(speedStr)-2, g.airspaceHeight-1, speedStr, styleDefault)

	// 9. Draw PAUSED overlay
	if g.state == StatePaused {
		boxW, boxH := 20, 7
		boxX, boxY := g.airspaceWidth/2-boxW/2, g.airspaceHeight/2-boxH/2
		for y := boxY; y < boxY+boxH; y++ {
			for x := boxX; x < boxX+boxW; x++ {
				g.screen.SetContent(x, y, ' ', nil, stylePaused)
			}
		}
		DrawText(g.screen, boxX+boxW/2-len("PAUSED")/2, boxY+1, "PAUSED", stylePaused)

		options := []string{"Resume", "Restart", "Quit"}
		for i, opt := range options {
			style := stylePaused
			msg := fmt.Sprintf("  %s  ", opt)
			if i == g.pauseMenuSelection {
				style = stylePaused.Reverse(true)
				msg = fmt.Sprintf("> %s <", opt)
			}
			DrawText(g.screen, boxX+boxW/2-len(msg)/2, boxY+3+i, msg, style)
		}
	}

	// Draw GAME OVER IMMINENT overlay
	if g.state == StateGameOverPending {
		msg := " !!! GAME OVER IMMINENT !!! "
		DrawText(g.screen, g.airspaceWidth/2-len(msg)/2, g.airspaceHeight/2, msg, styleConflict)
	}

	// 10. Draw DEBUG overlay
	g.renderTime = time.Since(start) // Store render time
	if g.showDebug {
		fps := 1.0 / g.renderTime.Seconds()
		debugStr1 := fmt.Sprintf("SimTime: %s", g.simTime)
		debugStr2 := fmt.Sprintf("RenderTime: %s", g.renderTime)
		debugStr3 := fmt.Sprintf("FPS: %.1f", fps)
		DrawText(g.screen, g.airspaceWidth-len(debugStr1)-2, g.airspaceHeight-4, debugStr1, styleDebug)
		DrawText(g.screen, g.airspaceWidth-len(debugStr2)-2, g.airspaceHeight-3, debugStr2, styleDebug)
		DrawText(g.screen, g.airspaceWidth-len(debugStr3)-2, g.airspaceHeight-2, debugStr3, styleDebug)
	}

	// 11. Push buffer to screen
	g.screen.Show()
}

// DrawLine uses Bresenham's line algorithm to draw a line between two points
func DrawLine(s tcell.Screen, x1, y1, x2, y2 int, r rune, style tcell.Style) {
	// Use local variables for iteration to avoid modifying the original parameters.
	x, y := x1, y1
	dx := math.Abs(float64(x2 - x1))
	dy := -math.Abs(float64(y2 - y1))
	sx, sy := 1, 1
	if x > x2 {
		sx = -1
	}
	if y > y2 {
		sy = -1
	}
	err := dx + dy

	for {
		// Don't draw over the start and end points
		if (x != x2 || y != y2) && (x != x1 || y != y1) {
			s.SetContent(x, y, r, nil, style)
		}
		if x == x2 && y == y2 {
			break
		}
		e2 := 2 * err
		if e2 >= dy {
			err += dy
			x += sx
		}
		if e2 <= dx {
			err += dx
			y += sy
		}
	}
}

// --- API Server ---

// ApiGameState is the structure for the JSON state representation.
type ApiGameState struct {
	Score      int         `json:"score"`
	Conflicts  int         `json:"conflicts"`
	IsGameOver bool        `json:"is_game_over"`
	Aircraft   []*Aircraft `json:"aircraft"`
}

// CommandRequest is the structure for receiving a command via the API.
type CommandRequest struct {
	Command string `json:"command"`
}

func (g *Game) stateHandler(w http.ResponseWriter, r *http.Request) {
	g.mutex.Lock()
	defer g.mutex.Unlock()

	state := ApiGameState{
		Score:      g.score,
		Conflicts:  g.conflicts,
		IsGameOver: g.state == StateGameOver || g.state == StateGameOverPending,
		Aircraft:   g.aircraftList,
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(state)
}

func (g *Game) commandHandler(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Only POST method is allowed", http.StatusMethodNotAllowed)
		return
	}

	var req CommandRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	g.mutex.Lock()
	g.ProcessCommand(req.Command)
	g.mutex.Unlock()

	w.WriteHeader(http.StatusOK)
	fmt.Fprintf(w, "Command received: %s", req.Command)
}

// GameConfig defines the structure for a settings file.
type GameConfig struct {
	MaxAircraft  int     `json:"max_aircraft"`
	SpawnRate    float64 `json:"spawn_rate"`
	BaseSpeed    float64 `json:"base_speed"`
	FlowTraffic  float64 `json:"flow_traffic_proportion"`
	SimRateHz    int     `json:"sim_rate_hz"`
	RenderRateHz int     `json:"render_rate_hz"`
}

// loadConfig reads a JSON file and returns a GameConfig struct.
func loadConfig(path string) (*GameConfig, error) {
	file, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	decoder := json.NewDecoder(file)
	config := &GameConfig{}
	err = decoder.Decode(config)
	if err != nil {
		return nil, err
	}
	return config, nil
}

// --- Main Function ---
func main() {
	var apiEnabled bool
	var apiPort int
	var configFile string
	flag.BoolVar(&apiEnabled, "api", false, "Enable the HTTP API server on :8080.")
	flag.StringVar(&configFile, "config", "", "Path to a JSON configuration file.")
	flag.IntVar(&apiPort, "port", 8080, "Port for the HTTP API server.")
	flag.Parse()

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

	game := NewGame(s, apiEnabled)
	defer s.Fini()

	if configFile != "" {
		config, err := loadConfig(configFile)
		if err != nil {
			fmt.Fprintf(os.Stderr, "Warning: could not load config file '%s': %v\n", configFile, err)
		} else {
			game.maxAircraftSetting = config.MaxAircraft
			game.spawnRateSetting = config.SpawnRate
			game.baseSpeedSetting = config.BaseSpeed
			game.flowTrafficProportionSetting = config.FlowTraffic
			if config.SimRateHz > 0 {
				game.simRateHz = config.SimRateHz
			}
			if config.RenderRateHz > 0 {
				game.renderRateHz = config.RenderRateHz
			}
			fmt.Printf("Loaded settings from %s\n", configFile)
		}
	}

	if apiEnabled {
		address := fmt.Sprintf(":%d", apiPort)
		http.HandleFunc("/state", game.stateHandler)
		http.HandleFunc("/command", game.commandHandler)
		go func() {
			fmt.Printf("API server starting on %s\n", address)
			http.ListenAndServe(address, nil)
		}()
	}

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
