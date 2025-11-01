# Go ATC Simulator

Go ATC Simulator is a terminal-based Air Traffic Control (ATC) game written in Go. Your mission is to manage aircraft within your airspace, guiding them safely to their destinations while preventing conflicts.

![gameplay screenshot](https://user-images.githubusercontent.com/12345/some-image-url.png) <!-- TODO: Add a gameplay screenshot -->

## Features

*   **Real-time Simulation:** Aircraft move, turn, climb, and descend in real-time.
*   **Command-based Control:** Issue heading and altitude commands to aircraft.
*   **Conflict Detection:** The system automatically detects and warns you of current and predicted separation violations.
*   **Scoring System:** Earn points for every aircraft that safely exits your airspace.
*   **Adjustable Difficulty & Speed:** Choose from multiple difficulty levels and control the simulation speed.
*   **Dynamic Aircraft Spawning:** New aircraft appear at the edges of your airspace.
*   **Color-coded UI:** The terminal UI uses color to provide at-a-glance information about aircraft status.

## Getting Started

### Prerequisites

*   Go (version 1.21 or newer recommended)
*   A terminal that supports `tcell` (most modern terminals on Linux, macOS, and Windows).

### Build & Run

1.  **Clone the repository (if applicable) or navigate to the project directory.**

2.  **Tidy dependencies:**
    This command will download the necessary libraries (`tcell` and its dependencies).
    ```sh
    go mod tidy
    ```

3.  **Build the executable:**
    ```sh
    go build
    ```

4.  **Run the game:**
    ```sh
    ./atc_go
    ```

#### Command-line Flags

The game supports several command-line flags for advanced configuration:

*   `-api`: Enables a web server for remote control and state observation, useful for reinforcement learning agents. By default, it uses port `8080`.
    *   `GET /state`: Returns a JSON object of the current game state.
    *   `POST /command`: Accepts a JSON object like `{"command": "DAL123 H 270"}` to issue commands.

*   `-config [path]`: Loads game settings from a specified JSON file. This allows you to create and use different scenarios easily.

*   `-port [number]`: Specifies the port for the API server (e.g., `-port 8081`).

**Example Usage:**
```sh
# Run with a custom configuration file
./atc_go -config config.sample.json

# Run with both the API and a config file
./atc_go -api -config my_scenario.json -port 8081
```

A sample configuration file (`config.sample.json`) is included in the repository. It looks like this:
```json
{
  "max_aircraft": 15,
  "spawn_rate": 1.0,
  "base_speed": 0.125
}
```

## How to Play

### The Objective

The goal is to manage all aircraft in your sector. You must ensure that aircraft maintain minimum separation:
*   **Lateral (Horizontal) Separation:** `3.0` units
*   **Vertical Separation:** `1000` feet

You score points for each aircraft that successfully transits your airspace and exits. The game ends if you accumulate too many conflicts (`20`).

### The Screen Layout

The screen is divided into four main areas:

1.  **Airspace (Main Grid):** The central area where aircraft blips (`a`, `b`, etc.) are displayed.
2.  **Status Board (Right Panel):** Shows detailed information for each aircraft, predicted conflicts, score, and game status.
3.  **System Log (Bottom-Left):** Displays messages about game events, such as new aircraft, successful handoffs, and command confirmations.
4.  **Command Area (Bottom-Center):** Where you type your commands.

### Controls & Commands

#### General Controls

| Key         | Action                               |
|-------------|--------------------------------------|
| `ESC`       | Open Pause Menu (Resume, Restart, Quit) |
| `+`         | Increase simulation speed (up to 4x) |
| `-`         | Decrease simulation speed (down to 0.5x) |
| `Ctrl+D`    | Toggle debug information overlay     |
| `Ctrl+C`    | Force quit the game                  |

#### Aircraft Commands

All aircraft commands follow the format: `[CALLSIGN] [ACTION] [VALUE]`

*   **CALLSIGN:** The 3-letter, 3-digit identifier for the aircraft (e.g., `DAL123`, `SWA456`).
*   **ACTION:** `H` for Heading, `A` for Altitude, or `HO` for Handoff.
*   **VALUE:** The numerical value for the new heading or altitude.

**Examples:**
*   `AAL456 H 270` - Orders American 456 to turn to a heading of 270 degrees.
*   `b A 25000` - Orders aircraft `b` to climb/descend to 25,000 feet.
*   `HO UAL123` - Hands off United 123, removing it from the screen and awarding points.

### Understanding the Display

*   **Aircraft Blips:**
    *   **Default Color:** Each airline has a unique color (lime, aqua, yellow, etc.).
    *   **`HotPink` / `Bold`:** The aircraft is part of a *predicted conflict*. You have time to resolve it.
    *   **`Red Background`:** The aircraft is in an *active conflict*, meaning it has violated minimum separation rules with another aircraft. The airspace border will also flash red.

*   **Predicted Conflicts:**
    *   When the system predicts two aircraft will violate separation within the next 20 minutes, a warning appears.
    *   The aircraft blips turn **pink**.
    *   Dotted lines are drawn from each aircraft to the predicted Closest Point of Approach (CPA), marked with an 'X'.
    *   Details are listed at the top of the Status Board, showing time to CPA and the callsigns involved.

*   **Aircraft Status Line:**
    The status board shows a detailed line for each aircraft:
    ```
    c SWA811 | Pos:(60.4,80.0) | Hdg:  0°(Tgt:311°) | Alt:22000(Tgt:18000) | DESCENDING
    ```
    *   `c`: The aircraft's ID on the map.
    *   `SWA811`: The aircraft's callsign.
    *   `Pos`: Current X, Y coordinates.
    *   `Hdg`: Current heading and (Target) heading.
    *   `Alt`: Current altitude and (Target) altitude.
    *   `STATUS`: The aircraft's current action (`CRUISING`, `CLIMBING`, `DESCENDING`, or `**CONFLICT**`). The color of this text also indicates its status.
