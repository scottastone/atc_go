#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# --- Configuration ---
OUTPUT_DIR="dist"
BINARY_NAME="atc_go"
OUTPUT_PATH="$OUTPUT_DIR/$BINARY_NAME"

# --- Colors for output ---
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Starting build process...${NC}"

# 1. Clean up previous build artifacts
echo "Cleaning up old build directory..."
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

# 2. Build and strip the Go application
echo "Building and stripping the executable..."
go build -ldflags="-s -w" -o "$OUTPUT_PATH" .

# 3. Report file size
FILE_SIZE=$(du -h "$OUTPUT_PATH" | cut -f1)

echo -e "\n${GREEN}Build successful!${NC}"
echo "Executable created at: $OUTPUT_PATH"
echo "File size: $FILE_SIZE"
