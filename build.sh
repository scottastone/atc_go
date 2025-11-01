#!/bin/bash

# Create the dist directory if it doesn't exist
mkdir -p dist

# Build the Go application
go build -o dist/atc_go .

echo "Build complete. Executable is in the 'dist' directory."
