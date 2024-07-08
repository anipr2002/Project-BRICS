#!/bin/bash

# Path to your Python script
PYTHON_SCRIPT="Generate.py"

# Function to run the Python script
run_python_script() {
    blenderproc run "$PYTHON_SCRIPT"
}

# Infinite loop to ensure the script runs until successful exit
while true; do
    run_python_script
    exit_code=$?
    
    # Check if the exit code is 0 (successful execution)
    if [ $exit_code -eq 69 ]; then
        echo "Python script executed successfully."
        break
    else
        echo "Python script failed with exit code $exit_code. Restarting..."
    fi
done