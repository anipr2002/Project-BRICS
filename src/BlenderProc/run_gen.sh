# #!/bin/bash

# if [ -z "$1" ]; then
#     echo "Usage: $0 <directory_path>"
#     exit 1
# fi

# dir="$1"

# # Path to your Python script
# PYTHON_SCRIPT="GenerateV2.py"

# # Function to run the Python script
# run_python_script() {
#     blenderproc run "$PYTHON_SCRIPT"
# }

# # Infinite loop to ensure the script runs until successful exit
# while true; do
#     run_python_script
#     exit_code=$?
    
#     # Check if the exit code is 0 (successful execution)
#     if [ $exit_code -eq 69 ]; then
#         echo "Python script executed successfully."
#         break
#     else
#         echo "Restarting to speed up..."
#     fi
# done


#!/bin/bash

# Check if the directory argument is provided
if [ -z "$1" ]; then
    echo "Usage: $0 <directory_path>"
    exit 1
fi

# Assign the first argument to the directory variable
dir="$1"

# Path to your Python script
PYTHON_SCRIPT="GenerateV2.py"

# Function to run the Python script
run_python_script() {
    blenderproc run "$PYTHON_SCRIPT"
}

# Function to count files in the directory
count_files() {
    find "$dir" -maxdepth 1 -type f | wc -l
}

# Main loop to ensure script runs until 3000 files are in the directory
while true; do
    # Count the current number of files
    file_count=$(count_files)
    
    # Check if the file count has reached 3000
    if [ "$file_count" -ge 3000 ]; then
        echo "Directory '$dir' now contains $file_count files. Stopping execution."
        break
    fi

    # Run the Python script
    run_python_script

done