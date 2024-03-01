#!/bin/bash

# Check if an argument was provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <python_file_to_run.py>"
    exit 1
fi

# Run the specified Python file
python "$1"
