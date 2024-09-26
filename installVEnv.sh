#!/bin/bash

ENV_NAME=$1

if [ -z "$ENV_NAME" ]; then
  echo "Please provide the name of the virtual environment. Usage: ./installVenv.sh myEnv01"
  return  # Replace exit 1 to ensure script continues to the end
fi

# Check if the environment exists
if [ -d "$ENV_NAME" ]; then
  echo "Virtual environment '$ENV_NAME' already exists. Activating it..."
else
  echo "Creating virtual environment '$ENV_NAME' with Python 3.10..."
  python3.10 -m venv $ENV_NAME  # Specify Python 3.10 here

  if [ $? -ne 0 ]; then
    echo "Failed to create virtual environment. Make sure Python 3.10 is installed."
    return  # Replace exit 1
  fi

  echo "Virtual environment '$ENV_NAME' created successfully."
fi

# Deactivate any existing conda or virtual environments
conda deactivate &> /dev/null
deactivate &> /dev/null

# Activate the virtual environment
source $ENV_NAME/bin/activate

if [ $? -ne 0 ]; then
  echo "Failed to activate the virtual environment."
  return  # Replace exit 1
fi

echo "Virtual environment '$ENV_NAME' is now active."
echo "Using pip from: $(which pip)"

# Install requirements
if [ -f "requirements.txt" ]; then
  echo "Installing requirements from requirements.txt..."
  pip install --upgrade pip
  pip install -r requirements.txt

  if [ $? -ne 0 ]; then
    echo "Failed to install some or all requirements."
    return  # Replace exit 1
  fi

  echo "All requirements installed successfully."
else
  echo "requirements.txt file not found."
  return  # Replace exit 1
fi

# Stay in the virtual environment after running the script
$SHELL
