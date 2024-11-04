#!/bin/bash

declare -A repos=(
    ["cranium/src/synapse_protobuf"]="https://github.com/NXPHoverGames/synapse_protobuf"
    ["cranium/src/synapse_tinyframe"]="https://github.com/NXPHoverGames/synapse_tinyframe"
    ["cranium/src/synapse_msgs"]="https://github.com/NXPHoverGames/synapse_msgs"
    ["cranium/src/synapse_ros"]="https://github.com/NXPHoverGames/synapse_ros"
    ["cranium/src/dream_world"]="https://github.com/NXPHoverGames/dream_world"
    ["cranium/src/b3rb_simulator"]="https://github.com/NXPHoverGames/b3rb_simulator"
    ["electrode/src/electrode"]="https://github.com/NXPHoverGames/electrode"
    ["ws/cerebri"]="https://github.com/NXPHoverGames/cerebri"
    ["cranium/src/nxp_cup_vision"]="https://github.com/NXPHoverGames/nxp_cup_vision"
)

# Update existing repositories or clone if they don't exist
for repo in "${!repos[@]}"; do
    echo "Processing $repo..."
    repo_path="${repo}" # Derive the full path
    
    # Check if the repository directory exists. If it does not, clone it.
    if [ ! -d "${repo_path}" ]; then
        echo "Repository ${repo_path} does not exist. Cloning..."
        git clone --branch nxp-cup "${repos[$repo]}" "${repo_path}"
        echo "Cloned ${repo} into ${repo_path}."
    fi
    
    # Navigate to the repository directory
    cd "${repo_path}" || { echo "Failed to change directory to ${repo_path}. Does it exist?"; continue; }
    
    # Set the new remote URL
    git remote set-url origin "${repos[$repo]}"
    echo "Remote changed to ${repos[$repo]}"

    # Fetch changes from the new remote
    git fetch origin
    echo "Fetched changes from origin."

    # Checkout the specific branch
    git checkout nxp-cup
    if [ $? -eq 0 ]; then
        echo "Checked out nxp-cup branch."
        # Pull the latest changes from the branch
        git pull origin nxp-cup
        echo "Pulled latest changes from nxp-cup branch."
    else
        echo "Failed to checkout nxp-cup branch. Does it exist?"
    fi
    
    # Return to the original directory
    cd - > /dev/null
done
