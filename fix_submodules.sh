#!/bin/bash
# Script to properly add all submodules from .gitmodules to git index

set -e

echo "Adding all submodules from .gitmodules to git index..."

# Read .gitmodules and extract submodule paths and URLs
while IFS= read -r line; do
    if [[ $line =~ ^\[submodule ]]; then
        submodule_name=""
    elif [[ $line =~ path[[:space:]]*=[[:space:]]*(.+) ]]; then
        submodule_path="${BASH_REMATCH[1]}"
        submodule_path=$(echo "$submodule_path" | xargs)  # trim whitespace
    elif [[ $line =~ url[[:space:]]*=[[:space:]]*(.+) ]]; then
        submodule_url="${BASH_REMATCH[1]}"
        submodule_url=$(echo "$submodule_url" | xargs)  # trim whitespace
        
        # Check if this submodule is already in the index
        if ! git ls-files -s "$submodule_path" | grep -q "^160000"; then
            echo "Adding submodule: $submodule_path"
            git submodule add --force -b $(git config -f .gitmodules submodule."$submodule_path".branch 2>/dev/null || echo "main") "$submodule_url" "$submodule_path" 2>&1 || echo "  (already exists or error, continuing...)"
        else
            echo "Submodule already registered: $submodule_path"
        fi
    fi
done < .gitmodules

echo ""
echo "Initializing and updating all submodules..."
git submodule update --init --recursive --jobs 4

echo ""
echo "Done! Checking submodule status..."
git submodule status | wc -l
echo "submodules registered."
