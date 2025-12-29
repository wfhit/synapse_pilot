#!/usr/bin/env python3
"""
Script to properly register all submodules from .gitmodules into git index
"""

import subprocess
import re
import os

def parse_gitmodules():
    """Parse .gitmodules file and return list of submodules"""
    submodules = []
    current = {}
    
    with open('.gitmodules', 'r') as f:
        for line in f:
            line = line.strip()
            
            if line.startswith('[submodule'):
                if current:
                    submodules.append(current)
                current = {}
            elif '=' in line:
                key, value = line.split('=', 1)
                key = key.strip()
                value = value.strip()
                current[key] = value
        
        if current:
            submodules.append(current)
    
    return submodules

def get_registered_submodules():
    """Get list of submodules already in git index"""
    result = subprocess.run(
        ['git', 'ls-files', '-s'],
        capture_output=True,
        text=True
    )
    
    registered = []
    for line in result.stdout.splitlines():
        if line.startswith('160000'):
            parts = line.split()
            if len(parts) >= 4:
                registered.append(parts[3])
    
    return set(registered)

def main():
    os.chdir('/home/frank/synapse_pilot_ws/SynapsePilot')
    
    print("Parsing .gitmodules...")
    submodules = parse_gitmodules()
    print(f"Found {len(submodules)} submodules in .gitmodules")
    
    print("\nChecking registered submodules...")
    registered = get_registered_submodules()
    print(f"Found {len(registered)} submodules already registered")
    
    print("\nAdding missing submodules...")
    for submod in submodules:
        path = submod.get('path')
        url = submod.get('url')
        branch = submod.get('branch', 'main')
        
        if not path or not url:
            continue
        
        if path in registered:
            print(f"  ✓ {path} (already registered)")
            continue
        
        print(f"  + Adding {path}...")
        
        # Add with force flag, suppress branch warnings
        cmd = ['git', 'submodule', 'add', '--force']
        if branch and branch != 'main':
            cmd.extend(['-b', branch])
        cmd.extend([url, path])
        
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True
        )
        
        if result.returncode != 0 and 'already exists' not in result.stderr:
            print(f"    Warning: {result.stderr.strip()}")
        else:
            print(f"    ✓ Added {path}")
    
    print("\n" + "="*60)
    print("Syncing submodules...")
    subprocess.run(['git', 'submodule', 'sync', '--recursive'])
    
    print("\nInitializing and updating all submodules...")
    print("(This may take several minutes...)")
    result = subprocess.run(
        ['git', 'submodule', 'update', '--init', '--recursive', '--jobs', '4'],
        capture_output=False
    )
    
    print("\n" + "="*60)
    print("Checking final status...")
    result = subprocess.run(
        ['git', 'submodule', 'status'],
        capture_output=True,
        text=True
    )
    
    count = len([l for l in result.stdout.splitlines() if l.strip()])
    print(f"\n✓ Total submodules registered: {count}")
    
    if count < len(submodules):
        print(f"\n⚠ Warning: Expected {len(submodules)} submodules, but only {count} are registered")
    else:
        print("\n✓ All submodules successfully registered!")

if __name__ == '__main__':
    main()
