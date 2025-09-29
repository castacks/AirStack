#!/usr/bin/env bash

# config.sh - Configuration-related commands for AirStack
# This module provides commands for configuring the AirStack environment

# Helper function for confirmation prompts
function confirm_no {
    read -r -p "${1:-Are you sure? [y/N]} " response
    case "$response" in
        [yY][eE][sS]|[yY]) 
            true
            ;;
        *)
            false
            ;;
    esac
}

# Function to configure Isaac Sim settings
function cmd_config_isaac_sim {
    log_info "Configuring Isaac Sim settings..."
    
    # Generate user.config.json for IsaacSim settings
    local USER_CONFIG_JSON_SOURCE="${PROJECT_ROOT}/simulation/isaac-sim/docker/user_TEMPLATE.config.json"
    local USER_CONFIG_JSON_DESTINATION="${PROJECT_ROOT}/simulation/isaac-sim/docker/user.config.json"
    
    log_info "Generating Default IsaacSim Config ($USER_CONFIG_JSON_DESTINATION)"
    
    if [ -d "$USER_CONFIG_JSON_DESTINATION" ] && [ $(ls -A "$USER_CONFIG_JSON_DESTINATION" | wc -l) == 0 ]; then
        # delete an empty directory with the same name as $USER_CONFIG_JSON_DESTINATION which gets created when
        # docker compose up is run before this script. Doing this will create a directory name user.config.json because
        # it is being mounted as a volume but it doesn't exist yet.
        rm -rf "$USER_CONFIG_JSON_DESTINATION"
    fi
    
    if [ -f "$USER_CONFIG_JSON_DESTINATION" ]; then
        log_warn "The file $USER_CONFIG_JSON_DESTINATION already exists."
        confirm_no "Do you want to reset it to the default? [y/N]" && cp "$USER_CONFIG_JSON_SOURCE" "$USER_CONFIG_JSON_DESTINATION"
    else
        cp "$USER_CONFIG_JSON_SOURCE" "$USER_CONFIG_JSON_DESTINATION"
    fi
    
    log_info "Isaac Sim configuration complete"
}

# Function to configure AirLab Omniverse Nucleus Server Login
function cmd_config_nucleus {
    log_info "Configuring AirLab Nucleus Login..."
    
    # AirLab Omniverse Nucleus Server Login Config
    local OMNI_PASS_SOURCE="${PROJECT_ROOT}/simulation/isaac-sim/docker/omni_pass_TEMPLATE.env"
    local OMNI_PASS_DESTINATION="${PROJECT_ROOT}/simulation/isaac-sim/docker/omni_pass.env"
    
    log_info "Configure AirLab Nucleus Login ($OMNI_PASS_DESTINATION)"
    
    log_info "Go to https://airlab-nucleus.andrew.cmu.edu/omni/web3/, log in, then right click on the cloud and click the \"API Tokens\" window to generate an API token and paste it here. Leave this blank if you want to skip this step: "
    if [ -f "$OMNI_PASS_DESTINATION" ]; then
        log_warn "The file $OMNI_PASS_DESTINATION already exists, leave it blank to skip."
    fi
    read -r -p "API Token: " API_TOKEN
    
    if [ ! -z "${API_TOKEN}" ]; then
        sed "s/PASTE-YOUR-API-TOKEN/$API_TOKEN/g" "$OMNI_PASS_SOURCE" > "$OMNI_PASS_DESTINATION"
        log_info "Nucleus login configuration complete"
    else
        log_info "Skipping Nucleus login configuration"
    fi
}

# Function to set up Git Hooks
function cmd_config_git_hooks {
    log_info "Setting up Git Hooks..."
    
    # Git Hooks
    local HOOKS_SOURCE="${PROJECT_ROOT}/git-hooks/docker-versioning/update-docker-image-tag.pre-commit"
    local HOOKS_DESTINATION="${PROJECT_ROOT}/.git/hooks/pre-commit"
    
    if [ -f "$HOOKS_SOURCE" ]; then
        cp "$HOOKS_SOURCE" "$HOOKS_DESTINATION"
        chmod +x "$HOOKS_DESTINATION"
        log_info "Git hooks setup complete"
    else
        log_error "Git hooks source file not found: $HOOKS_SOURCE"
    fi
}

# Function to run all configuration tasks
function cmd_config_all {
    log_info "Running all configuration tasks..."
    
    cmd_config_isaac_sim
    cmd_config_nucleus
    cmd_config_git_hooks
    
    log_info "All configuration tasks complete"
}

# Register commands from this module
function register_config_commands {
    COMMANDS["config"]="cmd_config_all"
    COMMANDS["config:isaac-sim"]="cmd_config_isaac_sim"
    COMMANDS["config:nucleus"]="cmd_config_nucleus"
    COMMANDS["config:git-hooks"]="cmd_config_git_hooks"
    
    # Add command help
    COMMAND_HELP["config"]="Run all configuration tasks"
    COMMAND_HELP["config:isaac-sim"]="Configure Isaac Sim settings"
    COMMAND_HELP["config:nucleus"]="Configure AirLab Nucleus login"
    COMMAND_HELP["config:git-hooks"]="Set up Git hooks"
}