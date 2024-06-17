#!/bin/bash

choose_value() {
    # Creates a user prompt with autocompletion feature to choose some value(s) 
    # amongst a predefined list of values. 
    # Usage: choose_value <message> <values> <default> <multiple_choice>
    # Parameters: 
    #  - message: the user prompt message to display 
    #  - values: the name of the array containing the possible values for the choice (passed by references)
    #  - default: the default value(s)
    #  - multiple_choice: true or false depending on whether multiple choices are possible or not. 
    

    local MESSAGE="$1"
    local -n COMPLETIONS_VALUES="$2"
    local DEFAULT="${3:-}"
    local MULTIPLE_CHOICE="${4:-false}" 

    DEBUG=false

    if $DEBUG; then 
        printf '%s\n' "===================" >&2
        printf '%s\n' "DEBUG: message '$MESSAGE'" >&2
        printf '%s\n' "DEBUG: completions_values '$COMPLETIONS_VALUES'" >&2
        printf '%s\n' "DEBUG: default '$DEFAULT'" >&2
        printf '%s\n' "DEBUG: multiple '$MULTIPLE_CHOICE'" >&2
        printf '%s\n' "===================" >&2
    fi
        
    read -a COMPLETIONS <<< "$COMPLETIONS_VALUES"
    
    if $DEBUG; then
        printf '%s\n' "DEBUG: completions '${COMPLETIONS[@]}'" >&2
    fi

    # Define the custom completion function
    comp() {
        local cur_word
        cur_word="${READLINE_LINE:0:$READLINE_POINT}"
        
        COMPREPLY=()
        COMPREPLY=( $(compgen -W "${COMPLETIONS[*]}" -- "$cur_word") )

        if [ ${#COMPREPLY[@]} -gt 0 ]; then
            # Find the longest common prefix
            local prefix="${COMPREPLY[0]}"
            for ((i=1; i<${#COMPREPLY[@]}; i++)); do
                local str="${COMPREPLY[$i]}"
                while [[ "${str:0:${#prefix}}" != "$prefix" ]]; do
                    prefix="${prefix:0:$((${#prefix}-1))}"
                done
            done
            
            # Display matching options
            display_vals="${COMPREPLY[@]}"
            printf '%s\n' "$display_vals" >&2

            # Insert the completion
            local remaining="${prefix#$cur_word}"
            READLINE_LINE+="$remaining"
            (( READLINE_POINT += ${#remaining} ))

            # DEBUGging
            if $DEBUG; then
                printf '%s\n' "DEBUG: READLINE_LINE=$READLINE_LINE" >&2
                printf '%s\n' "DEBUG: READLINE_POINT=$READLINE_POINT" >&2
            fi
        fi
    }
    export -f comp

    # Enable readline mode
    set -o emacs

    # Bind TAB to our custom completion function
    bind -x '"\t": comp'

    if [[ ! " ${COMPLETIONS[*]} " =~ " $DEFAULT " ]]; then
        DEFAULT="" 
    fi

    PROMPT_MESSAGE=$MESSAGE
    if [[ -n "$DEFAULT" ]]; then 
        PROMPT_MESSAGE="$MESSAGE [default: $DEFAULT]"
    fi

    local selected_values=()


    if [[ "$MULTIPLE_CHOICE" = "true" ]]; then 
        printf '%s\n' "$PROMPT_MESSAGE" >&2
        PROMPT_MESSAGE=">"
    fi

    # Prompt the user for input
    while :; do

        local temp_completions=("${COMPLETIONS[@]}")
        
        if $DEBUG; then
            printf '%s\n' "DEBUG: temp_completions=$temp_completions" >&2
        fi

        # Remove already selected values from the completions list
        for selected in "${selected_values[@]}"; do
            temp_completions=("${temp_completions[@]/$selected}")
        done

        if [ ${#temp_completions[@]} -eq 0 ]; then
            break
        fi

        local -n COMPLETIONS=temp_completions

        read -rep "$PROMPT_MESSAGE " INPUT

        if [[ -z "$INPUT" ]]; then 
            # no input given
            # we can break if some values were already selected or if default is set we can break
            if [[ ${#selected_values[@]} -ne 0 ]] || [[ -n "$DEFAULT" ]]; then
                break
            else
                printf "please select at least a value.\n" >&2
            fi
        elif [[ " ${COMPLETIONS[*]} " =~ " $INPUT " ]]; then
            selected_values+=("$INPUT")
        else
            printf "Invalid selection, please try again.\n" >&2
        fi

        if [[ "$MULTIPLE_CHOICE" = "false" ]] && [ ${#selected_values[@]} -ne 0 ]; then 
            break
        fi
    done

    if [ ${#selected_values[@]} -eq 0 ]; then
        selected_values+=$DEFAULT
    fi

    echo "${selected_values[@]}"
}

