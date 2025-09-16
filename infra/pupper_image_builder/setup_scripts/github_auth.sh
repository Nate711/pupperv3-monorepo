#!/bin/bash

# Helper functions to configure git to use a GitHub token via GIT_ASKPASS
# without persisting the token to disk.

setup_github_auth() {
    if [ -z "${GITHUB_TOKEN:-}" ]; then
        return 0
    fi

    if [ -n "${_GIT_ASKPASS_SCRIPT:-}" ]; then
        return 0
    fi

    _GIT_ASKPASS_SCRIPT="$(mktemp /tmp/git-askpass-XXXXXX)"

    cat <<'SCRIPT' > "${_GIT_ASKPASS_SCRIPT}"
#!/bin/sh
case "$1" in
    Username*)
        echo "x-access-token"
        ;;
    Password*)
        echo "${GITHUB_TOKEN:-}"
        ;;
    *)
        exit 0
        ;;
esac
SCRIPT

    chmod 700 "${_GIT_ASKPASS_SCRIPT}"

    if [ -n "${GIT_ASKPASS:-}" ]; then
        _GIT_PREV_GIT_ASKPASS="${GIT_ASKPASS}"
    else
        unset _GIT_PREV_GIT_ASKPASS
    fi

    if [ -n "${GIT_TERMINAL_PROMPT:-}" ]; then
        _GIT_PREV_GIT_TERMINAL_PROMPT="${GIT_TERMINAL_PROMPT}"
    else
        unset _GIT_PREV_GIT_TERMINAL_PROMPT
    fi

    export GIT_ASKPASS="${_GIT_ASKPASS_SCRIPT}"
    export GIT_TERMINAL_PROMPT=0
    return 0
}

cleanup_github_auth() {
    if [ -n "${_GIT_ASKPASS_SCRIPT:-}" ]; then
        rm -f "${_GIT_ASKPASS_SCRIPT}"
    fi

    if [ -n "${_GIT_PREV_GIT_ASKPASS+x}" ]; then
        if [ -n "${_GIT_PREV_GIT_ASKPASS:-}" ]; then
            export GIT_ASKPASS="${_GIT_PREV_GIT_ASKPASS}"
        else
            unset GIT_ASKPASS
        fi
    else
        unset GIT_ASKPASS
    fi

    if [ -n "${_GIT_PREV_GIT_TERMINAL_PROMPT+x}" ]; then
        if [ -n "${_GIT_PREV_GIT_TERMINAL_PROMPT:-}" ]; then
            export GIT_TERMINAL_PROMPT="${_GIT_PREV_GIT_TERMINAL_PROMPT}"
        else
            unset GIT_TERMINAL_PROMPT
        fi
    else
        unset GIT_TERMINAL_PROMPT
    fi

    unset _GIT_ASKPASS_SCRIPT
    unset _GIT_PREV_GIT_ASKPASS
    unset _GIT_PREV_GIT_TERMINAL_PROMPT

    if [ -n "${GITHUB_TOKEN:-}" ]; then
        unset GITHUB_TOKEN
    fi

    return 0
}
