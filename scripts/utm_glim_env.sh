#!/usr/bin/env bash

# Source this file from an Ubuntu UTM desktop shell before running GLIM.
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$UID}"
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"

if [[ -z "${XAUTHORITY:-}" ]]; then
    for auth_file in "$XDG_RUNTIME_DIR"/.mutter-Xwaylandauth.*; do
        if [[ -f "$auth_file" ]]; then
            export XAUTHORITY="$auth_file"
            break
        fi
    done
fi

# UTM's virtual GPU may not expose a usable accelerated GLX framebuffer.
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
unset MESA_GL_VERSION_OVERRIDE

printf 'GLIM display: DISPLAY=%s WAYLAND_DISPLAY=%s\n' "$DISPLAY" "$WAYLAND_DISPLAY"
printf 'GLIM renderer: LIBGL_ALWAYS_SOFTWARE=%s MESA_LOADER_DRIVER_OVERRIDE=%s\n' \
    "$LIBGL_ALWAYS_SOFTWARE" "$MESA_LOADER_DRIVER_OVERRIDE"
