#!/usr/bin/env bash
"$1" 2>&1
rc=$?
echo "EXIT_CODE=$rc"
