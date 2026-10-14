#!/bin/bash

set -euo pipefail

find src -type f \( -name "*.c" -o -name "*.h" \) -exec clang-format -i {} +
