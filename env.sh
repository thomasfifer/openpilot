#!/bin/bash

# Load environment variables from .env file if it exists
if [ -f .env ]; then
  set -a && source .env && set +a
fi

export FINGERPRINT="VOLVO_S60_RECHARGE"
export SKIP_FW_QUERY=1

# Set Konik API endpoints if USE_KONIK is enabled
if [ "$USE_KONIK" = "1" ]; then
  export API_HOST=https://api.konik.ai
  export ATHENA_HOST=wss://athena.konik.ai
fi