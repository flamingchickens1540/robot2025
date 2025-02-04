#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"
for file in ./vendordeps/*.json; do
  url="$(jq .jsonUrl "$file" | tr -d '"')"
  if [[ -z "$url" ]]; then
    continue
  fi
  echo "------$file------"
  ./gradlew vendordep --url "$url"
done
