#!/usr/bin/env bash
# Modal spend guard: the project's total budget is $30 across all months from 2026-10.
#   tools/modal_budget.sh            prints the total metered cost so far
#   tools/modal_budget.sh <usd>      exits 1 if total + <usd> (the planned job's estimate) exceeds the limit
set -euo pipefail
LIMIT=${MODAL_BUDGET_USD:-28}  # $30 cap minus a margin for billing lag
D="$(cd "$(dirname "$0")" && pwd)"
total=0
month=2026-10
while [[ "$month" < "$(date +%Y-%m)" || "$month" == "$(date +%Y-%m)" ]]; do
  cost=$("$D/modal.sh" billing summary --for "$month" 2>/dev/null | awk '/Metered Cost:/ {print $3}')
  total=$(python3 -c "print(round($total + ${cost:-0}, 2))")
  month=$(date -d "$month-01 +1 month" +%Y-%m)
done
planned=${1:-0}
echo "modal spend so far \$$total; planned \$$planned; limit \$$LIMIT (cap \$30)"
python3 -c "import sys; sys.exit(0 if $total + $planned <= $LIMIT else 1)" || { echo "REFUSED: would exceed the budget" >&2; exit 1; }
