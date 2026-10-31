#!/usr/bin/env bash
# Modal spend guard: the Starter plan includes $30 of compute per month; the project stays inside it (spending
# beyond needs the owner's OK). Checks the current month's metered cost.
#   tools/modal_budget.sh            prints this month's metered cost
#   tools/modal_budget.sh <usd>      exits 1 if this month's cost + <usd> (the planned job's estimate) exceeds the limit
set -euo pipefail
LIMIT=${MODAL_BUDGET_USD:-28}  # $30 monthly credit minus a margin for billing lag
D="$(cd "$(dirname "$0")" && pwd)"
month=$(date -u +%Y-%m)
spent=$("$D/modal.sh" billing summary --for "$month" 2>/dev/null | awk '/Metered Cost:/ {print $3}')
spent=${spent:-0}
planned=${1:-0}
echo "modal spend $month \$$spent; planned \$$planned; monthly limit \$$LIMIT (credit \$30)"
python3 -c "import sys; sys.exit(0 if $spent + $planned <= $LIMIT else 1)" || { echo "REFUSED: would exceed this month's credit" >&2; exit 1; }
