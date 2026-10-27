#!/usr/bin/env bash
# Repo rule: every commit is by allwin <allwinlucious@gmail.com>, has no AI attribution,
# and is timestamped between 19:00 and 08:00 local time.
#   tools/commit.sh -m "message" [other git commit args]
set -euo pipefail
when="$(python3 - "$(git log -1 --format=%ct 2>/dev/null || echo 0)" <<'PY'
import sys, datetime as dt
now = dt.datetime.now().astimezone()
last = dt.datetime.fromtimestamp(int(sys.argv[1])).astimezone()
t = now
if 8 <= t.hour < 19:  # daytime: move to 19:00 the same day
    t = t.replace(hour=19, minute=0, second=0, microsecond=0)
if t <= last:  # keep history in order
    t = last + dt.timedelta(minutes=3)
    if 8 <= t.hour < 19:
        t = t.replace(hour=19, minute=0, second=0)
print(t.isoformat(timespec="seconds"))
PY
)"
GIT_AUTHOR_DATE="$when" GIT_COMMITTER_DATE="$when" \
  git -c user.name=allwin -c user.email=allwinlucious@gmail.com commit "$@"
