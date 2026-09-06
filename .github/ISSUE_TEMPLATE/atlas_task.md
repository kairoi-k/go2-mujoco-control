---
name: Atlas task
about: Run one allow-listed task on the Atlas WSL runner
title: "[atlas] "
labels: atlas-task
assignees: ""
---

Only repository owners, members, and collaborators can start Atlas work.

Put exactly one JSON object in the body:

~~~json
{"task":"repo-smoke","parameters":{}}
~~~

Allowed tasks currently are repo-smoke and workspace-status. Arbitrary
commands, paths, branches, and pull-request code are not accepted.
