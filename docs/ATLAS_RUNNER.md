# Atlas task runner

This repository can dispatch a small allow-list of maintenance tasks to the
Atlas WSL2 Ubuntu-22.04 self-hosted runner through a labeled GitHub Issue.

The Atlas task dispatcher workflow listens only for newly opened issues with
the atlas-task label, and only accepts issues authored by an owner, member, or
collaborator. It has no pull-request trigger. Before execution it checks out
the trusted main branch with credentials removed. The issue body is parsed as
data; it is never passed to a shell.

The current task contract is:

~~~json
{"task":"repo-smoke","parameters":{}}
~~~

The only tasks are repo-smoke and workspace-status. Each task is mapped to
fixed argument lists in tools/atlas_dispatch.py; arbitrary commands,
arguments, paths, branches, and PR contents are rejected. Results are written
to an artifact and summarized back on the Issue. Successful Issues are labeled
atlas-complete and closed; failures are labeled atlas-failed and remain open.

The runner is installed outside the checkout at
/home/che/actions-runner, registered as atlas, and managed by its systemd
service. Windows starts the WSL instance with the Codex Atlas WSL runner
keepalive scheduled task so the service can reconnect after reboot.
