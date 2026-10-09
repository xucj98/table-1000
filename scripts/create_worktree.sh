#!/usr/bin/env bash
# Planning-stage MAM entry: isolated Git checkout, no runtime dependencies yet.
# Install as a regular file at .local/create_worktree.sh in the primary checkout.
set -euo pipefail
if [[ $# != 3 ]]; then
  echo 'usage: create_worktree.sh BASE_COMMIT BRANCH WORKSPACE_ROOT' >&2
  exit 2
fi
repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
base_commit=$(git -C "$repo_root" rev-parse --verify "$1^{commit}")
task_branch=$2
task_workspace=$3
[[ "$task_branch" == task/* ]] || { echo 'expected MAM task/ branch' >&2; exit 2; }
[[ "$task_workspace" == /* && "$task_workspace" != / ]] || { echo 'expected absolute workspace path' >&2; exit 2; }
[[ "$(realpath -m -- "$task_workspace")" == "$task_workspace" ]] || { echo 'workspace path must be normalized' >&2; exit 2; }
git -C "$repo_root" check-ref-format --branch "$task_branch" >/dev/null
task_checkout="$task_workspace/table-1000"
if [[ -e "$task_checkout" || -L "$task_checkout" ]]; then
  [[ ! -L "$task_checkout" && -f "$task_checkout/.git" && ! -L "$task_checkout/.git" ]]
  [[ "$(git -C "$task_checkout" rev-parse --path-format=absolute --git-common-dir)" == "$repo_root/.git" ]]
  [[ "$(git -C "$task_checkout" symbolic-ref --short HEAD)" == "$task_branch" ]]
else
  mkdir -p -- "$task_workspace"
  git -C "$repo_root" worktree add -b "$task_branch" "$task_checkout" "$base_commit"
fi
