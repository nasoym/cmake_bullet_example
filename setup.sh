#!/usr/bin/env bash
set -o errexit  # abort when any commands exits with error
set -o pipefail # abort when any command in a pipe exits with error
set -o nounset  # abort when any unset variable is used
set -o noglob # prevent bash from expanding glob
set -o errtrace # inherits trap on ERR in function and subshell
trap 'echo status:$? line:$LINENO line:$BASH_LINENO command:"$BASH_COMMAND" functions:$(printf " %s" ${FUNCNAME[@]:-})' ERR
if [[ "${trace:=0}" -eq 1 ]];then
  PS4='${LINENO}: '
  set -x
  export trace
fi

