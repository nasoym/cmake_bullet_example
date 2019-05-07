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

apt-get install -y socat
apt-get install -y curl

until curl -i -u "guest:guest" "http://rabbit:15672/api/queues"; do
  echo waiting for rabbit
  sleep 1
done

rm -rf /cmake_bullet/build || true
mkdir -p /cmake_bullet/build 
cd /cmake_bullet/build 
cmake ../
make
cd /cmake_bullet
  socat TCP-LISTEN:9999,reuseaddr,fork STDOUT \
    | ./build/hello \
    | rabbit_host="rabbit" python3 src/python_pika_write.py

