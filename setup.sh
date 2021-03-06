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

self_dir="$(dirname $(realpath $0))"

: ${ec2_host:="bullet_host"}

if [[ "$#" -eq 0 ]];then
  ${0} help

elif [[ "$1" == "help" ]];then shift
  which bash_scripts >/dev/null && bash_scripts show_commands ${0}

elif [[ "$1" == "create_ec2" ]];then shift
  export security_group="bullet"
  export security_group_id="sg-04063f7cc9a334a64"
  #t3.medium  2(vcpu)     4(gb ram)     0.048($ per hour)
  export instance_type="t3.medium"
  ec2 create ${ec2_host}

elif [[ "$1" == "setup_ec2" ]];then shift
  provision_ec2 docker ${ec2_host}
  ${0} scp
  ${0} mount
  ${0} ports
  ${0} launch

elif [[ "$1" == "show_ip" ]];then shift
  ec2 get-ip ${ec2_host}

elif [[ "$1" == "bullet_logs" ]];then shift
  ec2 ssh ${ec2_host} 'cd /home/ec2-user/cmake_bullet_example/project; docker-compose  logs -t --tail=10 -f bullet'

elif [[ "$1" == "bullet_restart" ]];then shift
  ec2 ssh ${ec2_host} 'cd /home/ec2-user/cmake_bullet_example/project; docker-compose restart bullet'
  ${0} bullet_logs

elif [[ "$1" == "restore" ]];then shift
  sudo umount ${self_dir}/project
  ec2 delete ${ec2_host}
  rm -rf project
  rm -rf project_backup
  git reset --hard HEAD

elif [[ "$1" == "scp" ]];then shift
  ec2 ssh ${ec2_host} 'rm -rf /home/ec2-user/cmake_bullet_example'
  ec2 scp ${ec2_host} cmake_bullet_example ${self_dir} 

elif [[ "$1" == "mount" ]];then shift
  mv ${self_dir}/project ${self_dir}/project_backup
  mkdir ${self_dir}/project
  sshfs -o ssh_command='ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no' ec2-user@$(ec2 get-ip ${ec2_host}):/home/ec2-user/cmake_bullet_example/project ${self_dir}/project

elif [[ "$1" == "launch" ]];then shift
  ec2 ssh ${ec2_host} 'cd /home/ec2-user/cmake_bullet_example/project; docker-compose up -d'

elif [[ "$1" == "ports" ]];then shift
  ec2 port ${ec2_host} 9999
  ec2 port ${ec2_host} 8080
  ec2 port ${ec2_host} 15672
  ec2 port ${ec2_host} 15674

else
  echo "unknown command:$@"
fi

