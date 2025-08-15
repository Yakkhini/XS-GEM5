#!usr/bin/env bash

function print_help() {
  printf "Usage:
    bash $0 workload_list.lst checkpoint_top_dir\n"
  exit 1
}

if [[ -z "$2" ]]; then
  echo "Arguments not provided!"
  print_help
fi

set -x

# Note 1: workload list contains the workload name, checkpoint path, and parameters, looks like:
#       astar_biglakes_122060000000 astar_biglakes_122060000000_0.244818/0/ 0 0 20 20
#       bwaves_1003220000000 bwaves_1003220000000_0.036592/0/ 0 0 20 20
# Note 2: The meaning of fields:
# workload_name, checkpoint_path, skip insts(usually 0), functional_warmup insts(usually 0), detailed_warmup insts (usually 20), sample insts
# Note 3: you can write a script to generate such a list accordingly
export workload_list=$(realpath $1)

# The checkpoint directory. We will find checkpoint_path in workload_list
# under this directory to get the checkpoint path.
export cpt_dir=$(realpath $2)

export tag=parallel-$(date +%F-%H-%M-%S)-result

export log_file='log.txt'

export ds=$XS_PROJECT_ROOT/out/gem5 # data storage. It is specific for BOSC machines, you can ignore it
export full_work_dir=$ds/$tag       # work dir wheter stats data stored
mkdir -p $full_work_dir

check() {
  if [ $1 -ne 0 ]; then
    echo FAIL
    rm running
    touch abort
    exit
  fi
}

trap cleanup SIGINT

function cleanup() {
  echo "Script interrupted, marking tasks as aborted..."
  find $full_work_dir -type f -name running -execdir bash -c 'rm -f running; touch abort' \;
  exit 1
}

function run() {
  set -x
  hostname

  cd $2 # work_dir

  if test -f "completed"; then
    echo "Already completed; skip $1"
    return
  fi

  rm -f abort
  rm -f completed

  touch running

  script_dir=$(dirname -- "$(readlink -f -- "$0")")
  xs-gem5 $GEM5_HOME/configs/example/xiangshan.py --difftest-ref-so $NEMU_HOME/build/riscv64-nemu-interpreter-so --gcpt-restore /nfs/share/gem5_ci/tools/normal-gcb-restorer.bin --generic-rv-cpt $1
  check $?

  rm running
  touch completed
}

function prepare_env() {
  set -x
  echo "prepare_env $@"
  all_args=("$@")
  task=${all_args[0]}
  task_path=${all_args[1]}
  # 同时匹配 gz zstd 后缀
  suffixes=("gz" "zstd")
  checkpoint=""
  for suffix in "${suffixes[@]}"; do
    checkpoint=$(find -L "$cpt_dir" -wholename "*${task_path}*.$suffix" | head -n 1)
    if [ -n "$checkpoint" ]; then
      break
    fi
  done
  echo $checkpoint

  export work_dir=$full_work_dir/$task
  echo $work_dir
  mkdir -p $work_dir
}

function arg_wrapper() {
  prepare_env $@

  all_args=("$@")
  args=(${all_args[0]})

  skip=${args[2]}
  fw=${args[3]}
  dw=${args[4]}
  sample=${args[5]}

  run $checkpoint $work_dir >$work_dir/$log_file 2>&1
}

export -f check
export -f run
export -f arg_wrapper
export -f prepare_env

# xsgem5_para_jobs define in docker compose
num_threads=$NIX_BUILD_CORES
function parallel_run() {
  # We use gnu parallel to control the parallelism.
  # If your server has 32 core and 64 SMT threads, we suggest to run with no more than 32 threads.
  cat $workload_list | parallel -a - -j $num_threads arg_wrapper {}
}

parallel_run
