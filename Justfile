setup:
  #!/usr/bin/env zsh
  cd ext/dramsim3
  git clone https://github.com/umd-memsys/DRAMsim3.git DRAMsim3
  cd DRAMsim3 && mkdir -p build
  cd build
  cmake ..
  make -j 48

build:
  scons build/RISCV/gem5.opt --gold-linker -j $NIX_BUILD_CORES
  mkdir -p $XS_PROJECT_ROOT/install/bin
  cp $GEM5_HOME/build/RISCV/gem5.opt $XS_PROJECT_ROOT/install/bin/xs-gem5

only-install:
  mkdir -p $XS_PROJECT_ROOT/install/bin
  cp $GEM5_HOME/build/RISCV/gem5.opt $XS_PROJECT_ROOT/install/bin/xs-gem5

prepare:
  nemumake riscv64-gem5-ref_defconfig
  nemumake -j100
  cd $NEMU_HOME/resource/gcpt_restore && make clean && make

xs-run workload:
  mkdir -p $XS_PROJECT_ROOT/out
  xs-gem5 -d $XS_PROJECT_ROOT/out/gem5 $GEM5_HOME/configs/example/xiangshan.py --difftest-ref-so $NEMU_HOME/build/riscv64-nemu-interpreter-so --gcpt-restore /nfs/share/gem5_ci/tools/normal-gcb-restorer.bin --generic-rv-cpt {{workload}}

xs-parallel-run workload:

xs-run-raw workload:
  mkdir -p $XS_PROJECT_ROOT/out
  xs-gem5 -d $XS_PROJECT_ROOT/out/gem5 $GEM5_HOME/configs/example/xiangshan.py --difftest-ref-so $NEMU_HOME/build/riscv64-nemu-interpreter-so --raw-cpt --generic-rv-cpt {{workload}}

xs-run-raw-debug workload:
  mkdir -p $XS_PROJECT_ROOT/out
  gef --tui --args xs-gem5 -d $XS_PROJECT_ROOT/out/gem5 $GEM5_HOME/configs/example/xiangshan.py --difftest-ref-so $NEMU_HOME/build/riscv64-nemu-interpreter-so --raw-cpt --generic-rv-cpt {{workload}}

clean:
  rm -r $GEM5_HOME/build
