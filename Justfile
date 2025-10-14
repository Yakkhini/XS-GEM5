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

build-debug:
  scons build/RISCV/gem5.debug --gold-linker -j $NIX_BUILD_CORES
  mkdir -p $XS_PROJECT_ROOT/install/bin
  cp $GEM5_HOME/build/RISCV/gem5.debug $XS_PROJECT_ROOT/install/bin/xs-gem5-debug

only-install:
  mkdir -p $XS_PROJECT_ROOT/install/bin
  cp $GEM5_HOME/build/RISCV/gem5.opt $XS_PROJECT_ROOT/install/bin/xs-gem5

prepare:
  nemumake riscv64-gem5-ref_defconfig
  nemumake -j100
  cd $NEMU_HOME/resource/gcpt_restore && make clean && make

xs-run workload:
  #!/usr/bin/env zsh
  mkdir -p $XS_PROJECT_ROOT/out
  export tag=single-$(date +%F)-$(cd $GEM5_HOME && git rev-parse --short HEAD)
  mkdir -p $XS_PROJECT_ROOT/out/gem5/$tag
  # should not hardcode the path here like `nfs`
  xs-gem5 -d $XS_PROJECT_ROOT/out/gem5/$tag $GEM5_HOME/configs/example/xiangshan.py --enable-bp-db tage --bp-type=DecoupledBPUWithBTB --disable-mgsc --ideal-kmhv3 --difftest-ref-so $NEMU_HOME/build/riscv64-nemu-interpreter-so --gcpt-restore /nfs/share/gem5_ci/tools/normal-gcb-restorer.bin --generic-rv-cpt {{workload}} > $XS_PROJECT_ROOT/out/gem5/$tag/log

xs-parallel-run workload:

xs-run-raw workload:
  #!/usr/bin/env zsh
  mkdir -p $XS_PROJECT_ROOT/out
  export tag=single-raw-$(date +%F)-$(cd $GEM5_HOME && git rev-parse --short HEAD)
  mkdir -p $XS_PROJECT_ROOT/out/gem5/$tag
  xs-gem5 -d $XS_PROJECT_ROOT/out/gem5/$tag $GEM5_HOME/configs/example/xiangshan.py --enable-bp-db tage --bp-type=DecoupledBPUWithBTB --disable-mgsc --ideal-kmhv3 --difftest-ref-so $NEMU_HOME/build/riscv64-nemu-interpreter-so --raw-cpt --generic-rv-cpt {{workload}}

xs-run-raw-debug workload:
  #!/usr/bin/env zsh
  mkdir -p $XS_PROJECT_ROOT/out
  export tag=single-raw-$(date +%F)-$(cd $GEM5_HOME && git rev-parse --short HEAD)
  mkdir -p $XS_PROJECT_ROOT/out/gem5/$tag
  gdb --args xs-gem5-debug -d $XS_PROJECT_ROOT/out/gem5/$tag $GEM5_HOME/configs/example/xiangshan.py --enable-bp-db tage --bp-type=DecoupledBPUWithBTB --disable-mgsc --ideal-kmhv3 --difftest-ref-so $NEMU_HOME/build/riscv64-nemu-interpreter-so --raw-cpt --generic-rv-cpt {{workload}}

run-dot8coverage:
  # should not hardcode the path here like `nfs`
  bash $GEM5_HOME/util/xs_scripts/parallel_sim.sh /nfs/share/gem5_ci/spec06_cpts/spec_0.8c_int.lst /nfs/home/share/jiaxiaoyu/simpoint_checkpoint_zstd_format/spec06_rv64gcb_O3_20m_gcc12.2.0-intFpcOff-jeMalloc

clean:
  rm -r $GEM5_HOME/build
