# XiangShan GEM5 模拟器

这是香山处理器的GEM5模拟器版本，目前在SPEC CPU 2006基准测试上与昆明湖处理器性能相当。

## 项目特点

XS-GEM5是专门为香山处理器定制的GEM5模拟器，相比官方GEM5：
- 仅支持全系统模拟（Full System Simulation）
- 支持香山特有的格式和功能
- 包含多个香山特有的功能增强

### 主要功能增强

1. 前端微架构校准
   - 解耦前端设计
   - TAGESC、ITTAGE和可选的Loop预测器
   - 与昆明湖对齐的指令延迟校准

2. 后端微架构校准
   - 分布式调度器
   - 调度/执行延迟校准
   - RVV向量扩展支持

3. 缓存层次结构优化
   - 多种预取器算法：Stream + Berti/Stride + BOP + SMS + Temporal + CDP
   - 主动/被动卸载框架
   - 多预取器协调
   - VA-PA转换支持

4. 其他特性
   - 并行RV PTW（页表遍历）
   - 级联FMA
   - 移动消除
   - L2 TLB和TLB预取
   - CSR修复

## 目录结构

```
.
├── configs/                # 配置文件目录
│   ├── example/           # 示例配置
│   │   └── xiangshan.py   # 香山处理器配置
│   └── common/            # 通用配置
├── src/                   # 源代码目录
│   ├── arch/             # 架构相关代码
│   │   └── riscv/        # RISC-V架构实现
│   ├── cpu/              # CPU相关代码
│   │   ├── o3/          # 乱序执行实现
│   │   └── pred/        # 分支预测器
│   └── mem/              # 内存系统相关代码
├── system/               # 系统相关代码
├── util/                 # 工具脚本
│   └── xs_scripts/       # 香山特有脚本
└── ext/                  # 外部依赖
    └── dramsim3/         # DRAMSim3内存模拟器
```

## 关键代码路径

### 1. 处理器配置
- `configs/example/xiangshan.py`: 香山处理器的基本配置
- `configs/common/XSConfig.py`: 香山特有的配置选项

### 2. CPU核心实现
- `src/cpu/o3/`: 乱序执行核心实现
  - `cpu.cc`: CPU核心的主要实现
  - `fetch.cc`: 取指单元
  - `decode.cc`: 解码单元
  - `rename.cc`: 重命名单元
  - `dispatch.cc`: 分发单元
  - `issue.cc`: 发射单元
  - `execute.cc`: 执行单元
  - `writeback.cc`: 写回单元

### 3. 分支预测器
- `src/cpu/pred/`: 分支预测器实现
  - BTB-based：基于传统BTB的设计
  - 详细内容查看BranchPredictor.py 中的DecoupledBPUWithBTB 模块

### 4. 内存系统
- `src/mem/`: 内存系统实现
  - `cache.cc`: 缓存实现
  - `prefetch/`: 预取器实现
  - `page_table.cc`: 页表实现

### 5. RISC-V架构支持
- `src/arch/riscv/`: RISC-V架构实现
  - `decoder.cc`: 指令解码器
  - `registers.cc`: 寄存器实现
  - `isa.cc`: 指令集实现

## 使用说明

### 环境要求
- Ubuntu 20.04/22.04
- Python 3.8（推荐使用conda环境）
- 其他依赖见README.md

### 构建步骤
1. 安装依赖
2. 克隆并构建DRAMSim3
3. 构建GEM5
4. 设置环境变量
5. 运行模拟器

详细步骤请参考README.md中的说明。

