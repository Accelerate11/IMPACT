# 论文初稿与可编辑源码

版本基于 2026-10-21 的当前研究成果，作者信息按要求暂留空。

- [英文PDF](IMPACT_CMICA2026_Draft_20261021.pdf)：正文与完整数学附录，19页。
- [英文LaTeX](IMPACT_CMICA2026_LaTeX_20261021.zip)：可编辑工程、图表、数据与独立核对记录。
- [中文PDF](IMPACT_CMICA2026_Chinese_Draft_20261021.pdf)：完整中文审阅版，30页。
- [中文LaTeX](IMPACT_CMICA2026_Chinese_LaTeX_20261021.zip)：可编辑工程，选择XeLaTeX编译。

中英文版本均保留 108 个编号公式。ZIP 为完整自包含论文项目；压缩包保留原始文件字节与清单，避免 Git 换行转换影响审计身份。新的服务器入口与实验代码见仓库根目录 README。

论文保留了验证边界：一段输入上的结果不代表多场景统计优越性；AP 未解锁接口验证不代表新联合融合驱动的完整任务；完整附录供审阅，正式投稿篇幅另按会议要求处理。

中英文ZIP解压后均从 `main.tex` 开始，包含编译说明；`sections/method.tex` 为方法，`sections/derivations.tex` 为推导，`sections/planning_interface.tex` 为规划接口。文件路径相对于ZIP根目录，不是仓库中已解压的文件。编译环境与绘图命令遵循各工程自带说明。

[English guide](../README.en.md) · [公式到代码](../server/ALGORITHM_COVERAGE.md) · [五组参考结果与字段](../docs/REFERENCE_RESULTS.md) · [软件引用方式](../CITATION.md)。文稿为初稿，没有据此声明会议录用、发表或DOI。
