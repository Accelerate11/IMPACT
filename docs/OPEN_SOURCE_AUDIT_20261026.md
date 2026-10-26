# 公开复现检查：2026-10-26

本轮从新读者的入口检查文档、命令、资源和验证范围，并执行独立目录验证。实际测试提交为 [2ab6c63](https://github.com/Accelerate11/IMPACT/commit/2ab6c63c7f3a6143d0ee6aae6a9f6173db6ea04f)；本说明及证据随后补入，不改变该提交的算法和运行器。

## 已修正

- 首页解释核心动机、19+6+3维状态及流程；区分数学检查、记录回放、历史仿真和AP接口。
- 增加免ROS/Gazebo、免bag的轻量依赖profile与六项研究入口，CI使用同一安装路径。
- 增加完整英文入口、五方法定义、误差对齐含义、参考结果表、输出字段与排错说明。
- 修正五路输入总数24,144、AP underlay路径、手工测试pipeline退出语义及旧帮助路径。
- 修正Ctrl+C遗留研究/编译子进程的问题；超时、启动失败和未运行组均有明确结果。
- FAST-LIO包元数据与随附GPL v2文本保持一致，保留文件级BSD notices，说明未知原始修订；只改两份许可/来源文件，数值算法未改。
- 更新GitHub About，去除旧Atlas/openEuler部署描述；补组件许可索引、软件引用和论文入口。

## 实际检查结果

环境为WSL Ubuntu22.04/x86_64、系统Python3.10.12。通过公开GitHub clone建立独立目录（路径含空格），检出候选代码后，确认其HEAD与公开origin/main一致、工作树干净。未借用原研究workspace的源码或install。

| 项目 | 结果与范围 |
|---|---|
| 轻量依赖presence check | PASS；本机依赖已安装，本轮不运行apt |
| 中断/超时/启动异常等生命周期测试 | 7项PASS，包含无残留活动worker检查；原失败fixture保留 |
| 六项实际离线研究 | 全部PASS，合计约116秒；含两项真实C++编译，耗时是本机记录，不是性能承诺 |
| 新clone安装发布资源 | 从既有、SHA认证的本地v2压缩包安装PASS，约25秒；核对25,503项 |
| 源码与数据静态检查 | 597源码、15包、21算法组、4原型、模型/SDF及完整B1文件检查PASS |
| 公开Release校验文件 | 实际无认证下载2142字节校验表，18个分卷记录与提交的descriptor一致；本轮没有重新联网下载全部资源 |
| 文档和参考结果 | 25段Bash示例语法通过、本地链接有效、三份结果表与原评估JSON一致 |
| 公式映射 | 映射的45个LaTeX label全部存在；此项是可定位性检查，不代替数学审稿 |
| 保全 | 原PDF/LaTeX ZIP、图集、历史evidence和旧validation均未修改；旧tag保持不变 |

证据索引：[README.json](../server/validation/open_source_audit_20261026/README.json)。独立目录执行记录：[summary.json](../server/validation/open_source_audit_20261026/summary.json)；[六项研究](../server/validation/open_source_audit_20261026/six-studies-summary.json)、[完整静态覆盖](../server/validation/open_source_audit_20261026/source-data-coverage.json)、[公开版本和资源检查](../server/validation/open_source_audit_20261026/public_snapshot_check.json)、[最终文档检查](../server/validation/open_source_audit_20261026/documentation_check_final.json)。绝对路径仅保存审计来源，读者不需要创建这些原路径。

GitHub CI运行依赖安装、7项运行器测试和六项研究：[候选提交的Actions记录](https://github.com/Accelerate11/IMPACT/actions/runs/40345826978)。实时状态以页面为准，不能把本机已有依赖检查当成GitHub全新环境安装成功。

## 本轮没有重新验收的范围

没有在本轮重建完整ROS/Gazebo/AP或重新运行五组原生LIVO回放。它们的已有证据固定在原参考tag，本文没有把它们改标为当前环境新PASS。source/依赖身份变化后，需要按公开入口在新runtime构建。

硬件标定和异步关联、Orin实时性、强串联基线接入、多场景统计、保护水平校准与新融合/AP完整任务仍有独立验收要求。第三方Vikit fork的混合许可声明如实列于[许可索引](../THIRD_PARTY_NOTICES.md)，没有重新授权或承诺统一商业许可。
