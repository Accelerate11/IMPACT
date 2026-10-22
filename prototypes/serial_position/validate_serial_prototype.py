#!/usr/bin/env python3
"""Freeze and verify only the serial baseline core, without changing packages."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess


def digest(path):
    data=path.read_bytes()
    return {"bytes":len(data),"sha256":hashlib.sha256(data).hexdigest()}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint-header",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    args=parser.parse_args()
    prototype=Path(__file__).resolve().parent
    paths={"research_serial_position.h":prototype/"research_serial_position.h",
           "test_research_serial_position.cpp":prototype/"test_research_serial_position.cpp",
           "research_joint_flow.h":args.joint_header.resolve(),
           "protocol.json":prototype/"protocol.json","validate_serial_prototype.py":Path(__file__).resolve()}
    output=args.output_dir.resolve();output.mkdir(parents=True)
    before={name:digest(path) for name,path in paths.items()}
    sources=output/"sources";sources.mkdir()
    for name,path in paths.items():shutil.copyfile(path,sources/name)
    binary=output/"test_research_serial_position"
    command=["g++","-std=c++17","-O0","-g0","-Wall","-Wextra","-Werror",
             "-I/usr/include/eigen3","-I"+str(sources),str(sources/"test_research_serial_position.cpp"),"-o",str(binary)]
    compiled=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    (output/"compile.log").write_text(compiled.stdout)
    tested=subprocess.run([str(binary)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True) if compiled.returncode==0 else None
    (output/"test.log").write_text(tested.stdout if tested else "NOT_RUN\n")
    after={name:digest(path) for name,path in paths.items()}
    matched=re.search(r"^PASS (\d+) checks;",tested.stdout if tested else "",re.MULTILINE)
    passed=compiled.returncode==0 and tested is not None and tested.returncode==0 and matched is not None and before==after
    report={"schema_version":1,"status":"PASS" if passed else "FAIL",
      "scope":"serial_engineering_baseline_standalone_not_native_replay",
      "source_before":before,"source_after":after,"source_file_sets_and_sha256_unchanged":before==after,
      "compiler_version":subprocess.check_output(["g++","--version"],text=True).strip(),
      "eigen_core":digest(Path("/usr/include/eigen3/Eigen/Core")),"compile_command":command,
      "compile_exit_code":compiled.returncode,"test_exit_code":tested.returncode if tested else None,
      "checks":int(matched.group(1)) if matched else None,"binary":digest(binary) if binary.exists() else None,
      "meaningful_cases":["actual_2_4cm_position_perturbation_recovered_from_raw_pixels",
         "isolated_corrupted_correspondence_robustly_trimmed",
         "full_nuisance_covariance_including_mutual_cross_blocks_and_4x_uncertainty",
         "nontruth_native_attitude_mean_perturbed_by_0_04rad",
         "30000_seeded_linearized_measurement_covariance_samples",
         "native_nominal_and_covariance_update_with_same_manifold_reset",
         "identical_joint_prior_distinguishes_serial_update_from_full_raw_pixel_EKF",
         "invalid_covariance_pure_rotation_duplicates_NIS_rejection_without_mutation"],
      "native_replay_validated":False,"AP_closed_loop_validated":False,"hardware_validated":False,
      "superiority_claim":False}
    (output/"result.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
    print(json.dumps({"status":report["status"],"checks":report["checks"],"output":str(output)}))
    return 0 if passed else 1


if __name__=="__main__":raise SystemExit(main())
