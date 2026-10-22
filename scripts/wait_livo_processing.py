#!/usr/bin/env python3
"""Wait for the mapper's atomic processing completion, never just receipt."""
import argparse
import json
import os
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--mapper-pid', type=int, required=True)
    parser.add_argument('--timeout-s', type=float, default=30)
    args = parser.parse_args()
    if args.mapper_pid <= 1 or not 0 < args.timeout_s <= 60:
        raise ValueError('invalid owned PID or bounded timeout')
    deadline = time.monotonic() + args.timeout_s
    while True:
        os.kill(args.mapper_pid, 0)
        path = args.run_dir / 'processing-result.json'
        if path.exists():
            result = json.loads(path.read_text())
            if result.get('status') != 'COMPLETE':
                raise RuntimeError('mapper processing did not complete: ' + str(result))
            print(json.dumps({'status': 'PASS', 'completion': result}))
            return
        if time.monotonic() >= deadline:
            raise TimeoutError('all inputs published, but complete image processing was not acknowledged')
        time.sleep(0.05)


if __name__ == '__main__':
    main()
