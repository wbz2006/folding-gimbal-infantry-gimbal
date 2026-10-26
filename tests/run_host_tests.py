"""Run real C control/state/chassis/queue code with hardware transport stubs, without flashing."""
from pathlib import Path
import os
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
CHASSIS = ROOT.parent / 'folding-gimbal_infantry-chassis'
OUT = ROOT / 'build' / 'host_fold_tests'
OUT.mkdir(parents=True, exist_ok=True)
CC = shutil.which('gcc')
if not CC:
    raise SystemExit('Host gcc unavailable; tests NOT run')
os.environ['PATH'] = str(Path(CC).parent) + os.pathsep + os.environ.get('PATH', '')
INCLUDES = ['tests/stubs', 'application', 'application/gimbal', 'application/cmd',
            'modules/motor/DMmotor', 'modules/motor', 'modules/algorithm', 'modules/imu',
            'modules/master_machine', 'modules/remote', 'modules/message_center', 'modules/daemon',
            'modules/can_comm', 'modules']
COMMON = [CC, '-std=gnu11', '-O1', '-g', '-Wall', '-Wextra', '-Wno-unused-parameter',
          '-Werror=sequence-point', '-Werror=implicit-function-declaration']

def run(name, sources, extra=(), board=ROOT):
    args = COMMON + ['-I'+str(ROOT/'tests/stubs')]
    args += ['-I'+str(board/'application')]
    args += ['-I'+str(ROOT/p) for p in INCLUDES[2:]]
    exe = OUT / (name+'.exe')
    subprocess.run(args + list(extra) + [str(p) for p in sources] + ['-lm', '-o', str(exe)], check=True, cwd=ROOT)
    output = subprocess.check_output([str(exe)], text=True, cwd=ROOT)
    print(output, end='')
    return output

run('control', [ROOT/'tests/control_test.c', ROOT/'modules/algorithm/controller.c',
                ROOT/'modules/algorithm/user_lib.c', ROOT/'modules/message_center/message_center.c'])
run('daemon', [ROOT/'tests/daemon_test.c', ROOT/'modules/daemon/daemon.c'])
run('shoot', [ROOT/'tests/shoot_test.c', ROOT/'modules/message_center/message_center.c'])
run('chassis', [ROOT/'tests/chassis_test.c', ROOT/'modules/algorithm/user_lib.c'],
    ['-DCHASSIS_SOURCE="'+(CHASSIS/'application/chassis/chassis.c').as_posix()+'"'], board=CHASSIS)
for board, label in [(ROOT, 'gimbal'), (CHASSIS, 'chassis')]:
    run('remote_'+label, [ROOT/'tests/remote_test.c'],
        ['-DREMOTE_SOURCE="'+(board/'modules/remote/remote_control.c').as_posix()+'"'], board=board)
    for depth in [1, 4]:
        run('queue_'+label+str(depth), [ROOT/'tests/queue_test.c', board/'modules/message_center/message_center.c'],
            ['-DQUEUE_SIZE='+str(depth)], board=board)
protocols = [run('protocol_'+label, [ROOT/'tests/protocol_test.c'], board=board)
             for board, label in [(ROOT, 'gimbal'), (CHASSIS, 'chassis')]]
assert protocols[0] == protocols[1], 'Two-board ABI mismatch'
for board, expected in [(ROOT, 'GIMBAL_BOARD'), (CHASSIS, 'CHASSIS_BOARD')]:
    header = (board/'application/robot_def.h').read_text(encoding='utf-8')
    actual = re.findall(r'^\s*#define\s+(GIMBAL_BOARD|CHASSIS_BOARD|ONE_BOARD)\b', header, re.M)
    assert actual == [expected], (board, actual)
    # Compare the full shared protocol declarations independently of ABI offsets above.
shared = [(board/'application/robot_def.h').read_text(encoding='utf-8').split('#pragma pack(1)',1)[1]
          for board in (ROOT, CHASSIS)]
assert shared[0] == shared[1], 'Shared declarations differ'
print('PASS both board macros, identical shared definitions and ABI; wire payloads remain 24 / 10 bytes')
