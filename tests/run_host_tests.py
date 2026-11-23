"""Build and run production controls with the repository's existing hardware fixtures."""
from pathlib import Path
import os
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
CHASSIS = ROOT.parent / 'folding-gimbal_infantry-chassis-restore-folded-mode'
OUT = ROOT / 'build' / 'host_fold_tests'
OUT.mkdir(parents=True, exist_ok=True)
CC = shutil.which('gcc')
if not CC:
    raise SystemExit('Host gcc unavailable; tests NOT run')
os.environ['PATH'] = str(Path(CC).parent) + os.pathsep + os.environ.get('PATH', '')
INCLUDES = ['application/gimbal', 'application/cmd', 'modules/motor/DMmotor',
            'modules/motor', 'modules/algorithm', 'modules/imu', 'modules/master_machine',
            'modules/remote', 'modules/message_center', 'modules/daemon', 'modules/can_comm', 'modules']
COMMON = [CC, '-std=gnu11', '-O1', '-g', '-Wall', '-Wextra', '-Wno-unused-parameter',
          '-Werror=sequence-point', '-Werror=implicit-function-declaration']


def run(name, sources, extra=(), board=ROOT):
    args = COMMON + ['-I'+str(ROOT/'tests/stubs'), '-I'+str(board/'application')]
    args += ['-I'+str(board/p) for p in INCLUDES]
    exe = OUT / (name+'.exe')
    subprocess.run(args + list(extra) + [str(p) for p in sources] + ['-lm', '-o', str(exe)],
                   check=True, cwd=ROOT)
    result = subprocess.run([str(exe)], stdout=subprocess.PIPE, text=True, cwd=ROOT)
    print(result.stdout, end='', flush=True)
    result.check_returncode()
    return result.stdout


fold_sources = [ROOT/'tests/gimbal_fold_test.c', ROOT/'modules/remote/remote_control.c',
                ROOT/'modules/can_comm/can_comm.c', ROOT/'modules/algorithm/crc8.c',
                ROOT/'modules/algorithm/controller.c', ROOT/'modules/algorithm/user_lib.c',
                ROOT/'modules/message_center/message_center.c', ROOT/'modules/daemon/daemon.c']
run('gimbal_fold', fold_sources)
run('gimbal_fold_center', fold_sources, ['-DTEST_MECHANICAL_CENTER'])
run('daemon', [ROOT/'tests/daemon_test.c', ROOT/'modules/daemon/daemon.c'])
run('imu_calibration', [ROOT/'tests/imu_calibration_test.c'])
run('shoot', [ROOT/'tests/shoot_test.c', ROOT/'modules/message_center/message_center.c',
              ROOT/'modules/daemon/daemon.c', ROOT/'modules/motor/DMmotor/dmmotor.c',
              ROOT/'modules/motor/DJImotor/dji_motor.c', ROOT/'modules/algorithm/controller.c',
              ROOT/'modules/algorithm/user_lib.c'])
run('chassis', [ROOT/'tests/chassis_test.c', ROOT/'modules/algorithm/user_lib.c',
                ROOT/'modules/daemon/daemon.c'],
    ['-DCHASSIS_SOURCE="'+(CHASSIS/'application/chassis/chassis.c').as_posix()+'"'], board=CHASSIS)
for board, label in [(ROOT, 'gimbal'), (CHASSIS, 'chassis')]:
    run('dji_'+label, [ROOT/'tests/dji_test.c', board/'modules/algorithm/controller.c',
                      board/'modules/daemon/daemon.c'],
        ['-DDJI_SOURCE="'+(board/'modules/motor/DJImotor/dji_motor.c').as_posix()+'"'], board=board)
    run('theta_'+label, [ROOT/'tests/theta_test.c', board/'modules/algorithm/user_lib.c'], board=board)
    run('remote_'+label, [ROOT/'tests/remote_test.c', board/'modules/daemon/daemon.c'],
        ['-DREMOTE_SOURCE="'+(board/'modules/remote/remote_control.c').as_posix()+'"'], board=board)
    for depth in [1, 4]:
        run('queue_'+label+str(depth), [ROOT/'tests/queue_test.c', board/'modules/message_center/message_center.c'],
            ['-DQUEUE_SIZE='+str(depth)], board=board)

protocols = [run('protocol_'+label, [ROOT/'tests/protocol_test.c'], board=board)
             for board, label in [(ROOT, 'gimbal'), (CHASSIS, 'chassis')]]
assert protocols[0] == protocols[1], 'Two-board ABI mismatch'
shared = [(board/'application/robot_def.h').read_text(encoding='utf-8').split('#pragma pack(1)', 1)[1]
          for board in (ROOT, CHASSIS)]


def declarations(text):
    return re.sub(r'\s+', '', re.sub(r'//[^\n]*|/\*.*?\*/', '', text, flags=re.S))


assert declarations(shared[0]) == declarations(shared[1]), 'Shared declarations differ'
print('PASS both boards share protocol declarations and ABI; wire payloads remain 24 / 65 bytes')
