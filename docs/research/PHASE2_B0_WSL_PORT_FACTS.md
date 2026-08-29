# B0 WSL/CycloneDDS port environment fact

This is an environment workaround, not a Go2 or acceptance-contract change. On
this Windows/WSL host, the Windows command netsh interface ipv4/ipv6
show excludedportrange protocol=udp reports excluded ranges 62889-62988 and
63089-63188. CycloneDDS 0.10.2 default discovery ports for domains 222 and 223
are inside those ranges: domain 222 uses multicast meta/SPDP 62900 and p=0
unicast meta/data 62910/62911 (candidate p=0..9: 62910..62929); domain 223
uses 63150 and 63160/63161 (p=0..9: 63160..63179). WSL socket inspection
found no owning process, so killing a process does not resolve this failure.

The existing temporary workaround is /home/che/dds_base8000_preload.so, built
from /home/che/dds_base8000_preload.c. Its dds_create_domain hook keeps the
interface on lo, leaves the requested domain ID unchanged, and injects
CycloneDDS Discovery Ports Base 8000. The resulting domains are domain 222
multicast 63500 and p=0 unicast 63510/63511; domain 223 multicast 63750 and
p=0 unicast 63760/63761. Domain 229 remains the Phase2 signal domain and is
not remapped by the experiment contract.

Exact reproduction from the project root (the pair remains serial in the
script and retains baseline=222, terrain=223):

    cd /home/che/dev/go2-workspace/current
    LD_PRELOAD=/home/che/dds_base8000_preload.so bash example/cpp/scripts/run_phase2_b0_fixed_pair.sh development 0

For a single Phase2 terrain run, the same inherited preload can be used with
the existing entry point, for example:

    cd /home/che/dev/go2-workspace/current
    LD_PRELOAD=/home/che/dds_base8000_preload.so bash example/cpp/scripts/run_trot.sh 18 b1_holdfix_epoch27_20260828 --headless --wall-clock-motion --wbc-full --gait-pattern running-trot --kernel raibert-trot --period 0.50 --duty 0.75 --step-length 0.15 --foot-lift 0.08 --tau-limit 45 --velocity-max-accel 0.80 --velocity-max-decel 1.20 --velocity-max-jerk 4.0 --velocity-command-script example/cpp/configs/phase2_b1_velocity_0p3.csv --terrain-planner --domain-id 229 --scene-file unitree_robots/go2/phase2_step_5cm.xml --phase2-milestone B1

The permanent choice (Windows reserved-port range or a committed CycloneDDS
XML/configuration) is intentionally left to the human owner.
