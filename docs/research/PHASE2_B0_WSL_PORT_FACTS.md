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

## Order-033 DDS hardening (Base=4000)

The Windows-side measurement command is:

    netsh interface ipv4 show excludedportrange protocol=udp

The complete UDP exclusions observed on this host are 62889-62988 and
63089-63188 (the corresponding IPv6 query is not used by CycloneDDS here).
The prior Base=8000 mapping put domain 222 at multicast 63500 and domain 223
at 63750, but did not protect the harness other 200-230 participants from
allocation failures. Base=4000 maps every harness domain 200-230 to multicast
ports 54000-61500 and participant-index p=0..9 unicast ranges 54010-61529;
neither exclusion intersects these ranges. For example, domain 229 uses
multicast 61250 and unicast 61260-61279, while the signal domain remains 229.

The reproducible preload source is
example/cpp/scripts/dds_base4000_preload.c; the generated WSL artifact is
/home/che/dds_base4000_preload.so:

    gcc -shared -fPIC -I/home/che/dev/go2-workspace/external/unitree_sdk2/thirdparty/include \
      example/cpp/scripts/dds_base4000_preload.c -ldl -o /home/che/dds_base4000_preload.so

It injects CycloneDDS Ports Base=4000 while preserving the requested domain ID
and loopback interface. Use LD_PRELOAD with both B0 fixed-pair domains and the
serial B1 canary; no parallel simulation is permitted.

## Order-041 DDS cleanup and participant-index ceiling

Before the Order-041 B0 retry, the WSL /dev/shm directory was inspected after the aborted runs; it contained no CycloneDDS shared-memory segment and no stale DDS processes were running. The Failed to find a free participant index symptom therefore cannot be attributed to a surviving shared-memory owner on this WSL instance. The Base=4000 preload was rebuilt with MaxAutoParticipantIndex=31 (up from 9), retaining ParticipantIndex=auto and the same port base, interface, and domain mapping. This removes the bounded participant-index ceiling while preserving the B0 port workaround.
