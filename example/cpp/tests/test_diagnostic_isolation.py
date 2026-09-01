from pathlib import Path
r=Path(__file__).resolve().parents[1]
c=(r/"CMakeLists.txt").read_text()
assert "add_executable(go2_diagnostic_observer" in c
assert "go2_add_ctest(test_diagnostic_consumer_identity)" in c
s="".join(p.read_text() for p in (r/"diagnostic").glob("*.cpp"))
for x in ("LowCmd_","ChannelPublisher","terrain_planner","trot_experiment","CreateSendChannel"): assert x not in s
print("isolation passed")
