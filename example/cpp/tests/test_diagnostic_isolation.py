from pathlib import Path
r=Path(__file__).resolve().parents[1]
c=(r/"CMakeLists.txt").read_text()
assert "add_executable(go2_diagnostic_observer" in c
assert "go2_add_ctest(test_diagnostic_consumer_identity)" in c
s="".join(p.read_text() for p in (r/"diagnostic").glob("*.cpp"))
assert "ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>" in s
assert "TopicName(Topic::kLowCmd)" in s
assert "void DdsCapture::OnLowCmd(const void *) noexcept" in s
assert "++lowcmd_count_;" in s
callback = s.split("void DdsCapture::OnLowCmd", 1)[1].split("void DdsCapture::OnSportState", 1)[0]
assert "static_cast" not in callback
body = callback.split("{", 1)[1]
code = "\n".join(line.split("//", 1)[0] for line in body.splitlines())
assert "." not in code
for x in ("ChannelPublisher", "CreateSendChannel", "Publish", "terrain_planner", "trot_experiment", "LowCmdWrite", "Controller"): assert x not in s
assert "LowCmd_" in s
print("isolation passed")
