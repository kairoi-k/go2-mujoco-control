# QR repair actual flat canary: FAILED tracking
Clean runtime 4bff757c1a465fc620edd2f32d220d22819b824b; same fixed-start
21-second protocol as attempt 0003. Raw run is
`example/cpp/experiments/_runs/joint_execution_flat_20260908_0004`.
No WBC failure log occurs. Actual joint authority adopts 10 versions from
STATE21.004 through the last sampled command at21.418 (count208).
CSV retains207 zero-extra-PD rows; sampled stdout count is not CSV row count.
The first hard posture report is roll -27.7917deg, pitch17.3154deg.
Sampled foot-reference error grows to0.461764m; COM error0.00954908m.
Small inverse-dynamics residuals do not establish physical tracking stability.
Sampled feedback latency p50/p95/max231.889/560.0108/636.684us.
The helper audit first_stop=null only means no JointExecution stop_requested
message: the upstream posture safety gate terminates actuation. It must NOT
be interpreted as successful completion. Wrapper reports safety rejection.
B1 remains NOT_CERTIFIED. Next investigate earliest actual/reference/contact
tracking divergence; no gain or acceptance changes are justified by this run.
