# Atlas local LLM benchmark

Date: 2026-09-03. This is a task-shaped integration benchmark, not a claim
about general model intelligence. Each candidate received the same eight
bounded Go2 diagnosis bundles and had to return the same strict JSON schema;
the class-match rate is useful for this workflow, while the official model
cards remain the source for general capability claims.

## Hardware and runtime

- Atlas: Ryzen 7 9700X, 31 GiB RAM, RTX 5080 with 16,303 MiB VRAM.
- Runtime: native Windows `llama.cpp` CUDA build `b10766-cuda13.3`, launched
  from WSL, loopback-only on `127.0.0.1:8090`.
- Download route: direct Hugging Face on Base, then direct Tailscale transfer
  to Atlas. No NB or GLaDOS route was needed.
- Every tested model was hash-verified. The server is started on demand,
  serialized with physical Activities, and stopped in `finally`.

## Results

| Candidate | Quantized file | Context | GPU at load | Generation | 8-case result | Decision |
| --- | --- | ---: | ---: | ---: | --- | --- |
| Qwen3-Coder-30B-A3B-Instruct | Q2_K_L, 11.33 GB | 16K | 14,095 MiB used / 1,883 free | 163–175 tok/s | 8/8 JSON, 8/8 class, mean 1.45 s | **selected** |
| GPT-OSS 20B | official MXFP4, 12.11 GB | 32K | 13,828 / 2,150 MiB | 251 tok/s | 8/8 JSON, 8/8 class, mean 3.43 s | fallback |
| Devstral Small 2 24B | Q4_K_M, 14.33 GB | 4K | 15,745 / 233 MiB | not retained | 8/8 JSON, 8/8 class, mean 4.89 s | too little headroom |
| Devstral Small 2 24B | Q4_K_M, 14.33 GB | 8K | 15,777 / 201 MiB | not retained | 8/8 JSON, 8/8 class, mean 5.15 s | reject |
| Qwen3.6 27B | Q3_K_M, 13.59 GB | 8K | 15,488 / 490 MiB | 54.5 tok/s | 0/8 usable JSON; reasoning/grammar conflict | reject |

The selected Qwen3-Coder file is
`Qwen3-Coder-30B-A3B-Instruct-Q2_K_L.gguf`, SHA-256
`7add73b0607b498f79157a5f4e4ccddc14ad7afd61d76655e064e1e92476267e`, from
`unsloth/Qwen3-Coder-30B-A3B-Instruct-GGUF@b17cb02dd882d5b6ab62fc777ad2995f19668350`.
Its official parent model card is
<https://huggingface.co/Qwen/Qwen3-Coder-30B-A3B-Instruct>.

GPT-OSS was tested from the official GGUF conversion at
`ggml-org/gpt-oss-20b-GGUF@ef9b12f2ff56c69cf32153a02784e7a3c88bf524`, SHA-256
`27cd6c432c7672cb812a92f611cf3ba7bbc35928262bb1e1253ff4ee6ae35901`.
References: <https://huggingface.co/openai/gpt-oss-20b> and
<https://huggingface.co/ggml-org/gpt-oss-20b-GGUF>.

Devstral Q4_K_M was validated from
`bartowski/mistralai_Devstral-Small-2-24B-Instruct-2512-GGUF@027695770ae1de77c2f6fb19f8e1ba9d65fcd15d`,
SHA-256 `bfd11c8679c6b81eb43763505465d7dcfa72e460ab1c220ecc235a3efadd7f7f`.
The official model card is
<https://huggingface.co/mistralai/Devstral-Small-2-24B-Instruct-2512>.

## Real closed-loop verification

The selected model then ran `atlas-qwen3coder-default-e2e-20260903` through Base
Temporal → Atlas preflight (`build_source`, `run_unit_tests`) → local LLM →
Base. It returned `PASS_DEV`, `source=local_llm`, confidence `1.0`, model hash
matching the pin, and 1.146 s inference latency. No cloud Codex call was
needed. This does not establish formal B0/B1 acceptance; those remain an
explicit human checkpoint.

The GPT-OSS fallback also completed `atlas-local-llm-e2e-20260903` with
`PASS_DEV`, hash verification, and about 2.35 s inference latency.
Artifacts remain on Atlas under
`/home/che/dev/go2-workspace/atlas-artifacts/`; the Devstral benchmark files
are `manual-devstral/benchmark-4096.json` and
`manual-devstral-8192/benchmark-8192.json`.

## Operational choice

Production defaults are Qwen3-Coder Q2_K_L, 16K context, reasoning off, one
Atlas Activity at a time, strict schema, pinned revision and SHA-256. GPT-OSS
MXFP4 remains the rollback target by overriding the documented environment
variables. Devstral is not suitable for the always-available worker because
its Q4 file leaves only about 200 MiB at useful context sizes. Qwen3.6 was not
selected because the current llama.cpp reasoning/grammar path produced no
usable structured responses in this workflow.
