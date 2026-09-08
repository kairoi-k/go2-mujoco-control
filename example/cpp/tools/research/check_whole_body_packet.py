"""Trusted fixture loader for native shadow tests; no controller authority.
Read/hash packet bytes once, verify pinned manifest plus every bound local source,
then pass those same packet bytes over stdin. Native Verify alone is not a file
integrity boundary. Source-root relocation preserves all original hash keys.
"""
import argparse
import hashlib
import json
import pathlib
import subprocess
PACKET_SHA = "d22f01fb3faeb1503da8978114045533df2c2ff41e4715affedcc3fbc0ce6567"
MANIFEST_SHA = "6a111f2f78c9426215bb7c6ab13ebb7e34d215f6780368194403b688879c3216"
def digest(data):
    return hashlib.sha256(data).hexdigest()
def validated_bytes(directory, root):
    directory, root = pathlib.Path(directory), pathlib.Path(root).resolve()
    packet = (directory / "trajectory.packet").read_bytes()
    manifest_bytes = (directory / "manifest.json").read_bytes()
    if digest(packet) != PACKET_SHA or digest(manifest_bytes) != MANIFEST_SHA:
        raise ValueError("pinned research fixture bytes changed")
    manifest = json.loads(manifest_bytes)
    if packet.splitlines()[1] != ("manifest_sha256 " + MANIFEST_SHA).encode():
        raise ValueError("packet manifest binding mismatch")
    if not manifest["research_only"] or manifest["production_ready"] or manifest["terrain"]["observed"]:
        raise ValueError("research authority mismatch")
    old_root = pathlib.PurePosixPath(manifest["model"]["scene"]).parents[2]
    def local(name):
        relative = pathlib.PurePosixPath(name).relative_to(old_root)
        path = (root / relative).resolve()
        path.relative_to(root)
        return path
    bound = dict(manifest["all_bound_input_hashes"])
    bound.update(manifest["model"]["recursive_hashes"])
    for key in ("nominal", "gain_evidence"):
        source = manifest["source_hashes"][key]
        bound[source["path"]] = source["sha256"]
    for name, expected in bound.items():
        if digest(local(name).read_bytes()) != expected:
            raise ValueError("bound source mismatch: " + name)
    model_digest = digest(json.dumps(manifest["model"]["recursive_hashes"], sort_keys=True, separators=(",", ":")).encode())
    if model_digest != manifest["model"]["identity_sha256"]:
        raise ValueError("model identity mismatch")
    return packet, local(manifest["model"]["scene"]), len(bound)
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("native_test")
    parser.add_argument("packet_directory")
    parser.add_argument("--root", default=str(pathlib.Path(__file__).resolve().parents[4]))
    args = parser.parse_args()
    packet, scene, count = validated_bytes(args.packet_directory, args.root)
    subprocess.run([args.native_test, "-", str(scene), PACKET_SHA, MANIFEST_SHA], input=packet, check=True)
    print(f"Pinned packet and {count} bound sources verified; identical bytes supplied to shadow parser.")
if __name__ == "__main__":
    main()
