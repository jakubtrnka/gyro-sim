"""Check documentation links and the installed launcher, outside the source tree."""

import pathlib
import re
import signal
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request


def check_links(root):
    documents = [root / "README.md", root / "gyroscope-simulation-design.md"]
    documents.extend((root / "docs").glob("*.md"))
    for document in documents:
        text = document.read_text()
        links = re.findall(r"\[[^\]]+\]\(([^\s)]+)\)", text)
        links += re.findall(r"^\[[^\]]+\]:\s+(\S+)", text, re.MULTILINE)
        for link in links:
            url = urllib.parse.urlsplit(link.strip("<>"))
            if url.scheme or url.netloc or not url.path:
                continue
            target = document.parent / urllib.parse.unquote(url.path)
            assert target.exists(), f"Broken link in {document}: {link}"


launcher, assets_arg, source_arg = sys.argv[1:]
assets = pathlib.Path(assets_arg)
check_links(pathlib.Path(source_arg))
check_links(assets)

with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryFile(mode="w+") as log:
    server = subprocess.Popen(
        [launcher, "--port", "0"],
        cwd=directory,
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    try:
        deadline = time.monotonic() + 20
        address = None
        while time.monotonic() < deadline:
            log.seek(0)
            output = log.read()
            address = re.search(r"http://127\.0\.0\.1:\d+", output)
            if address or server.poll() is not None:
                break
            time.sleep(0.05)
        assert address, f"Packaged server failed to start: {output}"
        base = address.group()
        # Do not let host proxy settings redirect local requests.
        client = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        for relative in [
            "index.html",
            "README.md",
            "docs/01-gyroscope-mechanics.md",
            "docs/02-simulation-design.md",
            "gyroscope-simulation-design.md",
        ]:
            with client.open(f"{base}/{relative}", timeout=5) as response:
                assert response.read() == (assets / relative).read_bytes(), relative
        wasm_files = list(assets.glob("assets/*.wasm"))
        assert wasm_files, "Production bundle has no WASM asset"
        for wasm in wasm_files:
            relative = wasm.relative_to(assets)
            with client.open(f"{base}/{relative}", timeout=5) as response:
                assert response.headers.get_content_type() == "application/wasm"
                assert response.read() == wasm.read_bytes()
        try:
            client.open(f"{base}/missing-asset.js", timeout=5)
        except urllib.error.HTTPError as error:
            assert error.code == 404
        else:
            raise AssertionError("Missing assets must return 404")
    finally:
        server.send_signal(signal.SIGINT)
        try:
            server.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait()
    assert server.returncode == 0, f"Server shutdown failed: {server.returncode}"

print("PASS documentation links, packaged launcher, bundled assets, WASM MIME, and 404")
