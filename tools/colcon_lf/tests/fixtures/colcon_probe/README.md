# colcon probe fixtures

`lf_analyze_import_broken.stderr` is the real stderr of `colcon lf-analyze --help`
(colcon-core 0.21.1, dev image) with the installed `colcon_lf.verb.analyze` raising at
import. colcon logs the extension load failure, drops the verb and then rejects it as
an invalid choice (exit 2). Captured with:

```bash
docker compose run --rm -T -e ENTRYPOINT_SKIP_BUILD=1 dev bash -c '
mkdir -p /tmp/shadow && cp -r /workspace/tools/colcon_lf/colcon_lf /tmp/shadow/
printf "\nraise ImportError(\"rcllf probe repro: lf-analyze import broken\")\n" \
  >> /tmp/shadow/colcon_lf/verb/analyze.py
cd /tmp && PYTHONPATH=/tmp/shadow colcon lf-analyze --help >/dev/null \
  2>/workspace/tools/colcon_lf/tests/fixtures/colcon_probe/lf_analyze_import_broken.stderr'
```

The shadow copy lives in the throwaway container, so the repo and the image stay
unchanged.
