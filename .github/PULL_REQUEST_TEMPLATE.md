## Summary

<!-- This repository is release-only: a maintainer lands an accepted pull request in the
     development repository and closes this one with a link to the release that carries
     it. See CONTRIBUTING.md, "Contribution route". -->

<!-- Brief description of the change. -->

Closes #N

## What Changed

<!-- Explain the implementation: what was changed and why. -->

## Verification Checklist

- [ ] `scripts/lint.sh` passes (C++, XML, CMake style)
- [ ] `cd packages && colcon build --symlink-install && colcon test && colcon test-result --verbose` passes (if C++ changes)
- [ ] `python -m pytest tests -v` passes in each tooling directory you changed (`tools/colcon_lf`, `tools/lf_eval`; in `tools/ros2lf` the directory is `test`); `scripts/ci/lint.sh` lists the CI suites (if tooling changes)
- [ ] `python -m pytest scripts/tests -q` passes (if script changes)
- [ ] `mkdocs build --strict` passes (if docs changes)
- [ ] Documentation updated (if user-facing changes)
- [ ] All GitHub Actions workflows are green

## Notes for Maintainers

<!-- Optional: notes for reviewers. -->
