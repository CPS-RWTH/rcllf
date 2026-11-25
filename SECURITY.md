# Security Policy

## Reporting a Vulnerability

rcllf is a research prototype. Fixes go into the next release, and only the latest release is supported.

If you discover a security vulnerability, please report it privately to:

**harun.teper@cps.rwth-aachen.de**

Include:
- Description of the vulnerability
- Steps to reproduce (if applicable)
- Affected versions or components
- Your suggested fix (if any)

We aim to acknowledge reports within 5 business days and will work on a fix on a best-effort basis. Public disclosure will be coordinated with you before any release.

## Security Considerations

As this is an early-stage research project:
- Releases are prototypes, not production software
- Breaking changes and refactoring are frequent
- The codebase integrates multiple third-party systems (ROS 2, Lingua Franca, reactor-c)
- Dependencies are tracked in `docker-compose.yml` and submodules in `third_party/`

Please review the Apache-2.0 license (`LICENSE`) and the disclaimer therein.
