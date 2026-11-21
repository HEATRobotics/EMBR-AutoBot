The workflow lints `docker/autonomy.Dockerfile`, scans `docker/`,
`compose.*.yaml`, and `.env.example` for API keys and secrets, verifies every
direct requirement in `docker/requirements-rl.txt` is pinned with `==`, and
builds the autonomy image once with the GitHub Actions BuildKit cache. The job
summary reports the build duration and image size.