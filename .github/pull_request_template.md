# What features did you add and/or bugs did you address?
- Which GitHub issue does this address?

- Additional description if not fully described in the GitHub issue

- Please add videos and images to demonstrate the feature. Please upload videos to somewhere persistent (e.g. YouTube or Vimeo) for archival purposes.

# How did you implement it?
- Algorithm details, design decisions, engineering notes, and any other relevant information about the implementation should be included

# How do you run and use it?
- What commands and button presses do you use to manually launch the stack to use your new feature? 

- Write a detailed procedure with EXACT BASH COMMANDS so that another maintainer can replicate and understand the benefits of your feature, and reproduce the videos and images you added above.


# Testing with PyTest
<!-- Tests should be added under `tests/` and should be run with `airstack test` (i.e. pytest). -->

- What pytests did you add to ensure the feature is reliable and robust? What metrics are used?

- What's the exact command to run the pytests that test your feature? i.e. `airstack test -m ...`

- What are the expected results of the tests? What should a maintainer look at to understand whether the test succeeded?

<!-- 
Upon submitting this pull request, tests will be automatically run by GitHub Actions if you're a core contributor. 
Core contributors can trigger further tests by commenting "/pytest -m <MARKS> <args>" under this pull request.
-->


# Documentation

- Was mkdocs.yml updated? (y/n)

- Do the docs have sufficient scope such that a newcomer can easily reproduce and use your feature?

- Is there sufficient visual media?

<!-- 
FYI Docs are updated via mkdocs.yml and markdown files under `docs/`. It should render at localhost:8000 when you run `docker compose up docs`.
-->

# Versioning
- Did you make sure to bump the [version number](https://github.com/castacks/AirStack/blob/main/.env#L15) in the `.env` file according to [semantic versioning](https://semver.org/)?

<!-- 
  1. Look at the current version on main
  2. If the version ENDS WITH A LABEL ALPHA/BETA/RC followed by a number, just bump the trailing label number (0.18.0-alpha.5 -> 0.18.0-alpha.6)
  3. If the version is ONLY numbers, e.g. 0.18.3
      a. if it's a new feature, consider bumping the MINOR number and adding the ALPHA label (0.18.3 -> 0.19.0-alpha.1)
      b. if it's a hot fix just bump the PATCH number (0.18.3 -> 0.18.4)
-->
