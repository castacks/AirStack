# Contributing

This page describes how to merge content back into main.

## Dependencies
Make sure to add your ROS2 package dependencies to your `package.xml` file.
These get installed when the docker image is built.

If you need to add a dependency that's not in the docker image, please add a section to the `Dockerfile` in the `docker/` directory.

## Documentation

Please make sure to document your work.
Docs are under `AirStack/docs/`. The navigation tree is under `AirStack/mkdocs.yml`.

This documentation is built with Material MKDocs.
Visit [mkdocs.org](https://www.mkdocs.org) and [mkdocs-material](https://squidfunk.github.io/mkdocs-material/) to learn how to use it.

### Commands

```
pip install mkdocs-material
mkdocs serve
```
Launches docs on https://localhost:8000.

- `mkdocs -h` - Print help message and exit.

### Project layout

    mkdocs.yml    # The configuration file.
    docs/
        index.md  # The documentation homepage.
        ...       # Other markdown pages, images and other files.

## Merge

Submit a pull request.

All tests must pass before merging.

Regression tests are run so that we don't break anything.