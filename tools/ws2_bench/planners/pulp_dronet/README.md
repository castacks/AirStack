# PULP-DroNet v3 adapter

The worker and installation instructions now live in the sibling
[`engcang/pulp-dronet` fork](https://github.com/engcang/pulp-dronet/blob/master/README_AIRSTACK.md).
Clone it next to `AirStack/` and build `pulp-dronet-airstack:1.0` using
`docker/Dockerfile.airstack` in that checkout. The adapter registry selects
`pulp_dronet_airstack.py`; no copied worker or weights are needed here.

Run clean qualification using the [common bench instructions](../../README.md)
before interpreting attacked flights. Successful installation alone does not
establish reliable navigation in a new environment.
