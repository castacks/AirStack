# LWIR baseline

Integrates E3–E4 with configuration-supplied gray emissivity and an effective reflected blackbody environment. The solver uses the same resolved surface and range as RGB. The alternate reference integrates in micrometres using 32-digit arithmetic and composite Simpson quadrature. The center pixel exercises the foreground emissivity and temperature. A separate matched reflected/surface temperature check verifies equality of emitted-plus-reflected radiance across emissivities under the stated conditions.
