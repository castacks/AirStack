"""Testing the stability of the API in this module"""

import omni.graph.core.tests as ogts
import airlab.pegasus as ogs
from omni.graph.tools.tests.internal_utils import _check_module_api_consistency, _check_public_api_contents


# ======================================================================
class _TestOmniGraphAscentNodeApi(ogts.OmniGraphTestCase):
    _UNPUBLISHED = ["bindings", "ogn", "tests"]

    async def test_api(self):
        _check_module_api_consistency(ogs, self._UNPUBLISHED)  # noqa: PLW0212
        _check_module_api_consistency(ogs.tests, is_test_module=True)  # noqa: PLW0212

    async def test_api_features(self):
        """Test that the known public API features continue to exist"""
        _check_public_api_contents(ogs.tests, [], [], only_expected_allowed=True)  # noqa: PLW0212
