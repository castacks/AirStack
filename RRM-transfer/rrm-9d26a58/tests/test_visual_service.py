"""Capability inspection must not turn worker health into visual qualification."""
import io
import json
import unittest
from unittest.mock import patch
from urllib.error import HTTPError, URLError

from rrm.visual_service import probe_visual_service


class Response(io.BytesIO):
    pass


class VisualServiceTests(unittest.TestCase):
    def declared(self):
        return {"schema_version": "rrm-cosmos-capabilities/v1", "execution_dispatch": False,
                "routes": ["/v1/propose", "/v1/verify-entities"], "worker_source_sha256": "a" * 64}

    def check(self, payload):
        with patch("rrm.visual_service.urlopen", return_value=Response(json.dumps(payload).encode())) as send:
            result = probe_visual_service("http://worker:8090")
            request = send.call_args.args[0]
            self.assertEqual(request.get_method(), "GET")
            self.assertEqual(request.full_url, "http://worker:8090/v1/capabilities")
            self.assertIsNone(request.data)
            return result

    def test_declared_api_is_not_model_or_visual_qualification(self):
        result = self.check(self.declared())
        self.assertEqual(result["status"], "DECLARED")
        self.assertTrue(result["entity_api_declared"])
        self.assertFalse(result["execution_dispatch"])
        self.assertIn("still require verification", result["reason"])

    def test_health_legacy_missing_route_and_bad_identity_are_blocked(self):
        for payload in ({"status": "ready", "execution_dispatch": False}, [], None,
                        {**self.declared(), "routes": []},
                        {**self.declared(), "execution_dispatch": True},
                        {**self.declared(), "worker_source_sha256": "bad"}):
            with self.subTest(payload=payload):
                self.assertFalse(self.check(payload)["entity_api_declared"])

    def test_transport_errors_are_bounded_and_do_not_expose_private_details(self):
        for error in (HTTPError("http://secret", 404, "secret", {}, None), URLError("secret"), TimeoutError()):
            with patch("rrm.visual_service.urlopen", side_effect=error):
                result = probe_visual_service("http://worker:8090")
                self.assertEqual(result["status"], "BLOCKED")
                self.assertNotIn("secret", json.dumps(result))
        with patch("rrm.visual_service.urlopen", return_value=Response(b" " * 4097)):
            self.assertEqual(probe_visual_service("http://worker:8090")["status"], "BLOCKED")

    def test_unconfigured_and_invalid_urls_do_not_send_requests(self):
        with patch("rrm.visual_service.urlopen") as send:
            for url in ("", "https://worker", "http://user:password@worker", "http://worker/path"):
                self.assertEqual(probe_visual_service(url)["status"], "BLOCKED")
            send.assert_not_called()
