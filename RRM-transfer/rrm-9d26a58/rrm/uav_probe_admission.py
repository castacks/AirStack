"""Read-only server discovery followed by caller-owned probe admission checks."""


def wait_for_checked_server(client, check=None, *, timeout_sec=5):
    """Never treat server discovery as freshness or motion authorization."""
    if not client.wait_for_server(timeout_sec=timeout_sec):
        raise RuntimeError('action_server_missing')
    if check is not None:
        check()
