import argparse
import json
from pathlib import Path
import socket

import numpy as np


class PolicyClient:
    def __init__(self, assets, socket_path, timeout_s=.02):
        self.assets = json.loads(Path(assets).read_text())
        self.socket_path = socket_path
        self.timeout_s = timeout_s
        self.connection = None
        self.stream = None
        self.reset_history()


    def request(self, payload):
        try:
            if self.connection is None:
                self.connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self.connection.settimeout(self.timeout_s)
                self.connection.connect(self.socket_path)
                self.stream = self.connection.makefile("rb")
            payload["bundle_sha256"] = self.assets["bundle_sha256"]
            self.connection.sendall((json.dumps(payload, allow_nan=False)+"\n").encode())
            response = self.stream.readline(65536)
            if not response.endswith(b"\n"):
                raise RuntimeError("Truncated policy response")
            result = json.loads(response)
            if "error" in result:
                raise RuntimeError(result["error"])
            return result
        except (OSError, ValueError, RuntimeError):
            if self.stream is not None:
                self.stream.close()
            if self.connection is not None:
                self.connection.close()
            self.connection = self.stream = None
            raise


    def reset_history(self):
        self.request({"operation": "reset"})


    def act(self, state, goal_velocity, previous_action):
        result = self.request({"operation": "act", "state": {k: np.asarray(v).tolist() for k, v in state.items()},
                               "goal": np.asarray(goal_velocity).tolist(), "previous": np.asarray(previous_action).tolist()})
        action, physical = np.asarray(result["action"]), np.asarray(result["physical"])
        if action.shape != (4,) or physical.shape != (4,) or not np.isfinite(np.r_[action, physical]).all():
            raise RuntimeError("Invalid policy-worker output")
        return action, physical


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle", required=True)
    parser.add_argument("--assets", required=True)
    parser.add_argument("--socket", required=True, type=Path)
    args = parser.parse_args()
    from gamma_zero_controller.inference import BVPolicy
    policy = BVPolicy(args.assets, args.bundle)
    policy.act({"linear_velocity_flu": np.zeros(3), "angular_velocity_flu": np.zeros(3),
                "projected_gravity_flu": np.array([0., 0., -1.])}, np.zeros(4), np.zeros(4))
    policy.reset_history()
    if args.socket.exists():
        raise ValueError("Socket path already exists; inspect owner before removing it")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
        server.bind(str(args.socket))
        args.socket.chmod(0o600)
        server.listen(1)
        print("Policy worker ready: "+str(args.socket), flush=True)
        try:
            while True:
                connection, _ = server.accept()
                policy.reset_history()
                with connection, connection.makefile("rb") as stream:
                    while True:
                        raw = stream.readline(65536)
                        if not raw:
                            break
                        try:
                            if not raw.endswith(b"\n"):
                                raise ValueError("Oversized policy request")
                            request = json.loads(raw)
                            if request["bundle_sha256"] != policy.assets["bundle_sha256"]:
                                raise ValueError("Worker/client bundles differ")
                            if request["operation"] == "reset":
                                policy.reset_history()
                                result = {"reset": True}
                            elif request["operation"] == "act":
                                action, physical = policy.act(request["state"], request["goal"], request["previous"])
                                result = {"action": action.tolist(), "physical": physical.tolist()}
                            else:
                                raise ValueError("Unknown operation")
                        except (ValueError, RuntimeError, KeyError, TypeError) as exc:
                            policy.reset_history()
                            result = {"error": str(exc)}
                        try:
                            connection.sendall((json.dumps(result, allow_nan=False)+"\n").encode())
                        except OSError:
                            break
        finally:
            args.socket.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
