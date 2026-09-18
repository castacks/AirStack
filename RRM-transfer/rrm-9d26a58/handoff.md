## PSC Submission Blocking Issue (2026-09-18)
The user is unable to submit the inference request to PSC using the `rrm_psc_bridge_manual.sh` script.

### Sequence of Failures:
1. **Background SSH restriction:** Initial attempts using `ssh -fN` (SSH multiplexing with `ControlMaster`) were immediately killed by PSC's firewall, because PSC does not allow interactive background SSH sockets that don't allocate a PTY or run a command.
2. **Rate limiting:** Repeated attempts triggered a temporary IP ban from PSC (`Connection reset by peer`) due to fail2ban or similar firewall rules catching the rapid succession of aborted connections.
3. **Data Transfer Node constraint:** We discovered the script was pointing to `data.bridges2.psc.edu` (the DTN). The DTN blocks shell execution (`Login denied: Only file transfers are allowed on this account`), which caused our pipe `tar ... | ssh ... sbatch` to fail.
4. **Current State:** We switched the host in the script to `bridges2.psc.edu` (the login node), which should allow shell execution. However, the user wants to pause here.

### Next Steps when returning:
- Test the updated `rrm_psc_bridge_manual.sh` script again. It now points to `bridges2.psc.edu` and uses a single `tar` stream over SSH to avoid multiplexing and DTN restrictions.
- If it still fails, check if PSC requires a specific 2FA workflow that breaks `stdin` piping, or if the user's account is completely restricted from `sbatch` via SSH.
