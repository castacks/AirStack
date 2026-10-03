"""Raw trial logs retain state, commands, identity, and a terminal reason."""
import json
from pathlib import Path

from svg_ground_control.deploy.trial_log import TrialLog


def test_trial_log_round_trip(tmp_path):
    trial = TrialLog(str(tmp_path), {'policy': 'diffaero', 'mode': 'sim'})
    trial.write('state', time_s=1.5, position_enu=[1, 2, 3])
    trial.write('policy_command', time_s=1.5, setpoint={'type': 'velocity'})
    trial.close('operator_hold', time_s=2.0)
    trial.close('ignored')
    rows = [json.loads(line) for line in Path(trial.path).read_text().splitlines()]
    assert [row['kind'] for row in rows] == ['start', 'state', 'policy_command', 'end']
    assert rows[0]['metadata'] == {'policy': 'diffaero', 'mode': 'sim'}
    assert rows[-1]['reason'] == 'operator_hold'
