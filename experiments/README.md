# Experiments

experiments/ contains configs and results for each experiment

Running something from experiments should exactly reproduce that experiment

The experiments/ are .gitignored from the code base. 
Instead, the experiments should be backed up to external data storage or a server.


launching will automatically generate a folder of this structure and log under experiments/

```
experiments/
- YYYY-MM-DD-HH-MM-SS_Experiment_Name/
    - field_test_plan.pdf
    - launch_params/
        - launch_command.txt
        - ros_params_dump.txt
        - environment_variables.txt
    - raw/
        - code-config-and-calibration/
            - the_code.zip
        - run/
            - robot1/
                - device-date/
                    - yyyy-mm-dd-hh-mm-ss_raw.mcap
                    - logs/
            - gcs/
                - device-date/
                    - yyyy-mm-dd-hh-mm-ss_raw.mcap
                    - logs/
    - processed/
        - run/
            - device/
                - device-date/
                    - yyyy-mm-dd-hh-mm-ss_processed.mcap
                    - logs/
    - media/
        - yyyy-mm-dd-hh-mm-ss_description.mp4
        - yyyy-mm-dd-hh-mm-ss_description.jpg

- YYYY-MM-DD-HH-MM-SS_CFA_[description]
```