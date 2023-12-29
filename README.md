# How to conduct tracking using M3T

## Move 

## Generate config

Generate config
```bash
python scripts/generate_tracker_config.py
```

Guess initial pose
```bash
python scripts/init_pose_guess.py
```

## Run tracking 

```bash
./build/examples/run_generated_tracker ./test_data/config/0000/config.yaml >> test_data/log/0000.log
```

## Parse log

```bash
python scripts/log_parse.py --log_dir=./test_data/log --sequence_id=0
```

Now the poses are saved at `test_data/parsed_data.json`.