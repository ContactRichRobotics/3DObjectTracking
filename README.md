# How to conduct tracking using M3T

## Generate config

```bash
python scripts/generate_tracker_config.py
```

## Run tracking 

```bash
cd ${BUILD_DIR}
./build/examples/run_generated_tracker ./test_data/config/0000/config.yaml
```