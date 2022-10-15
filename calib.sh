root_path=$1
cargo run --release --bin calib $root_path 0.5
python3 calib.py $root_path
