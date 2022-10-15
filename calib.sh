root_path=$1
cargo run --release --bin calib $root_path 2.5
python3 calib.py $root_path
