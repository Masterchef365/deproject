Project black
Estimate average and stddev of all pixels. This means storing the data for each pixel. Not an issue. OpenCV!
Project white. Do the same.
Determine if the brightness went UP for each pixel, within a given CI

Store a configuration with the run name, number
Pattern names in the following format:
* `A_B_C`. That maps to a particular configuration.

```rust
struct CapturePlan {
    /// Maximum granularity
    max_step: usize,
    // n_segments: usize, = 2
    // step by power law for starters
    /// How many frames to record for each pattern
    n_samples: usize,
}
```

`cargo run --bin record`
`cargo run --bin calib`
`cargo run --bin demo`
`cargo run --bin slime`

For the recorder, have dual system. The main thread will do the graphics. The other thread will do the capturing and saving.
