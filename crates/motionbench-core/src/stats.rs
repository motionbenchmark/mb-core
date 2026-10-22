//! Small, dependency-free statistics with a deterministic RNG so every
//! binding reproduces the same confidence intervals.

pub fn mean(v: &[f64]) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.iter().sum::<f64>() / v.len() as f64
}

/// Sample standard deviation (n − 1). Zero for fewer than two values.
pub fn std_dev(v: &[f64]) -> f64 {
    if v.len() < 2 {
        return 0.0;
    }
    let m = mean(v);
    (v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / (v.len() - 1) as f64).sqrt()
}

/// Linear-interpolated percentile, `q` in [0, 1].
pub fn percentile(v: &[f64], q: f64) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    let mut s = v.to_vec();
    s.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let pos = q.clamp(0.0, 1.0) * (s.len() - 1) as f64;
    let lo = pos.floor() as usize;
    let hi = pos.ceil() as usize;
    s[lo] + (s[hi] - s[lo]) * (pos - lo as f64)
}

pub fn median(v: &[f64]) -> f64 {
    percentile(v, 0.5)
}

/// SplitMix64: tiny, well-distributed, and identical on every platform.
pub struct SplitMix64(u64);

impl SplitMix64 {
    pub fn new(seed: u64) -> Self {
        SplitMix64(seed)
    }
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    pub fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }
}

/// Stable 64-bit FNV-1a hash, used to seed bootstrap resampling per (task, agent).
pub fn fnv1a(s: &str) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in s.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    h
}

/// Percentile bootstrap 95% CI of the mean.
pub fn bootstrap_ci95(v: &[f64], iters: usize, seed: u64) -> (f64, f64) {
    if v.is_empty() {
        return (f64::NAN, f64::NAN);
    }
    if v.len() == 1 {
        return (v[0], v[0]);
    }
    let mut rng = SplitMix64::new(seed);
    let mut means = Vec::with_capacity(iters);
    for _ in 0..iters {
        let mut s = 0.0;
        for _ in 0..v.len() {
            s += v[rng.below(v.len())];
        }
        means.push(s / v.len() as f64);
    }
    (percentile(&means, 0.025), percentile(&means, 0.975))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basics() {
        let v = [1.0, 2.0, 3.0, 4.0];
        assert_eq!(mean(&v), 2.5);
        assert_eq!(median(&v), 2.5);
        assert!((std_dev(&v) - 1.2909944).abs() < 1e-6);
        let (lo, hi) = bootstrap_ci95(&v, 2000, 7);
        assert!(lo < 2.5 && hi > 2.5);
        // deterministic
        assert_eq!(bootstrap_ci95(&v, 2000, 7), (lo, hi));
    }
}
