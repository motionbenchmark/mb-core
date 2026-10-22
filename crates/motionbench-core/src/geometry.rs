use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Axis-aligned rectangle in arena coordinates (metres). `x`,`y` = lower-left corner.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Rect {
    pub x: f64,
    pub y: f64,
    pub w: f64,
    pub h: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

impl Rect {
    pub fn new(x: f64, y: f64, w: f64, h: f64) -> Self {
        Rect {
            x,
            y,
            w,
            h,
            label: None,
        }
    }

    pub fn contains(&self, px: f64, py: f64) -> bool {
        px >= self.x && px <= self.x + self.w && py >= self.y && py <= self.y + self.h
    }

    pub fn center(&self) -> (f64, f64) {
        (self.x + self.w / 2.0, self.y + self.h / 2.0)
    }

    pub fn area(&self) -> f64 {
        self.w * self.h
    }

    /// True when a circle of radius `r` centred at (cx, cy) overlaps this rectangle.
    pub fn overlaps_circle(&self, cx: f64, cy: f64, r: f64) -> bool {
        let nx = cx.clamp(self.x, self.x + self.w);
        let ny = cy.clamp(self.y, self.y + self.h);
        dist(cx, cy, nx, ny) < r
    }
}

pub fn dist(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt()
}

/// Absolute smallest difference between two headings, in radians, in `[0, π]`.
pub fn angle_diff(a: f64, b: f64) -> f64 {
    let mut d = (a - b) % (2.0 * PI);
    if d > PI {
        d -= 2.0 * PI;
    } else if d < -PI {
        d += 2.0 * PI;
    }
    d.abs()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angle_wraps() {
        assert!((angle_diff(0.1, 2.0 * PI - 0.1) - 0.2).abs() < 1e-9);
        assert!((angle_diff(PI, -PI)).abs() < 1e-9);
    }

    #[test]
    fn circle_overlap() {
        let r = Rect::new(1.0, 1.0, 1.0, 1.0);
        assert!(r.overlaps_circle(0.9, 1.5, 0.2));
        assert!(!r.overlaps_circle(0.5, 1.5, 0.2));
    }
}
