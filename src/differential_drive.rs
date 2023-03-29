use nalgebra::{Matrix3, Vector2, Vector3};

pub struct State {
    x: f64,
    y: f64,
    yaw: f64,
    linear_velocity: f64,
    angular_velocity: f64,
    icc: Option<Vector2<f64>>,
}

// static TIME_RESOLUTION: f64 = 0.000001;
// static TIME_RESOLUTION: f64 = 0.00001;

impl State {
    pub fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self {
            x,
            y,
            yaw,
            linear_velocity: 0f64,  // in the direction of the +x axis
            angular_velocity: 0f64, // in the direction of the yaw axis, i.e., +z axis
            icc: None,
        }
    }

    fn update_icc(&mut self) {
        let curvature_radius: f64 = self.linear_velocity / self.angular_velocity;

        if curvature_radius.abs() == f64::INFINITY {
            self.icc = None;
        } else {
            self.icc = Some(Vector2::new(
                self.x - curvature_radius * (self.yaw.sin()),
                self.y + curvature_radius * (self.yaw.cos()),
            ));
        }
    }

    pub fn update_linear_velocity(&mut self, linear_vel_: f64) {
        self.linear_velocity = linear_vel_;
        self.update_icc();
    }

    pub fn update_angular_velocity(&mut self, angular_vel_: f64) {
        self.angular_velocity = angular_vel_;
        self.update_icc();
    }

    fn propagate_step(&mut self, dt: f64) {
        // wdt = `omega`*dt
        // the angular velocity times dt
        let wdt: f64 = self.angular_velocity * dt;
        let propagation_matrix = Matrix3::new(
            wdt.cos(), (-wdt).sin(), 0f64,
            wdt.sin(), wdt.cos(), 0f64,
            0f64, 0f64, 1f64,
        );
        if let Some(icc_vector) = self.icc {
            let mut position_states = Vector3::new(self.x, self.y, self.yaw);
            position_states = propagation_matrix
                * Vector3::new(self.x - icc_vector[0], self.y - icc_vector[1], self.yaw)
                + Vector3::new(icc_vector[0], icc_vector[1], wdt);

            self.x = position_states[0];
            self.y = position_states[1];
            self.yaw = position_states[2];
            self.update_icc();
        } else {
            // icc does not exist, so it is inf, or hasn't been initialized
            // in either case, we assume constant angular velocity
            self.x += (self.linear_velocity*dt)* self.yaw.cos();
            self.y += (self.linear_velocity*dt) * self.yaw.sin();
            // println!("{} {}", self.x, self.y);
        }
    }

    pub fn propagate(&mut self, del_t: f64) -> Vector3<f64> {
        // let iterations: u64 = (del_t/TIME_RESOLUTION) as u64;
        // for _ in 0..iterations {
            self.propagate_step(del_t);
            // self.propagate_step(TIME_RESOLUTION);
        // }

        Vector3::new(self.x, self.y, self.yaw)
    }
}
