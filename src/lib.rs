#[allow(dead_code)]
pub mod differential_drive;

use approx::relative_eq;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn initialize_check_propagation() {
        let mut drive = differential_drive::State::new(0.0, 0.0, 0.0);
        drive.update_linear_velocity(1.0);
        // 1.0 m/s in the linear x direction
        drive.update_angular_velocity(2.0*3.1415916);


        let result = drive.propagate(1.0/4.0);
        // propagate the state 1 second into the future

        println!("{}", result);
        relative_eq!(result.x, 1.0, epsilon=0.0001);
    }
}
