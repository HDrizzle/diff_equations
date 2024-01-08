use diff_equations::prelude::*;

fn main() {
    let stepper: Stepper<2, spring::StaticSpringAndMass> = Stepper::<2, spring::StaticSpringAndMass>::new(0.01);
    diff_equations::gui::main(stepper);
}