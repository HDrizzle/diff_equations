use std::collections::HashMap;

// Implementation of super::Component
use serde::{Serialize, Deserialize};
use crate::prelude::*;
use super::Component;

pub struct Resistor {
    value: Float
}

impl Component for Resistor {
    fn state_representation_vec_size(&self) -> usize {
        0
    }
    fn initial_state(&self) -> VDyn {
        VDyn::from_vec(Vec::new())
    }
    fn voltage_on_other_nodes(&mut self, node: usize, voltage: Float) -> std::collections::HashMap<usize, Float> {
        HashMap::new()// TODO
    }
}
