// For static iformation loaded from simulation file
use serde::{Serialize, Deserialize};
use crate::prelude::*;

trait ComponentSolver {
	
}

struct GenericComponent {
	nodes: Vec<Node>
}

struct Node {
	id: u32
}

struct CircuitStatic {
	nodes: Vec<Node>,

}